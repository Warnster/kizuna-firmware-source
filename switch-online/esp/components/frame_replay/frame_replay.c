/* frame_replay.c -- see include/frame_replay.h. */
#include "frame_replay.h"
#include <string.h>

void fr_init(frame_replay_t *r, int delay_frames)
{
    memset(r, 0, sizeof *r);
    r->delay = r->delay_floor = delay_frames > 0 ? delay_frames : FR_DELAY_DEFAULT;
    r->last_key = -1;
}
void fr_set_delay(frame_replay_t *r, int delay_frames)
{
    if (delay_frames > 0 && delay_frames < FR_N) r->delay = r->delay_floor = delay_frames;
}
bool fr_is_keys(const uint16_t w[7]) { return (w[0] & 0xFF00u) == 0xBE00u; }
int  fr_depth(const frame_replay_t *r) { return (r->head - r->tail + FR_N) % FR_N; }

static uint8_t ctr_of(const uint16_t w[7]) { return (uint8_t)(w[1] >> 8); }
static bool idle_keys(const uint16_t w[7]) { return fr_is_keys(w) && (w[1] & 0xFF) == FR_IDLE_KEY; }
static int32_t due_at(const frame_replay_t *r, int i) { return r->base_due + (r->ext[i] - r->base_ext); }
static void anchor(frame_replay_t *r, int32_t ext, int32_t due) { r->based = true; r->base_ext = ext; r->base_due = due; }
static void fresh(frame_replay_t *r, int32_t ext, int32_t due) { anchor(r, ext, due); r->lead_avg16 = r->delay * 16; r->rebased++; }

/* unwrap the 8-bit counter: consecutive frames differ by a few counts at most, so the shortest signed step is the truth */
static int32_t unwrap(frame_replay_t *r, uint8_t ctr)
{
    if (!r->have_ext) { r->have_ext = true; r->last_ctr = ctr; r->last_ext = ctr; return r->last_ext; }
    r->last_ext += (int8_t)(ctr - r->last_ctr);
    r->last_ctr = ctr;
    return r->last_ext;
}

static void remove_at(frame_replay_t *r, int idx)      /* drop entry idx, shifting the older part up by one */
{
    for (int i = idx; i != r->tail; i = (i + FR_N - 1) % FR_N) {
        int p = (i + FR_N - 1) % FR_N;
        memcpy(r->q[i], r->q[p], 7 * sizeof(uint16_t)); r->arr[i] = r->arr[p]; r->ext[i] = r->ext[p];
    }
    r->tail = (r->tail + 1) % FR_N;
}

void fr_push(frame_replay_t *r, const uint16_t w[7], int32_t now)
{
    bool was_empty = r->head == r->tail;
    int32_t ext = 0;
    if (fr_is_keys(w)) {
        ext = unwrap(r, ctr_of(w));
        if (!r->based || (was_empty && now - r->empty_since > FR_GAP_REBASE)) fresh(r, ext, now + r->delay);
        else {
            int32_t d = r->base_due + (ext - r->base_ext) - now;
            if (d > FR_DISCONT || d < -FR_DISCONT) fresh(r, ext, now + r->delay);   /* a different stream: fresh timeline */
        }
    }
    int nxt = (r->head + 1) % FR_N;
    if (nxt == r->tail) {
        /* full: shed the oldest idle frame that has idle neighbours (an idle run one frame shorter), else the oldest */
        int victim = -1, prev = -1;
        for (int i = r->tail; i != r->head; i = (i + 1) % FR_N) {
            int nx = (i + 1) % FR_N;
            if (prev >= 0 && nx != r->head && idle_keys(r->q[prev]) && idle_keys(r->q[i]) && idle_keys(r->q[nx])) { victim = i; break; }
            prev = i;
        }
        if (victim >= 0) { remove_at(r, victim); r->shed_idle++; }
        else { r->tail = (r->tail + 1) % FR_N; r->drops++; }
    }
    memcpy(r->q[r->head], w, 7 * sizeof(uint16_t));
    r->arr[r->head] = now; r->ext[r->head] = ext;
    r->head = nxt; r->enq++;
}

static void drop_head(frame_replay_t *r, int32_t now)
{
    r->tail = (r->tail + 1) % FR_N;
    if (r->head == r->tail) r->empty_since = now;
}

bool fr_pop(frame_replay_t *r, uint16_t out[7], int32_t now)
{
    if (r->head == r->tail) return false;
    const uint16_t *h = r->q[r->tail];
    if (!fr_is_keys(h)) {                                   /* blocks, standby, close: in order, at once */
        memcpy(out, h, 7 * sizeof(uint16_t));
        drop_head(r, now); r->deq++;
        r->based = false; r->last_key = -1;                 /* the keys timeline restarts after it */
        return true;
    }
    if (!r->based) { fresh(r, r->ext[r->tail], now + r->delay); return false; }
    int32_t d = due_at(r, r->tail) - now;
    if (d > 0) return false;                                /* not yet: hold the last state */
    if (d < -r->delay) {                                    /* stalled past the buffer: play from here, and deepen the buffer */
        fresh(r, r->ext[r->tail], now); r->late++; r->last_late = now;
        if (r->delay + FR_DELAY_GROW <= FR_DELAY_MAX) r->delay += FR_DELAY_GROW;
    } else if (r->delay > r->delay_floor && now - r->last_late > FR_DELAY_DECAY) {
        r->delay--; r->last_late = now;                     /* quiet for a while: one frame shallower */
    }
    /* emit exactly this frame; never collapse. If the frame behind it is already due too (a burst arrived late, or a
     * consumer slower than 60 Hz) move the timeline so it is due on the next pop: frames keep their spacing. */
    uint16_t cand[7]; memcpy(cand, h, sizeof cand);
    int32_t cand_arr = r->arr[r->tail], cand_ext = r->ext[r->tail];
    int32_t due = due_at(r, r->tail);
    drop_head(r, now); r->deq++;
    /* A missed or deferred pop leaves several frames due at once. Frames carrying the SAME key as this one are identical
     * states: keep only the newest, so the timeline stays exact (re-timing them one pop later stretched every press by a
     * frame per missed poll -- bench: "3 steps become 4", 19% of frames). A frame with a DIFFERENT key is a real
     * change: it goes out on the next pop, one frame late. */
    while (r->head != r->tail) {
        const uint16_t *n = r->q[r->tail];
        if (!fr_is_keys(n) || due_at(r, r->tail) - now > 0) break;
        int ck = cand[1] & 0xFF, nk = n[1] & 0xFF;
        if (ck != r->last_key && ck != nk) break;           /* a one-frame event: show it, the rest goes one pop late */
        memcpy(cand, n, sizeof cand); cand_arr = r->arr[r->tail]; cand_ext = r->ext[r->tail]; due = due_at(r, r->tail);
        drop_head(r, now); r->deq++; r->collapsed++;
    }
    anchor(r, cand_ext, due);
    r->last_key = cand[1] & 0xFF;
    if (r->head != r->tail && fr_is_keys(r->q[r->tail]) && due_at(r, r->tail) - now <= 0) {
        anchor(r, r->ext[r->tail], now + 1); r->skipped++;  /* a key change also due: next pop, one frame late */
    }
    /* lead control: average how long frames wait; one-frame slews on an idle frame */
    int32_t lead16 = (due - cand_arr) * 16;
    r->lead_avg16 += (lead16 - r->lead_avg16) / 8;
    if ((cand[1] & 0xFF) == FR_IDLE_KEY) {
        if (r->lead_avg16 < r->delay * 8) { r->base_due += 1; r->lead_avg16 += 16; r->slewed_late++; }   /* one frame of extra hold */
        else {
            /* catch-up: how many frames early we may go = the larger of the lead excess and the backlog excess, but never
             * past the idle run ahead (its idle frames are dropped; the hold repeat shows the same keys), so no pressed run
             * moves relative to its neighbours. A thinned stream has no idle frames queued: then only the counter gap to the
             * next frame (minus one) is available. */
            int excess = (r->lead_avg16 - r->delay * 16) / 16;
            int backlog = fr_depth(r) - (r->delay + FR_DRAIN_MARGIN);
            int want = excess > backlog ? excess : backlog;
            if (want > 0) {
                int shift = 0;
                if (r->head == r->tail) shift = 1;
                else {
                    const uint16_t *n = r->q[r->tail];
                    if (!fr_is_keys(n)) shift = 1;
                    else if (r->ext[r->tail] - cand_ext >= 2) {                 /* thinned stream: the gap itself */
                        int gap = r->ext[r->tail] - cand_ext;
                        shift = want < gap - 1 ? want : gap - 1;
                    } else {                                                    /* dense stream: drop idle frames ahead */
                        while (shift < want && r->head != r->tail && idle_keys(r->q[r->tail])) { drop_head(r, now); r->deq++; r->shed_idle++; shift++; }
                    }
                }
                if (shift > 0) { r->base_due -= shift; r->lead_avg16 -= 16 * shift; r->catchup += (uint32_t)shift; }
            }
        }
    }
    memcpy(out, cand, sizeof cand);
    return true;
}

int fr_due_depth(const frame_replay_t *r, int32_t now)
{
    int n = 0;
    for (int i = r->tail; i != r->head; i = (i + 1) % FR_N) {
        if (!fr_is_keys(r->q[i])) { n++; continue; }
        if (!r->based || due_at(r, i) - now > 0) break;
        n++;
    }
    return n;
}
