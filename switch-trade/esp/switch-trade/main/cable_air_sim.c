/* cable_air_sim — see cable_air_sim.h. Thin sim_engine adapter over cable_air. */
#include "cable_air_sim.h"
#include "gbaframe.h"   /* gba_in */
#include "ni.h"         /* NI_ACTIVITY_TRADE, NI_LANGUAGE_ENGLISH */
#include <string.h>
#include <stdio.h>

static void refresh_identity(struct cable_air_sim *a)
{
    if (a->identity_set || !a->air) return;
    uint16_t tid;
    if (cable_air_get_identity(a->air, &tid, a->name8)) {
        a->engine.lp_trainer_id = tid;
        memcpy(a->engine.lp_name_raw, a->name8, 8);
        a->engine.lp_name_raw_valid = 1;   /* raw FRLG-charmap bytes from the cart */
        a->identity_set = 1;
    }
}

void cable_air_sim_seed_identity(struct cable_air_sim *a, uint16_t tid, const uint8_t name8[8])
{
    if (a->identity_set) return;
    a->engine.lp_trainer_id = tid;
    memcpy(a->name8, name8, 8);
    memcpy(a->engine.lp_name_raw, name8, 8);
    a->engine.lp_name_raw_valid = 1;
    a->identity_set = 1;
}

void cable_air_sim_reset_identity(struct cable_air_sim *a)
{
    a->identity_set = 0;
    a->engine.lp_trainer_id = 0x8822;
    a->engine.lp_name_raw_valid = 0;
}

/* sim_engine.tick: the next child UNI trade slot (idle = all zero). */
static void eng_tick(void *ctx, uint16_t words[7])
{
    struct cable_air_sim *a = ctx;
    if (!a->air) { memset(words, 0, 7 * sizeof(uint16_t)); return; }
    refresh_identity(a);
    cable_air_engine_tick(a->air, words);
}

/* sim_engine.feed_in_frame: the leader's slot0 (command) and slot1 (its reflection of our block). */
static void eng_feed(void *ctx, const gba_in *rec)
{
    struct cable_air_sim *a = ctx;
    static const uint8_t zero[14] = { 0 };
    if (!a->air) return;
    const uint8_t *s0 = rec->nslots >= 1 ? rec->slots[0] : zero;
    const uint8_t *s1 = rec->nslots >= 2 ? rec->slots[1] : zero;
    cable_air_engine_feed_slots(a->air, s0, s1);
}

/* sim_engine.poll_send_done: used as the per-tick identity refresh before sim builds its NI. */
static void eng_poll(void *ctx) { refresh_identity(ctx); }

void cable_air_sim_set_air(struct cable_air_sim *a, struct CableAir *air)
{
    a->air = air;
    cable_air_sim_reset_identity(a);
}

sim_engine *cable_air_sim_bind(struct cable_air_sim *a, struct CableAir *air)
{
    memset(a, 0, sizeof(*a));
    a->air = air;
    a->engine.tick = eng_tick;
    a->engine.feed_in_frame = eng_feed;
    a->engine.poll_send_done = eng_poll;
    a->engine.has_backlog = NULL;
    a->engine.ctx = a;
    a->engine.in_seat_phase = 0;          /* relay */
    a->engine.established = 0;            /* relay */
    a->engine.lp_version_low = VERSION_EMERALD;   /* Ruby/Sapphire shows as Emerald to the Switch */
    a->engine.ni_compat_or = NI_COMPAT_NATIONAL;
    a->engine.lp_trainer_id = 0x8822;     /* until the cart's LinkPlayer arrives */
    a->engine.lp_name = "Kizuna";
    a->engine.lp_language = NI_LANGUAGE_ENGLISH;
    a->engine.ni_activity = NI_ACTIVITY_TRADE;
    a->engine.ni_started = 1;
    return &a->engine;
}
