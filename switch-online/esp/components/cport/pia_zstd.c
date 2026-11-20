/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
#include "pia_zstd.h"
#include <string.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"   /* CONFIG_CPORT_ZSTD_NO_COMPRESS */
#endif
#define ZSTD_STATIC_LINKING_ONLY   /* ZSTD_initStaticDCtx / ZSTD_estimateDCtxSize */
#include <zstd.h>

/* ONE decoder context for the whole session, in a static workspace. ZSTD_decompress() builds and frees
 * a context on every call: 96 KB with zstd's default 64 KB literal buffer, ~31 KB with
 * ZSTD_DECODER_INTERNAL_BUFFER=1024 (set by the ESP zstd component). On the ESP32-C3 the largest free
 * heap block after Wi-Fi is up is ~69 KB, so the default never fits and even the small one must not be
 * re-allocated per datagram. If the workspace is too small for the linked library (host builds against a
 * stock libzstd), fall back to one heap-allocated context, created once. */
#define PIA_ZSTD_DCTX_WS 32768
/* PIA_ZSTD_SESSION_HEAP: the decoder workspace lives on the heap only while a session needs it --
 * pia_zstd_reserve() right after Bluetooth has released its memory, pia_zstd_release() before Bluetooth starts again --
 * because the C3 cannot hold NimBLE and the workspaces at once. Same fixed size and ZSTD_initStaticDCtx either way. */
#ifdef PIA_ZSTD_SESSION_HEAP
#if defined(ESP_PLATFORM) && !defined(CONFIG_CPORT_ZSTD_NO_COMPRESS)
#error "PIA_ZSTD_SESSION_HEAP needs CONFIG_CPORT_ZSTD_NO_COMPRESS=y: only the decoder is session-heap; a compressor would stay static"
#endif
#include <stdlib.h>
static uint8_t  *s_dctx_ws;
static size_t    s_dctx_ws_len;
/* Session heap: exactly what the linked decoder needs (~31.4 KB), not the fixed 32 KB. Bench: after one
 * Ruby/Sapphire visit the largest block was exactly 32768 and the 32 KB request missed it by its heap overhead. */
static size_t ws_len(void) {
    size_t n = ZSTD_estimateDCtxSize();
    if (n > PIA_ZSTD_DCTX_WS) n = PIA_ZSTD_DCTX_WS;   /* a stock libzstd needs more: dctx_get falls back below */
    return (n + 7) & ~(size_t)7;
}
static void ws_alloc(void) {
    if (s_dctx_ws) return;
    s_dctx_ws_len = ws_len();
    s_dctx_ws = aligned_alloc(8, s_dctx_ws_len);
}
#else
static uint8_t   s_dctx_ws[PIA_ZSTD_DCTX_WS] __attribute__((aligned(8)));
#endif
static ZSTD_DCtx *s_dctx;
static int        s_dctx_static;

static ZSTD_DCtx *dctx_get(void) {
    if (s_dctx) return s_dctx;
#ifdef PIA_ZSTD_SESSION_HEAP
    ws_alloc();                                                         /* lazy fallback if nobody reserved */
    if (!s_dctx_ws) return NULL;
    const size_t ws_size = s_dctx_ws_len;
#else
    const size_t ws_size = PIA_ZSTD_DCTX_WS;
#endif
    if (ZSTD_estimateDCtxSize() <= ws_size) {
        s_dctx = ZSTD_initStaticDCtx(s_dctx_ws, ws_size);
        s_dctx_static = s_dctx != NULL;
    }
    if (!s_dctx) s_dctx = ZSTD_createDCtx();
    return s_dctx;
}

int pia_zstd_reserve(void) {
#ifdef PIA_ZSTD_SESSION_HEAP
    ws_alloc();
    return s_dctx_ws != NULL;
#else
    return 1;
#endif
}

void pia_zstd_release(void) {
#ifdef PIA_ZSTD_SESSION_HEAP
    if (s_dctx && !s_dctx_static) ZSTD_freeDCtx(s_dctx);
    s_dctx = NULL; s_dctx_static = 0;
    free(s_dctx_ws); s_dctx_ws = NULL; s_dctx_ws_len = 0;
#endif
}

size_t pia_zstd_dctx_bytes(int *is_static) {
    if (is_static) *is_static = s_dctx_static;
    return ZSTD_estimateDCtxSize();
}

static const uint8_t ZSTD_MAGIC[4] = {0x28, 0xb5, 0x2f, 0xfd};
#define WINDOW_DESC 0x18   /* the window descriptor the console uses */

const char *pia_zstd_last_error = "";   /* set to ZSTD_getErrorName on a decompress failure */

long pia_decompress(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    if (in_len < 4 || memcmp(in, ZSTD_MAGIC, 4) != 0) {   /* not zstd -> pass through */
        if (in_len > out_cap) return -1;
        memcpy(out, in, in_len);
        return (long)in_len;
    }
    /* The Pia plaintext often carries a trailing station footer + 0xFF padding AFTER the zstd
     * frame; one-shot decode of the whole buffer errors on the trailing bytes, so find the exact
     * frame size first, then decode only that, on the session's single decoder context. */
    size_t fcs = ZSTD_findFrameCompressedSize(in, in_len);
    if (ZSTD_isError(fcs)) { pia_zstd_last_error = ZSTD_getErrorName(fcs); return -1; }
    ZSTD_DCtx *d = dctx_get();
    if (!d) { pia_zstd_last_error = "no decoder context"; return -1; }
    size_t r = ZSTD_decompressDCtx(d, out, out_cap, in, fcs);
    if (ZSTD_isError(r)) { pia_zstd_last_error = ZSTD_getErrorName(r); return -1; }
    return (long)r;
}

#ifdef CONFIG_CPORT_ZSTD_NO_COMPRESS
/* No compressor, no 40 KB workspace (see Kconfig). pia_compress() still returns a VALID zstd frame: the stored (raw block) form in
 * the console's canonical header `28 b5 2f fd 00 18 <3-byte block header> <bytes>` -- the same header the real compressor writes
 * (_to_window_frame), only with the data uncompressed. Any zstd decoder accepts it; the Switch accepted it from the host role on
 * every hosted trade (pia_host_zstd_stored). Returns -1 when the input does not fit (window 8 KiB, or out_cap < in_len + 9), and the
 * sim then falls back to a raw datagram as it always has. */
long pia_compress(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    static const uint8_t prefix[6] = { 0x28, 0xb5, 0x2f, 0xfd, 0x00, 0x18 };
    if (in_len > 8192 || out_cap < in_len + 9) return -1;
    uint32_t b = ((uint32_t)in_len << 3) | 1u;      /* block header: size << 3 | last-block bit | raw type (0) */
    memmove(out + 9, in, in_len);                    /* in may alias out + 9 */
    memcpy(out, prefix, 6);
    out[6] = (uint8_t)b; out[7] = (uint8_t)(b >> 8); out[8] = (uint8_t)(b >> 16);
    return (long)(in_len + 9);
}
#else
/* Rewrite ZSTD_compress()'s frame header to the console's canonical `magic 00 18 <blocks>`
 * form — the C port of _to_window_frame(). The rewrite only ever shrinks the header, so it's
 * done in place (blocks shift left). */
/* One persistent compressor context as well: ZSTD_compress() allocates and frees a ~31-58 KB workspace
 * (level 4, 100-1600 B inputs) on EVERY call; a kept context grows its workspace once and reuses it. */
/* Level 4 with the window/hash/chain logs zstd itself picks for a few-hundred-byte input (10/10/9): that
 * bounds the workspace at ~36 KB for ANY input size (plain level 4 would grow it to 58 KB at 1.4 KB inputs,
 * the sim's SIM_MAXAPP). A 1 KB window only costs ratio on bodies longer than 1 KB; the frame stays valid. */
#define PIA_ZSTD_CCTX_WS (40 * 1024)
static uint8_t   s_cctx_ws[PIA_ZSTD_CCTX_WS] __attribute__((aligned(8)));
static ZSTD_CCtx *s_cctx;

static ZSTD_CCtx *cctx_get(void) {
    if (s_cctx) return s_cctx;
    s_cctx = ZSTD_initStaticCCtx(s_cctx_ws, sizeof s_cctx_ws);
    if (!s_cctx) s_cctx = ZSTD_createCCtx();     /* workspace too small for this libzstd: heap, once */
    if (!s_cctx) return NULL;
    ZSTD_CCtx_setParameter(s_cctx, ZSTD_c_compressionLevel, 4);
    ZSTD_CCtx_setParameter(s_cctx, ZSTD_c_windowLog, 10);
    ZSTD_CCtx_setParameter(s_cctx, ZSTD_c_hashLog, 10);
    ZSTD_CCtx_setParameter(s_cctx, ZSTD_c_chainLog, 9);
    return s_cctx;
}

long pia_compress(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    ZSTD_CCtx *c = cctx_get();
    if (!c) return -1;
    ZSTD_CCtx_reset(c, ZSTD_reset_session_only);
    size_t r = ZSTD_compress2(c, out, out_cap, in, in_len);
    if (ZSTD_isError(r)) return -1;
    if (r < 6 || memcmp(out, ZSTD_MAGIC, 4) != 0) return (long)r;

    uint8_t fhd = out[4];                       /* Frame Header Descriptor */
    if ((fhd & 0x03) || (fhd & 0x04)) return (long)r;   /* dict id / reserved -> leave as-is */
    int fcs_flag = fhd >> 6;
    static const int ss_sizes[4] = {1, 2, 4, 8};   /* single-segment: content-size field width */
    static const int ms_sizes[4] = {0, 2, 4, 8};   /* multi-segment:  content-size field width */
    size_t blocks_off;
    if (fhd & 0x20) {                           /* single segment (no window byte) */
        blocks_off = 5 + ss_sizes[fcs_flag];
    } else {
        if (out[5] > WINDOW_DESC) return (long)r;   /* window already larger -> don't shrink */
        blocks_off = 6 + ms_sizes[fcs_flag];
    }
    if (blocks_off > r) return -1;
    size_t blocks_len = r - blocks_off;
    memmove(out + 6, out + blocks_off, blocks_len);
    out[4] = 0x00;
    out[5] = WINDOW_DESC;
    return (long)(6 + blocks_len);
}
#endif /* CONFIG_CPORT_ZSTD_NO_COMPRESS */
