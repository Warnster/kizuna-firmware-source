/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* pia_zstd.h — zstd (de)compression for the Pia application payload, C port of the
 * compress()/decompress() in frlgsim crypto.py. The console emits stock zstd frames (no dict)
 * rewritten to the window-descriptor form `28b52ffd 00 18 …`; we produce the same on compress
 * and transparently pass through non-zstd payloads on decompress.
 *
 * Backed by libzstd (host: -lzstd; ESP32: the espressif/zstd managed component). */
#ifndef PIA_ZSTD_H
#define PIA_ZSTD_H
#include <stdint.h>
#include <stddef.h>

/* Decompress a Pia payload. If `in` is not a zstd frame (no 28b52ffd magic) it is copied
 * through unchanged (matches decompress()'s no-op). Returns output length, or -1 on error
 * (corrupt frame, or out_cap too small). */
extern const char *pia_zstd_last_error;   /* ZSTD_getErrorName of the last decompress failure */
long pia_decompress(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap);

/* Compress app bytes into the console's zstd frame form (level 4, window descriptor 0x18,
 * no content size). Returns output length, or -1 on error / out_cap too small. */
long pia_compress(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap);

/* Size of the decoder context the linked zstd needs, and whether it lives in pia_zstd's static
 * workspace (1) or had to be heap-allocated once (0). Diagnostics for the ESP32-C3 memory budget. */
size_t pia_zstd_dctx_bytes(int *is_static);
/* Session-heap mode (PIA_ZSTD_SESSION_HEAP): reserve both workspaces now (1 = ok), release them when the session is
 * over. Without the define both are no-ops returning success, and the workspaces are static. */
int  pia_zstd_reserve(void);
void pia_zstd_release(void);

#endif
