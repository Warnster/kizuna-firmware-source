/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* charmap.h — GBA Gen-3 (English) name encode/decode, portable C port of frlgsim/charmap.py.
 * Terminator/pad = 0xFF; LinkPlayer/uname fields pad with 0x00 after the terminator. */
#ifndef CHARMAP_H
#define CHARMAP_H
#include <stdint.h>
#include <stddef.h>

#define CHARMAP_EOS 0xFF

/* char -> GBA byte, or -1 if unencodable (dropped). */
int charmap_encode_char(char c);
/* byte -> ASCII char ('.' for unknown, ' ' for 0x00). */
char charmap_decode_byte(uint8_t b);

/* Encode `s`. If width>0: truncate to width-1, append EOS, pad to width with `pad`; returns width.
 * If width<0: raw encode (no terminator), returns count. `out` must be big enough. */
size_t charmap_encode(const char *s, int width, uint8_t pad, uint8_t *out);
/* Decode n bytes to a C string (NUL-terminated). Stops at EOS if stop_at_eos. */
size_t charmap_decode(const uint8_t *b, size_t n, char *out, int stop_at_eos);

#endif
