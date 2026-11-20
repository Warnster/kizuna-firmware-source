/* SPDX-License-Identifier: AGPL-3.0-only
 * Derived from frlg-ldn-trade (https://github.com/tornadus/frlg-ldn-trade), Copyright (c) the frlg-ldn-trade authors.
 * Changes Copyright (c) 2026 Kizuna. See NOTICE.
 */
/* charmap.c — portable C port of frlgsim/charmap.py. See charmap.h. */
#include "charmap.h"

int charmap_encode_char(char c) {
    if (c >= 'A' && c <= 'Z') return 0xBB + (c - 'A');   /* 0xBB..0xD4 */
    if (c >= 'a' && c <= 'z') return 0xD5 + (c - 'a');   /* 0xD5..0xEE */
    if (c >= '0' && c <= '9') return 0xA1 + (c - '0');   /* 0xA1..0xAA */
    switch (c) {
        case ' ': return 0x00;
        case '!': return 0xAB;
        case '?': return 0xAC;
        case '.': return 0xAD;
        case '-': return 0xAE;
        case ',': return 0xB8;
        case '/': return 0xBA;
        default:  return -1;                              /* unknown -> dropped */
    }
}

char charmap_decode_byte(uint8_t b) {
    if (b == 0x00) return ' ';
    if (b >= 0xA1 && b <= 0xAA) return (char)('0' + (b - 0xA1));
    if (b >= 0xBB && b <= 0xD4) return (char)('A' + (b - 0xBB));
    if (b >= 0xD5 && b <= 0xEE) return (char)('a' + (b - 0xD5));
    switch (b) {
        case 0xAB: return '!'; case 0xAC: return '?'; case 0xAD: return '.';
        case 0xAE: return '-'; case 0xB8: return ','; case 0xBA: return '/';
        default:   return '.';
    }
}

size_t charmap_encode(const char *s, int width, uint8_t pad, uint8_t *out) {
    size_t n = 0;
    for (const char *p = s; *p; p++) {
        int b = charmap_encode_char(*p);
        if (b >= 0) out[n++] = (uint8_t)b;
    }
    if (width > 0) {
        if (n > (size_t)(width - 1)) n = (size_t)(width - 1);
        out[n++] = CHARMAP_EOS;
        while (n < (size_t)width) out[n++] = pad;
        n = (size_t)width;
    } else if (width == 0) {
        n = 0;
    }
    return n;
}

size_t charmap_decode(const uint8_t *b, size_t n, char *out, int stop_at_eos) {
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        if (b[i] == CHARMAP_EOS) { if (stop_at_eos) break; else continue; }
        out[o++] = charmap_decode_byte(b[i]);
    }
    out[o] = 0;
    return o;
}
