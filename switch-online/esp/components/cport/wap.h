/* wap.h — Wireless Adapter Protocol command layer (the 0x9966 packets the GBA firmware
 * forwards). Mirrors host/pico_host.py's parse. Wire: one header word
 *   header = (0x9966 << 16) | (word_count << 8) | command
 * followed by `word_count` u32 LE data words. Self-framing. */
#ifndef WAP_H
#define WAP_H
#include <stdint.h>

#define WAP_MAGIC 0x9966u

typedef struct {
    uint8_t command;    /* header & 0xFF (includes the 0x80 "is response" bit) */
    uint8_t word_count; /* (header >> 8) & 0xFF */
    int is_response;    /* command & 0x80 */
} wap_header;

/* Parse a header word. Returns 1 if the 0x9966 magic matches, else 0. */
int wap_parse_header(uint32_t header, wap_header *out);

/* Human name for a command (masks off the response bit). Never NULL. */
const char *wap_command_name(uint8_t command);

#endif
