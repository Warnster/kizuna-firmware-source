/* relay_engine — a sim_engine whose GBA side is a byte pipe (USB console in T1, a relay later).
 * Replaces the GBA/RP2040 seam when the far end of the trade is not local: the Switch's slots go OUT as
 * {"ev":"slot",...} lines and the partner's slots come IN through relay_engine_push(). Install BEFORE the
 * brain starts (ldn_brain_set_engine); the identity is the Kizuna default (FireRed, English). */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum { RELAY_VIA_USB = 0, RELAY_VIA_BLE = 1 } relay_via_t;
void relay_engine_install(relay_via_t via); /* ldn_brain_set_engine(relay); idempotent (via may be changed) */
bool relay_engine_installed(void);
bool relay_engine_push(const uint8_t slot14[14]);   /* partner -> Switch; false = queue full (dropped) */
void relay_engine_status(char *out, size_t cap);    /* JSON fragment: counters */
