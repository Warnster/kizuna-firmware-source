/* trade_session — FRLG room discovery + the LDN join/seat/brain state machine (from an earlier trade_scan.c,
 * ported for the two-chip board). In M1 the GBA-side triggers come from the console; in M2 they come from
 * the RP2040's RFU1 traffic. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ldn_advert.h"

void trade_session_start(void);                 /* once, after esp_wifi_start(): sniffer + join init + task */
void trade_session_set_keys(const ldn_keys *k); /* RAM-only keys; NULL clears (discovery stops decrypting) */
bool trade_session_arm(void);                   /* "the GBA is browsing": discover + associate. false = no keys */
void trade_session_disarm(void);                /* back to idle: end any session, promiscuous off, scan task parks */
bool trade_session_armed(void);
bool trade_session_has_keys(void);
/* Radio hand-over requests: true = a GBA is past login and we have keys -> the mode should bring
 * Wi-Fi up (and Bluetooth down) then arm; false = the GBA is gone with no session running -> idle again. Called
 * from the RP2040 bridge's poll context: do the work elsewhere. */
void trade_session_set_radio_cb(void (*cb)(bool session));
void trade_session_select(void);                /* "the GBA selected the Switch's room": start the sim join */
void trade_session_stop(void);                  /* end the session, full re-arm back to scanning */
void trade_session_status(char *out, size_t cap);   /* one JSON line */

/* Kept from trade_scan.c (room identity for the GBA-facing side). */
bool trade_room_identity(char name_out[9], uint16_t *tid_out);
bool trade_room_identity_raw(uint8_t name_raw[8]);   /* host's in-game name, FRLG charmap */
bool trade_room_present(void);
