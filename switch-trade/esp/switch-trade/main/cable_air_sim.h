/* cable_air_sim — cable_air as cport/sim's trade-slot engine (option B), for the Ruby/Sapphire
 * build of switch-trade. sim owns the Switch-side NI + reliable + Pia; cable_air produces the child's UNI
 * trade slots (from the cart) and consumes the leader's slots. Ported from an earlier
 * Kizuna cable_air_sim.c. On the two-chip board every cable_air call runs in the brain task
 * (the cable bridge's poll and ldn_brain_tick), so the C6's lock is gone. */
#pragma once

#include <stdint.h>
#include "cable_air.h"
#include "sim.h"       /* cport */

struct cable_air_sim {
    struct CableAir *air;
    sim_engine engine;
    uint8_t name8[8];
    int identity_set;
};

/* Wire `a` as the sim engine over `air` and return the engine to pass to ldn_brain_set_engine(). The cart identity
 * (trainer id + name) is taken from the cart's LinkPlayer as soon as it is known, else the seeded or default one.
 * Cross-gen: presents the cart to the Switch as an English Emerald that can link nationally. */
sim_engine *cable_air_sim_bind(struct cable_air_sim *a, struct CableAir *air);

/* Point the engine at a (new) translator, or NULL while none is allocated: the callbacks then send idle slots and
 * ignore the leader's. Resets the identity. */
void cable_air_sim_set_air(struct cable_air_sim *a, struct CableAir *air);

/* Seed the identity before the cart's LinkPlayer arrives (the join fires during the cable handshake): a cached real
 * identity from an earlier run, so the NI game-data the Switch sees matches the trade slots. No-op once set. */
void cable_air_sim_seed_identity(struct cable_air_sim *a, uint16_t tid, const uint8_t name8[8]);

/* Forget the identity (a new cable session may be a different cart). */
void cable_air_sim_reset_identity(struct cable_air_sim *a);

