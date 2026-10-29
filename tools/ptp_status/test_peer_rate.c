#include <assert.h>
#include <stdio.h>
#include "ptp_peer_rate.h"

static void encode(uint8_t wire[8], int64_t scaled)
{
    uint64_t value = (uint64_t)scaled;
    for (unsigned index = 0; index < 8; ++index) { wire[7-index] = value; value >>= 8; }
}
int main(void)
{
    ptp_peer_rate_t rate = {0};
    uint8_t responder[10] = {1};
    struct timespec remote = {1000000000, 0}, local = {10, 0};
    assert(!ptp_peer_rate_update(&rate, 1, responder, &remote, .25, &local));
    remote.tv_sec++; remote.tv_nsec = 100000; local.tv_sec++;
    assert(ptp_peer_rate_update(&rate, 1, responder, &remote, .25, &local));
    assert(fabs(rate.ratio-1.0001)<1e-12);
    double delay;
    /* 30 ms turnaround, 500 ns one-way local delay, remote clock +100 ppm. */
    assert(ptp_peer_delay_local(rate.ratio, 30001000, 30003000, 0, 0, &delay));
    assert(fabs(delay-500)<1e-6);
    assert(!ptp_peer_delay_local(1, 30001000, 30003000, 0, 0, &delay));
    assert(ptp_peer_delay_local(.9999, 30001000, 29997000, 0, 0, &delay));
    assert(fabs(delay-500)<1e-6);
    assert(ptp_peer_delay_local(1, 4000, 2000, .25, .75, &delay));
    assert(fabs(delay-999.75)<1e-9);
    assert(!ptp_peer_delay_local(NAN, 4000, 2000, 0, 0, &delay));
    assert(!ptp_peer_delay_local(1, -1, 2000, 0, 0, &delay));
    assert(!ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    remote.tv_sec++; local.tv_sec++;
    assert(ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    responder[9]++;
    assert(!ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    remote.tv_sec += 20; local.tv_sec += 20;
    assert(!ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    remote.tv_sec++; local.tv_sec++;
    assert(ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    assert(!ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    remote.tv_sec++; local.tv_sec += 2;
    assert(!ptp_peer_rate_update(&rate, 2, responder, &remote, 0, &local));
    /* A step of the local clock: the sample after it re-anchors and keeps
     * the ratio measured before; the next one measures on. */
    ptp_peer_rate_t stepped = {0};
    struct timespec stepped_remote = {500, 0}, stepped_local = {20, 0};
    assert(!ptp_peer_rate_update(&stepped, 3, responder, &stepped_remote, 0, &stepped_local));
    stepped_remote.tv_sec++; stepped_remote.tv_nsec = 100000; stepped_local.tv_sec++;
    assert(ptp_peer_rate_update(&stepped, 3, responder, &stepped_remote, 0, &stepped_local));
    stepped.reanchor = true;
    stepped_remote.tv_sec++; stepped_local.tv_sec += 1000;
    assert(ptp_peer_rate_update(&stepped, 3, responder, &stepped_remote, 0, &stepped_local));
    assert(fabs(stepped.ratio-1.0001)<1e-12 && !stepped.reanchor && stepped.updates == 1);
    stepped_remote.tv_sec++; stepped_local.tv_sec++;
    assert(ptp_peer_rate_update(&stepped, 3, responder, &stepped_remote, 0, &stepped_local));
    assert(fabs(stepped.ratio-1.0)<1e-12);
    /* Before any ratio, a re-anchor still has none. */
    ptp_peer_rate_t fresh = {0};
    assert(!ptp_peer_rate_update(&fresh, 4, responder, &stepped_remote, 0, &stepped_local));
    fresh.reanchor = true;
    stepped_remote.tv_sec++; stepped_local.tv_sec++;
    assert(!ptp_peer_rate_update(&fresh, 4, responder, &stepped_remote, 0, &stepped_local));
    /* A neighbor's step spoils the interval across it: the ratio measured
     * before stands, and the next sound interval measures on. */
    ptp_peer_rate_t neighbor = {0};
    struct timespec neighbor_remote = {700, 0}, neighbor_local = {30, 0};
    assert(!ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    neighbor_remote.tv_sec++; neighbor_remote.tv_nsec = 100000; neighbor_local.tv_sec++;
    assert(ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    neighbor_remote.tv_sec += 2; neighbor_local.tv_sec++;
    assert(ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    assert(fabs(neighbor.ratio-1.0001)<1e-12 && neighbor.outliers == 1 && neighbor.updates == 1);
    neighbor_remote.tv_sec++; neighbor_local.tv_sec++;
    assert(ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    assert(fabs(neighbor.ratio-1.0)<1e-12 && neighbor.outliers == 0 && neighbor.updates == 2);
    /* Backward steps too, up to three in a row; a fourth fails, and a
     * sound interval after it measures again. */
    for (unsigned step = 1; step <= PTP_PEER_RATE_OUTLIERS_MAX; ++step) {
        neighbor_remote.tv_sec -= 5; neighbor_local.tv_sec++;
        assert(ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
        assert(fabs(neighbor.ratio-1.0)<1e-12 && neighbor.outliers == step);
    }
    neighbor_remote.tv_sec -= 5; neighbor_local.tv_sec++;
    assert(!ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    assert(!neighbor.valid && neighbor.outliers == 0);
    neighbor_remote.tv_sec -= 5; neighbor_local.tv_sec++;
    assert(!ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    neighbor_remote.tv_sec++; neighbor_local.tv_sec++;
    assert(ptp_peer_rate_update(&neighbor, 5, responder, &neighbor_remote, 0, &neighbor_local));
    /* A neighbor off rate from the start never gets a ratio to keep. */
    ptp_peer_rate_t slewed = {0};
    struct timespec slewed_remote = {900, 0}, slewed_local = {40, 0};
    assert(!ptp_peer_rate_update(&slewed, 6, responder, &slewed_remote, 0, &slewed_local));
    slewed_remote.tv_nsec = 937500000; slewed_local.tv_sec++;
    assert(!ptp_peer_rate_update(&slewed, 6, responder, &slewed_remote, 0, &slewed_local));
    assert(!slewed.valid && slewed.outliers == 0);
    uint8_t correction[8];
    encode(correction, 16384); assert(ptp_peer_correction(correction)==.25);
    encode(correction, -49152); assert(ptp_peer_correction(correction)==-.75);
    puts("Peer rate handles unrelated epochs, +/-100ppm, fractional corrections, identity/lifecycle changes, gaps, invalid rates, local clock steps and neighbor clock steps");
}
