#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "ptp_sw_pi.h"

/* The software clock's servo as it was: per observation, offset / 2000
 * proportional and offset / 8000 into the integral (esp_ptp ptp.c). */
static int32_t old_step(int32_t *drift, int64_t offset_ns)
{
    *drift += (int32_t)(offset_ns / 8000);
    if (*drift > 512000) *drift = 512000;
    if (*drift < -512000) *drift = -512000;
    int64_t target = offset_ns / 2000 + *drift;
    if (target > 512000) target = 512000;
    if (target < -512000) target = -512000;
    return (int32_t)target;
}

static double gaussian(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

typedef struct {
    double peak_ns;         /* largest offset after the start */
    double settle_s;        /* last time |offset| was above 1 us */
    double late_peak_ns;    /* largest offset in the last 10 min */
    double late_rms_ns;     /* rms offset in the last 10 min */
    double trim_ppb;        /* final trim */
} result_t;

/* A clock oscillator_ppb slow against its reference, starting in phase
 * as after a jump, observed every interval_ns with gaussian noise of
 * noise_ns on each observation; 40 minutes. */
static result_t simulate(bool damped, int64_t interval_ns, int32_t oscillator_ppb,
                         double noise_ns, double start_ns)
{
    int64_t integral = 0;
    int32_t drift = 0;
    double offset = start_ns, trim = 0;
    result_t result = {0};
    double late_sum = 0;
    unsigned late_count = 0;
    double total_s = 2400, dt = interval_ns / 1e9;
    unsigned steps = (unsigned)(total_s / dt);
    for (unsigned step = 0; step < steps; ++step) {
        double now = step * dt;
        int64_t seen = (int64_t)llround(offset + noise_ns * gaussian());
        trim = damped ? ptp_sw_pi_step(&integral, seen, interval_ns) : old_step(&drift, seen);
        offset += (oscillator_ppb - trim) * dt;
        if (fabs(offset) > result.peak_ns) result.peak_ns = fabs(offset);
        if (fabs(offset) > 1000) result.settle_s = now;
        if (now >= total_s - 600) {
            if (fabs(offset) > result.late_peak_ns) result.late_peak_ns = fabs(offset);
            late_sum += offset * offset;
            ++late_count;
        }
        assert(isfinite(offset));
    }
    result.late_rms_ns = sqrt(late_sum / late_count);
    result.trim_ppb = trim;
    printf("%s interval=%4.0f ms drift=%6d ppb noise=%5.0f ns start=%7.0f ns: peak %8.1f us, "
           "settled %6.0f s, last 10 min peak %8.3f us rms %7.3f us, trim %7.0f ppb\n",
           damped ? "damped" : "old   ", interval_ns / 1e6, oscillator_ppb, noise_ns, start_ns,
           result.peak_ns / 1000, result.settle_s, result.late_peak_ns / 1000,
           result.late_rms_ns / 1000, result.trim_ppb);
    return result;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    srand(1);
    /* The bench: about one beacon element a second (1.024 s), a C6 some
     * tens of ppm off. The old loop rings: +-2.5 ms, 9.5 min period, still
     * over a millisecond 40 minutes later. */
    result_t old = simulate(false, 1024000000, 27000, 0, 0);
    assert(old.peak_ns > 2000000 && old.late_peak_ns > 1000000);

    const int64_t intervals[] = {102400000, 1024000000, 4000000000};
    const int32_t drifts[] = {27000, -100000, 300000};
    for (unsigned i = 0; i < sizeof(intervals) / sizeof(intervals[0]); ++i) {
        for (unsigned j = 0; j < sizeof(drifts) / sizeof(drifts[0]); ++j) {
            result_t r = simulate(true, intervals[i], drifts[j], 0, 0);
            /* Peak about 0.45 x drift / wn, settled within a few minutes,
             * and the trim holds the oscillator's rate. */
            assert(r.peak_ns < 0.6 * fabs((double)drifts[j]) / 0.05);
            assert(r.settle_s < 300);
            assert(r.late_peak_ns < 100);
            assert(fabs(r.trim_ppb - drifts[j]) < 10);
        }
        /* A phase step of the reference, as when the envelope moves. */
        result_t phase = simulate(true, intervals[i], 27000, 0, 5000000);
        assert(phase.settle_s < 300 && phase.late_peak_ns < 100);
        /* Observation noise: the loop averages it down and stays stable,
         * less so with fewer observations (11.5 us rms at 4 s). */
        result_t noisy = simulate(true, intervals[i], 27000, 20000, 0);
        assert(noisy.late_rms_ns < 0.75 * 20000);
    }

    /* Saturation: the output pins at the limit and the integral holds. */
    int64_t integral = 0;
    assert(ptp_sw_pi_step(&integral, INT64_MAX, 1000000000) == PTP_SW_PI_LIMIT);
    assert(integral == 0);
    assert(ptp_sw_pi_step(&integral, INT64_MIN, 1000000000) == -PTP_SW_PI_LIMIT);
    assert(integral == 0);
    /* Out-of-range intervals are clamped, not trusted: 1 us for 10 ms
     * adds 0.025 ppb to the integral, for 4 s 10 ppb. */
    assert(ptp_sw_pi_step(&integral, 1000, 0) == 80);
    assert(integral == 1638);
    integral = 0;
    assert(ptp_sw_pi_step(&integral, 1000, INT64_MAX) == 90);
    assert(integral == 655360);
    printf("ok\n");
    return 0;
}
