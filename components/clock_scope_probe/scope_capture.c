#include "driver/gptimer.h"
#include "gptimer_priv.h"
#include "../ftm_clock_probe/timer_capture.h"
#if CONFIG_IDF_TARGET_ESP32C6
/* esp_ptp's local clock (ptp.h), declared here: this file builds inside
 * the GPTimer driver, which has no esp_ptp include path. */
extern void ptp_clock_local_latch(void);
extern int64_t ptp_clock_local_latched_ns(void);

bool scope_capture_local(gptimer_handle_t timer, uint64_t *before,
                         int64_t *local_ns, uint64_t *after)
{
    /* C6 is single core. Preserve the shared local clock latch against ISR reads. */
    portENTER_CRITICAL(&timer->spinlock);
    uint64_t first = clock_capture_timer(&timer->hal);
    ptp_clock_local_latch();
    uint64_t last = clock_capture_timer(&timer->hal);
    int64_t latched = ptp_clock_local_latched_ns();
    portEXIT_CRITICAL(&timer->spinlock);
    *before = first;
    *after = last;
    *local_ns = latched;
    return latched > 0;
}
#else
void scope_capture_register(gptimer_handle_t timer, volatile const uint32_t *counter,
                            uint64_t *before, uint32_t *value, uint64_t *after)
{
    portENTER_CRITICAL(&timer->spinlock);
    uint64_t first = clock_capture_timer(&timer->hal);
    uint32_t sample = *counter;
    uint64_t last = clock_capture_timer(&timer->hal);
    portEXIT_CRITICAL(&timer->spinlock);
    *before = first;
    *value = sample;
    *after = last;
}
#endif
