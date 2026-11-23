/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Beacon time element from the clock link (CONFIG_BEACON_TIME_FROM_LINK),
 * coprocessor side. The host stamps the element's Follow_Up when it
 * builds it, and the AP TSF was added when the element got here, a host
 * message later. Stations had to recover from that gap, and from a
 * rate of 0, with a slow envelope that let their clocks wander by
 * hundreds of microseconds. Here, from the host's gPTP time on this
 * chip's clock-link timer, the element carries gPTP time at the very
 * TSF it carries, and the rate of gPTP time against the TSF in its
 * cumulativeScaledRateOffset, as AVB Wireless §2.4.1 lets an access
 * point that measures it. The host logs the counts (BEACONTIME lines).
 *
 * The TSF read takes tens of microseconds through the Wi-Fi library, so
 * gPTP time is stamped at its earliest point: the element's gPTP time
 * is then never later than its TSF, as stations expect of a gap (their
 * envelope follows the smallest one). A stamp at the middle erred both
 * ways and their slope learner, biased by it, swung ±1.5 ms.
 */
#include "sdkconfig.h"

#if CONFIG_BEACON_TIME_FROM_LINK && CONFIG_IDF_TARGET_ESP32C6

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ftm_time_responder_wire.h"
#include "ftm_time_transfer_cp.h"
#include "hosted_avb.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

#define TRANSFER_MAX_AGE_US 1000000    /* five of the host's publications */
#define ANCHOR_MAX_AGE_NS 1500000000LL /* since the clock link's edge */
#define MAX_UNCERTAINTY_NS 25000
#define STATUS_PERIOD_MS 10000
#define MAC_CHECK_READS 16
/* The Wi-Fi MAC's microsecond clock (as the clock link reads it). */
#define MAC_CLOCK_REG ((volatile const uint32_t *)0x600ad000U)

/* Offsets in the element's Follow_Up (header, origin, FollowUp TLV). */
#define FU_CORRECTION 8
#define FU_ORIGIN 34
#define FU_RATE 54
#define FU_MIN_LENGTH (FU_RATE + 4)

static const char *TAG = "beacon_time";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static beacon_time_status_t s_counts;

static void put_be(uint8_t *at, uint64_t value, unsigned octets) {
  for (unsigned i = 0; i < octets; i++)
    at[i] = (uint8_t)(value >> (8 * (octets - 1 - i)));
}

/* gPTP time at timer time cp_ns, from a fresh transfer. */
static bool gptp_at(const ftm_time_transfer_t *transfer, int64_t cp_ns,
                    int64_t *gptp_ns) {
  int64_t anchor_age_ns = cp_ns - transfer->cp_anchor_ns;
  if (anchor_age_ns < -ANCHOR_MAX_AGE_NS || anchor_age_ns > ANCHOR_MAX_AGE_NS)
    return false;
  *gptp_ns = transfer->reference_ns +
             (int64_t)((double)anchor_age_ns * (double)transfer->rate_q32 /
                       4294967296.0);
  return *gptp_ns >= 0;
}

bool hosted_avb_beacon_time_fill(uint8_t *follow_up, size_t follow_up_len,
                                 uint8_t tsf_le[8]) {
  ftm_time_transfer_t transfer;
  int64_t received_us;
  if (follow_up_len < FU_MIN_LENGTH ||
      !ftm_time_transfer_latest(&transfer, &received_us) ||
      esp_timer_get_time() - received_us > TRANSFER_MAX_AGE_US) {
    portENTER_CRITICAL(&s_lock);
    s_counts.no_time++;
    portEXIT_CRITICAL(&s_lock);
    return false;
  }
  /* (rate - 1) x 2^41, gPTP time against the timer, which the TSF shares
   * its crystal with. Set whenever the rate is known, so it doesn't
   * flip to the host's 0 on an element left to the host. */
  int64_t scaled = (transfer.rate_q32 - INT64_C(4294967296)) * 512;
  if (scaled > INT32_MAX)
    scaled = INT32_MAX;
  if (scaled < INT32_MIN)
    scaled = INT32_MIN;
  put_be(follow_up + FU_RATE, (uint32_t)(int32_t)scaled, 4);
  int32_t rate_ppb =
      (int32_t)((transfer.rate_q32 - INT64_C(4294967296)) * 1000000000LL /
                INT64_C(4294967296));

  int64_t tsf_us, cp_ns, gptp_ns;
  uint32_t uncertainty_ns;
  if (!hosted_avb_clock_link_ap_tsf(&tsf_us, &cp_ns, &uncertainty_ns) ||
      uncertainty_ns > MAX_UNCERTAINTY_NS) {
    portENTER_CRITICAL(&s_lock);
    s_counts.no_tsf++;
    portEXIT_CRITICAL(&s_lock);
    return false;
  }
  /* The earliest the TSF can have reached tsf_us. */
  if (!gptp_at(&transfer, cp_ns - (int64_t)uncertainty_ns, &gptp_ns)) {
    portENTER_CRITICAL(&s_lock);
    s_counts.no_time++;
    portEXIT_CRITICAL(&s_lock);
    return false;
  }
  /* Measured: stations otherwise run this far behind wired gPTP. */
  gptp_ns += CONFIG_BEACON_TIME_OFFSET_NS;

  /* gPTP time at the TSF below, all in the preciseOriginTimestamp. */
  memset(follow_up + FU_CORRECTION, 0, 8);
  put_be(follow_up + FU_ORIGIN, (uint64_t)gptp_ns / 1000000000U, 6);
  put_be(follow_up + FU_ORIGIN + 6, (uint64_t)gptp_ns % 1000000000U, 4);
  for (unsigned i = 0; i < 8; i++)
    tsf_le[i] = (uint8_t)((uint64_t)tsf_us >> (8 * i));

  portENTER_CRITICAL(&s_lock);
  if (!s_counts.filled++ || uncertainty_ns < s_counts.read_min_ns)
    s_counts.read_min_ns = uncertainty_ns;
  if (uncertainty_ns > s_counts.read_max_ns)
    s_counts.read_max_ns = uncertainty_ns;
  s_counts.rate_ppb = rate_ppb;
  portEXIT_CRITICAL(&s_lock);
  return true;
}

/* Is the MAC clock register the AP TSF? Read it on both sides of a TSF
 * read; if it is, TSF less the register lies between 0 and the read's
 * length before, and between minus that and 0 after. */
static void check_mac_clock(beacon_time_status_t *status) {
  bool any = false;
  for (unsigned i = 0; i < MAC_CHECK_READS; i++) {
    int64_t t0, t1, t2, t3;
    uint32_t mac_before, mac_after;
    if (!hosted_avb_clock_link_coprocessor_read(MAC_CLOCK_REG, &t0,
                                                &mac_before, &t1))
      return;
    int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_AP);
    if (!hosted_avb_clock_link_coprocessor_read(MAC_CLOCK_REG, &t2,
                                                &mac_after, &t3) ||
        tsf <= 0)
      return;
    int32_t before = (int32_t)((uint32_t)tsf - mac_before);
    int32_t after = (int32_t)((uint32_t)tsf - mac_after);
    if (!any || before < status->tsf_less_mac_before_min)
      status->tsf_less_mac_before_min = before;
    if (!any || before > status->tsf_less_mac_before_max)
      status->tsf_less_mac_before_max = before;
    if (!any || after < status->tsf_less_mac_after_min)
      status->tsf_less_mac_after_min = after;
    if (!any || after > status->tsf_less_mac_after_max)
      status->tsf_less_mac_after_max = after;
    any = true;
  }
}

/* The fill runs on esp_hosted_avb's receive callback, which mustn't send;
 * this task reports. */
static void status_task(void *arg) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    beacon_time_status_t status;
    portENTER_CRITICAL(&s_lock);
    status = s_counts;
    s_counts = (beacon_time_status_t){.rate_ppb = s_counts.rate_ppb};
    portEXIT_CRITICAL(&s_lock);
    check_mac_clock(&status);
    status.version = BEACON_TIME_STATUS_VERSION;
    hosted_avb_peer_send(HOSTED_AVB_MSG_BEACON_TIME_STATUS,
                         (const uint8_t *)&status, sizeof(status));
  }
}

__attribute__((constructor)) static void start_status(void) {
  if (xTaskCreate(status_task, "beacon_time", 2560, NULL, 3, NULL) != pdPASS)
    ESP_LOGE(TAG, "No status task; the host sees no BEACONTIME counts");
}

#endif
