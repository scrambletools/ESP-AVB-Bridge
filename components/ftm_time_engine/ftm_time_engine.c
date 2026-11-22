/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * FTM time engine for a Wi-Fi coprocessor station (CONFIG_FTM_TIME_ENGINE).
 * While the station is associated it asks the access point for an
 * 802.1AS-style FTM burst (PTP compatible, three frames) every 125 ms. The
 * responder puts a Follow_Up in each FTM frame: gPTP time at that
 * measurement's t1. With the round trip that gives gPTP time at t2, our
 * receive time, which is on the Wi-Fi MAC clock. The clock link's map of
 * the MAC clock onto its timer turns t2 into that timer's time, and each
 * session's result goes to the host, which has the same timer paired with
 * its own PTP clock (hosted_avb_wifi_time.c).
 *
 * Ported from the station's 11-11 time engine (ftm_endpoint with pair
 * selection and the signed round trip, ftm_local_clock), without esp_ptp:
 * the host keeps source selection and the servo. esp-hosted owns the
 * station itself; this only reads its association and runs FTM.
 */
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ftm_follow_up.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"
#include "sdkconfig.h"

#if !CONFIG_ESP_WIFI_ENABLE_FTM_PTP
#error "The FTM time engine needs CONFIG_ESP_WIFI_ENABLE_FTM_PTP"
#endif
#if !CONFIG_IDF_TARGET_ESP32C6
#error "The FTM time engine reads the ESP32-C6 Wi-Fi MAC clock"
#endif

#define CADENCE_MS 125
#define LOG_SYNC_INTERVAL -3 /* 125 ms, 802.1AS 12.8.2 */
#define REPORT_TIMEOUT_MS 2000
#define START_RETRY_MS 1000
#define MAX_ENTRIES 16
#define REMOTE_MASK ((UINT64_C(1) << 48) - 1)
#define RTT_MIN_PS -25000 /* the bench's round trip reads slightly negative */
#define RTT_MAX_PS 2000000
#define MAP_MAX_SHIFT_NS 500
#define ANCHOR_MAX_AGE_US 2000000 /* a previous session's anchor for the rate */
#define RATE_MAX_AGE_US 5000000   /* the last rate, when there is no anchor */

static const char *TAG = "ftm_time";

static TaskHandle_t s_task;
static portMUX_TYPE s_report_lock = portMUX_INITIALIZER_UNLOCKED;
static wifi_event_ftm_report_t s_report;
static uint32_t s_boot;

static wifi_ftm_report_entry_t s_entries[MAX_ENTRIES];
static wifi_ftm_vendor_data_t s_vendors[MAX_ENTRIES];
static ftm_follow_up_fields_t s_fields[MAX_ENTRIES];
static bool s_paired[MAX_ENTRIES];

/* The responder's clock against ours needs two measurements. A burst
 * that brought only one (its last frame lost) takes it from the previous
 * session's first measurement instead, about 150 ms back, or failing
 * that the last rate seen. */
static struct {
  bool valid;
  uint64_t t1, t2;
  int64_t at_us;
} s_anchor;
static struct {
  bool valid;
  double rate;
  int64_t at_us;
} s_rate;

static bool plausible_rate(double rate) {
  return isfinite(rate) && fabs(rate - 1) <= .001;
}

typedef hosted_avb_mac_map_t mac_map_t;

static bool map_mac_clock(mac_map_t *map) {
  return hosted_avb_clock_link_map_mac(map);
}

static bool map_convert(const mac_map_t *map, uint64_t ftm_ps, int64_t *cp_ns) {
  return hosted_avb_mac_map_convert(map, ftm_ps, cp_ns);
}

static int32_t map_shift(const mac_map_t *before, const mac_map_t *after) {
  int64_t mac_ns = (int64_t)(int32_t)(after->mac_us - before->mac_us) * 1000;
  return (int32_t)(after->cp_ns - before->cp_ns - mac_ns);
}

/* A, B: the four timestamps' independent minima (ftm_pair_selection.h):
 * the shortest apparent forward and reverse trips over the burst. */
typedef struct {
  int forward, reverse;
  double forward_ps, reverse_ps;
} pair_selection_t;

static void pair_add(pair_selection_t *selection, int index,
                     const wifi_ftm_report_entry_t *entry, uint64_t remote_origin,
                     uint64_t local_origin, double remote_per_local) {
  uint64_t remote_tx = (entry->t1 - remote_origin) & REMOTE_MASK;
  uint64_t remote_rx = (entry->t4 - remote_origin) & REMOTE_MASK;
  if (entry->t2 < local_origin || entry->t3 < entry->t2 ||
      remote_rx < remote_tx || remote_rx > UINT64_C(2000000000000) ||
      entry->t3 - local_origin > UINT64_C(2000000000000))
    return;
  double forward =
      (double)(entry->t2 - local_origin) * remote_per_local - (double)remote_tx;
  double reverse =
      (double)remote_rx - (double)(entry->t3 - local_origin) * remote_per_local;
  if (selection->forward < 0 || forward <= selection->forward_ps) {
    selection->forward = index;
    selection->forward_ps = forward;
  }
  if (selection->reverse < 0 || reverse <= selection->reverse_ps) {
    selection->reverse = index;
    selection->reverse_ps = reverse;
  }
}

/* Turns a report into one measurement. Returns the result code. */
static uint8_t consume(const wifi_event_ftm_report_t *report,
                       const mac_map_t *map, hosted_avb_wifi_time_t *out) {
  unsigned count = report->ftm_report_num_entries;
  unsigned vendor_count = report->ftm_vendor_num_entries;
  out->ftm_status = report->status;
  out->entries = count;
  out->granted_frames =
      report->asap && report->burst_exponent == 0 ? report->ftms_per_burst : 0;
  if (report->status != FTM_STATUS_SUCCESS || !count) {
    esp_wifi_ftm_get_report(NULL, 0);
    esp_wifi_ftm_get_vendor_data(NULL, 0);
    return HOSTED_AVB_WIFI_TIME_FAILED;
  }
  if (count > MAX_ENTRIES)
    count = MAX_ENTRIES;
  if (vendor_count > MAX_ENTRIES)
    vendor_count = MAX_ENTRIES;
  memset(s_entries, 0, sizeof(s_entries));
  memset(s_vendors, 0, sizeof(s_vendors));
  memset(s_paired, 0, sizeof(s_paired));
  if (esp_wifi_ftm_get_report(s_entries, count) != ESP_OK ||
      esp_wifi_ftm_get_vendor_data(s_vendors, vendor_count) != ESP_OK)
    return HOSTED_AVB_WIFI_TIME_FAILED;
  /* The responder's TOD and TOA are 48-bit picosecond fields on air.
   * Earlier Wi-Fi libraries report them as such; later ones extend them
   * to 64 bits, past 2^48 after 281 s. Reduce to the 48-bit field the
   * arithmetic below works in. */
  for (unsigned i = 0; i < count; i++) {
    s_entries[i].t1 &= REMOTE_MASK;
    s_entries[i].t4 &= REMOTE_MASK;
  }

  /* Each measurement's Follow_Up arrives in a later frame, matched by the
   * Follow Up Dialog Token. */
  unsigned matches = 0, refused = 0;
  for (unsigned i = 0; i < count; i++) {
    const wifi_ftm_report_entry_t *entry = &s_entries[i];
    if (!entry->dlog_token || !entry->t1 || !entry->t2 ||
        entry->t3 <= entry->t2 || entry->t1 > REMOTE_MASK ||
        entry->t4 > REMOTE_MASK)
      continue;
    unsigned found = 0;
    for (unsigned v = 0; v < vendor_count; v++) {
      ftm_follow_up_fields_t decoded;
      if (s_vendors[v].dialog_token != entry->dlog_token)
        continue;
      /* A responder that declines still sends the element, empty. */
      if (!ftm_follow_up_parse(s_vendors[v].vendor_ie,
                               s_vendors[v].vendor_ie_len, &decoded)) {
        refused++;
        continue;
      }
      int64_t correction_ns = decoded.correction_scaled / 65536;
      if (correction_ns < -2000000000 || correction_ns > 2000000000 ||
          decoded.origin_ns > INT64_MAX - INT64_C(3000000000) ||
          decoded.origin_ns + correction_ns < 0)
        continue;
      s_fields[i] = decoded;
      found++;
    }
    s_paired[i] = found == 1;
    matches += s_paired[i];
  }
  out->follow_ups = matches;
  out->refused = refused > UINT8_MAX ? UINT8_MAX : refused;
  if (matches < 1)
    return HOSTED_AVB_WIFI_TIME_NO_FOLLOW_UP;
  unsigned first = 0, last = count - 1;
  while (!s_paired[first])
    first++;
  while (!s_paired[last])
    last--;
  for (unsigned i = first + 1; i <= last; i++)
    if (s_paired[i] &&
        (memcmp(s_fields[first].source_port, s_fields[i].source_port, 10) ||
         s_fields[first].time_base != s_fields[i].time_base ||
         s_fields[first].domain != s_fields[i].domain))
      return HOSTED_AVB_WIFI_TIME_REJECTED;

  /* The responder's clock against ours: over the burst, or with one
   * measurement, from the previous session's first one. */
  int64_t now_us = esp_timer_get_time();
  double neighbor_rate = 0;
  uint8_t rate_source;
  if (last > first) {
    uint64_t remote_span =
        (s_entries[last].t1 - s_entries[first].t1) & REMOTE_MASK;
    if (s_entries[last].t2 <= s_entries[first].t2 ||
        remote_span < UINT64_C(500000000) ||
        remote_span > UINT64_C(1500000000000))
      return HOSTED_AVB_WIFI_TIME_REJECTED;
    neighbor_rate = (double)remote_span /
                    (double)(s_entries[last].t2 - s_entries[first].t2);
    rate_source = HOSTED_AVB_WIFI_TIME_RATE_BURST;
  } else if (s_anchor.valid && now_us - s_anchor.at_us <= ANCHOR_MAX_AGE_US &&
             s_entries[first].t2 > s_anchor.t2 &&
             s_entries[first].t2 - s_anchor.t2 <= UINT64_C(2000000000000)) {
    uint64_t remote_span = (s_entries[first].t1 - s_anchor.t1) & REMOTE_MASK;
    neighbor_rate =
        (double)remote_span / (double)(s_entries[first].t2 - s_anchor.t2);
    rate_source = HOSTED_AVB_WIFI_TIME_RATE_SESSIONS;
    /* A responder clock step between the sessions shows as a wild rate:
     * fall back to the last rate. */
    if (!plausible_rate(neighbor_rate) && s_rate.valid &&
        now_us - s_rate.at_us <= RATE_MAX_AGE_US) {
      neighbor_rate = s_rate.rate;
      rate_source = HOSTED_AVB_WIFI_TIME_RATE_HELD;
    }
  } else if (s_rate.valid && now_us - s_rate.at_us <= RATE_MAX_AGE_US) {
    neighbor_rate = s_rate.rate;
    rate_source = HOSTED_AVB_WIFI_TIME_RATE_HELD;
  } else {
    rate_source = HOSTED_AVB_WIFI_TIME_RATE_HELD;
  }
  /* The next session's anchor, whatever becomes of this one. */
  s_anchor.valid = true;
  s_anchor.t1 = s_entries[first].t1;
  s_anchor.t2 = s_entries[first].t2;
  s_anchor.at_us = now_us;
  out->rate_source = rate_source;
  if (!plausible_rate(neighbor_rate))
    return HOSTED_AVB_WIFI_TIME_REJECTED;
  if (rate_source != HOSTED_AVB_WIFI_TIME_RATE_HELD) {
    s_rate.valid = true;
    s_rate.rate = neighbor_rate;
    s_rate.at_us = now_us;
  }

  pair_selection_t selection = {.forward = -1, .reverse = -1};
  for (unsigned i = 0; i < count; i++) {
    if (!s_paired[i])
      continue;
    const wifi_ftm_report_entry_t *entry = &s_entries[i];
    uint64_t response_ps = (entry->t4 - entry->t1) & REMOTE_MASK;
    uint64_t turnaround_ps = entry->t3 - entry->t2;
    if (response_ps > UINT64_C(1000000000) ||
        turnaround_ps > UINT64_C(1000000000))
      continue;
    double rtt = (double)response_ps - (double)turnaround_ps * neighbor_rate;
    if (rtt < RTT_MIN_PS || rtt > RTT_MAX_PS)
      continue;
    pair_add(&selection, i, entry, s_entries[first].t1, s_entries[first].t2,
             neighbor_rate);
  }
  if (selection.forward < 0 || selection.reverse < 0)
    return HOSTED_AVB_WIFI_TIME_REJECTED;
  double rtt_ps = selection.forward_ps + selection.reverse_ps;
  if (rtt_ps < RTT_MIN_PS || rtt_ps > RTT_MAX_PS)
    return HOSTED_AVB_WIFI_TIME_REJECTED;

  /* gPTP time at the forward measurement's arrival: its Follow_Up time
   * at t1, plus half the round trip in gPTP time. */
  unsigned best = selection.forward;
  const ftm_follow_up_fields_t *fields = &s_fields[best];
  double reference_rate = 1 + fields->cumulative_rate_offset / 2199023255552.0;
  int64_t reference_ns = fields->origin_ns + fields->correction_scaled / 65536 +
                         (int64_t)llround(rtt_ps * reference_rate / 2000);
  int64_t cp_ns;
  if (!map_convert(map, s_entries[best].t2, &cp_ns))
    return HOSTED_AVB_WIFI_TIME_NO_MAP;
  out->cp_ns = cp_ns;
  out->reference_ns = reference_ns;
  out->rtt_ps = (int32_t)llround(rtt_ps);
  out->rate_ppb = (int32_t)llround((neighbor_rate * reference_rate - 1) * 1e9);
  out->domain = fields->domain;
  out->log_interval = fields->log_interval;
  out->time_base = fields->time_base;
  memcpy(out->source_port, fields->source_port, 10);
  out->rssi = s_entries[best].rssi;
  return HOSTED_AVB_WIFI_TIME_OBSERVED;
}

static void on_ftm_report(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  portENTER_CRITICAL(&s_report_lock);
  s_report = *(const wifi_event_ftm_report_t *)data;
  portEXIT_CRITICAL(&s_report_lock);
  xTaskNotifyGive(s_task);
}

static void send(hosted_avb_wifi_time_t *message, uint32_t session,
                 uint8_t result) {
  message->version = HOSTED_AVB_WIFI_TIME_VERSION;
  message->result = result;
  message->coprocessor_boot = s_boot;
  message->session = session;
  hosted_avb_peer_send(HOSTED_AVB_MSG_WIFI_TIME, (const uint8_t *)message,
                       sizeof(*message));
}

static void engine_task(void *arg) {
  /* The clock link starts with esp_hosted_avb, the event loop with
   * esp-hosted; wait for both. */
  mac_map_t first_map;
  while (!map_mac_clock(&first_map))
    vTaskDelay(pdMS_TO_TICKS(500));
  while (esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT,
                                    on_ftm_report, NULL) != ESP_OK)
    vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, "FTM time engine ready");

  uint32_t session = 0;
  bool was_associated = false;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CADENCE_MS));
    wifi_ap_record_t ap;
    bool associated = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    if (associated && !was_associated)
      ESP_LOGI(TAG, "Measuring to %02x:%02x:%02x:%02x:%02x:%02x", ap.bssid[0],
               ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4],
               ap.bssid[5]);
    else if (!associated && was_associated)
      ESP_LOGI(TAG, "Not associated");
    was_associated = associated;
    if (!associated)
      continue;

    hosted_avb_wifi_time_t message = {0};
    mac_map_t map_before, map_after;
    if (!map_mac_clock(&map_before)) {
      send(&message, ++session, HOSTED_AVB_WIFI_TIME_NO_MAP);
      continue;
    }
    wifi_ftm_initiator_cfg_t config = {
        .channel = ap.primary,
        .frm_count = 3,
        .ptp_compatible = true,
        .log_sync_intvl = LOG_SYNC_INTERVAL,
        .report_vendor_ie = true,
    };
    memcpy(config.resp_mac, ap.bssid, 6);
    ulTaskNotifyTake(pdTRUE, 0);
    session++;
    esp_err_t err = esp_wifi_ftm_initiate_session(&config);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "FTM session: %s", esp_err_to_name(err));
      send(&message, session, HOSTED_AVB_WIFI_TIME_NOT_STARTED);
      vTaskDelay(pdMS_TO_TICKS(START_RETRY_MS));
      wake = xTaskGetTickCount();
      continue;
    }
    if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(REPORT_TIMEOUT_MS))) {
      esp_wifi_ftm_end_session();
      send(&message, session, HOSTED_AVB_WIFI_TIME_NO_REPORT);
      wake = xTaskGetTickCount();
      continue;
    }
    wifi_event_ftm_report_t report;
    portENTER_CRITICAL(&s_report_lock);
    report = s_report;
    portEXIT_CRITICAL(&s_report_lock);
    uint8_t result;
    if (!map_mac_clock(&map_after)) {
      esp_wifi_ftm_get_report(NULL, 0);
      esp_wifi_ftm_get_vendor_data(NULL, 0);
      result = HOSTED_AVB_WIFI_TIME_NO_MAP;
    } else {
      message.map_uncertainty_ns = map_after.uncertainty_ns;
      message.map_shift_ns = map_shift(&map_before, &map_after);
      result = consume(&report, &map_after, &message);
      if (result == HOSTED_AVB_WIFI_TIME_OBSERVED &&
          (message.map_shift_ns > MAP_MAX_SHIFT_NS ||
           message.map_shift_ns < -MAP_MAX_SHIFT_NS))
        result = HOSTED_AVB_WIFI_TIME_NO_MAP;
    }
    send(&message, session, result);
  }
}

__attribute__((constructor)) static void start_engine(void) {
  s_boot = esp_random() | 1;
  if (xTaskCreate(engine_task, "ftm_time", 4096, NULL, 4, &s_task) != pdPASS)
    ESP_LOGE(TAG, "No task; no FTM time engine");
}
