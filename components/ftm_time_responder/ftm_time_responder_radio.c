/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * FTM time responder, coprocessor side (see ftm_time_responder.h). For
 * each transfer from the host it maps the Wi-Fi MAC clock onto the clock
 * link's timer, puts gPTP time on the MAC clock, and prepares a Follow_Up
 * relaying the upstream one. The FTM responder's callback, on the Wi-Fi
 * task, then completes that Follow_Up for each frame's measurement and
 * puts it in the frame, for stations the host has admitted.
 *
 * Carried over from the 11-11 bench probe (ftm_clock_probe FOLLOW_UP),
 * with the clock link in place of its own edge capture.
 */
#include "sdkconfig.h"

#if CONFIG_FTM_TIME_RESPONDER && CONFIG_IDF_TARGET_ESP32C6

#if !CONFIG_ESP_WIFI_ENABLE_FTM_PTP
#error "The FTM time responder needs CONFIG_ESP_WIFI_ENABLE_FTM_PTP"
#endif

#include <inttypes.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ftm_follow_up.h"
#include "ftm_follow_up_tx.h"
#include "ftm_radio_association.h"
#include "ftm_time_responder_wire.h"
#include "ftm_time_transfer_cp.h"
#include "hosted_avb.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

_Static_assert(HOSTED_AVB_ASSOCIATIONS_MAX == FTM_ASSOC_MAX,
               "esp_hosted_avb and the FTM map list the same stations");

#define SNAPSHOT_COUNT 4
#define SOURCE_MAX_AGE_NS 500000000LL /* since the upstream Sync */
#define ANCHOR_MAX_AGE_NS 1500000000LL /* since the clock link's edge */
#define STATUS_PERIOD_MS 10000

static const char *TAG = "ftm_responder";

/* Read on the Wi-Fi task from the frame callback: all in DRAM. */
static DRAM_ATTR portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR ftm_follow_up_tx_t s_snapshots[SNAPSHOT_COUNT];
static DRAM_ATTR int64_t s_prepared_us[SNAPSHOT_COUNT];
static DRAM_ATTR unsigned s_next_snapshot;
static DRAM_ATTR ftm_association_state_t s_admitted;
static DRAM_ATTR ftm_radio_association_t s_radio;
static DRAM_ATTR uint32_t s_admitted_generation;
static DRAM_ATTR uint64_t s_latest_departure_ps;
static DRAM_ATTR struct {
  uint16_t port, sequence;
} s_sequences[FTM_ASSOC_MAX];
static DRAM_ATTR ftm_time_responder_status_t s_status;

static QueueHandle_t s_transfers;
static TaskHandle_t s_task;
static uint32_t s_radio_boot;
static uint32_t s_transfer_host, s_transfer_generation, s_transfer_publication;
static volatile bool s_register_callbacks = true;

static void invalidate_snapshots(void) {
  portENTER_CRITICAL(&s_lock);
  for (unsigned i = 0; i < SNAPSHOT_COUNT; i++)
    s_snapshots[i].valid = false;
  portEXIT_CRITICAL(&s_lock);
}

/* esp_hosted_avb owns the AP's associations and reports them to the
 * host; every change retires the admitted list until the host's next. */
void hosted_avb_associations_changed(
    const hosted_avb_associations_t *associations) {
  portENTER_CRITICAL(&s_lock);
  s_radio.generation = associations->generation;
  s_radio.count = associations->count;
  s_radio.forwarded = false;
  memcpy(s_radio.macs, associations->macs, sizeof(s_radio.macs));
  s_admitted_generation = 0;
  ftm_association_retire_all(&s_admitted);
  for (unsigned i = 0; i < SNAPSHOT_COUNT; i++)
    s_snapshots[i].valid = false;
  portEXIT_CRITICAL(&s_lock);
}

void hosted_avb_associations_forwarded(uint32_t generation, bool queued) {
  portENTER_CRITICAL(&s_lock);
  ftm_radio_association_forwarded(&s_radio, generation, queued);
  portEXIT_CRITICAL(&s_lock);
}

static void receive_associations(const uint8_t *data, size_t length) {
  ftm_association_packet_t packet;
  uint32_t generation;
  if (!ftm_radio_association_decode(data, length, &generation, &packet))
    return;
  int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_lock);
  uint32_t previous = s_admitted_generation;
  bool accepted =
      ftm_radio_association_accepts(&s_radio, generation, &packet) &&
      ftm_association_apply(&s_admitted, &packet, s_transfer_host,
                            s_radio_boot, now_us);
  if (accepted)
    s_admitted_generation = generation;
  portEXIT_CRITICAL(&s_lock);
  if (accepted && previous != generation)
    ESP_LOGI(TAG, "Admitted %" PRIu32 " stations (generation %" PRIu32 ")",
             packet.count, generation);
}

/* Turns a transfer into a Follow_Up snapshot on the MAC clock. */
static void prepare_snapshot(const ftm_time_transfer_t *transfer) {
  int64_t prepared_us = esp_timer_get_time();
  s_status.transfers++;
  if (transfer->version != FTM_TIME_TRANSFER_VERSION)
    return;
  if (s_transfer_host == transfer->host_boot &&
      (int32_t)(transfer->publication - s_transfer_publication) <= 0)
    return;
  if (s_transfer_host != transfer->host_boot ||
      s_transfer_generation != transfer->generation)
    invalidate_snapshots();
  s_transfer_host = transfer->host_boot;
  s_transfer_generation = transfer->generation;
  s_transfer_publication = transfer->publication;
  if (!transfer->valid || !transfer->generation ||
      transfer->coprocessor_boot != hosted_avb_clock_link_coprocessor_boot() ||
      transfer->rate_q32 < INT64_C(4286377361) ||
      transfer->rate_q32 > INT64_C(4303557231) ||
      transfer->upstream_receive_ns < 0 || transfer->reference_ns < 0 ||
      transfer->reference_ns > INT64_MAX - INT64_C(4000000000)) {
    s_status.transfers_rejected++;
    invalidate_snapshots();
    return;
  }
  /* Until a frame has gone out there is nothing to place the anchor
   * against (the MAC clock reads 32 bits, FTM times 64). */
  uint64_t departure_ps = s_latest_departure_ps;
  if (!departure_ps)
    return;
  hosted_avb_mac_map_t map;
  if (!hosted_avb_clock_link_map_mac(&map)) {
    s_status.maps_failed++;
    return;
  }
  /* gPTP time at the map's MAC instant, from the host's anchor. */
  int64_t anchor_age_ns = map.cp_ns - transfer->cp_anchor_ns;
  if (anchor_age_ns < -ANCHOR_MAX_AGE_NS || anchor_age_ns > ANCHOR_MAX_AGE_NS) {
    s_status.transfers_rejected++;
    invalidate_snapshots();
    return;
  }
  int64_t reference_ns =
      transfer->reference_ns +
      (int64_t)((double)anchor_age_ns * (double)transfer->rate_q32 /
                4294967296.0);
  int64_t source_age_ns = reference_ns - transfer->upstream_receive_ns;
  if (source_age_ns < 0 || source_age_ns > SOURCE_MAX_AGE_NS) {
    s_status.transfers_rejected++;
    invalidate_snapshots();
    return;
  }
  /* The map's 32-bit microsecond on the 64-bit departure clock. */
  uint64_t frame_us = departure_ps / 1000000;
  uint32_t delta = map.mac_us - (uint32_t)frame_us;
  int64_t anchor_us = (int64_t)frame_us + (delta <= INT32_MAX
                                               ? (int64_t)delta
                                               : (int64_t)delta - INT64_C(4294967296));
  if (anchor_us < 0 || (uint64_t)anchor_us > UINT64_MAX / 1000000 - 2000000)
    return;
  int64_t residence_ns = reference_ns - transfer->upstream_receive_ns +
                         transfer->peer_delay_ns;
  int64_t remaining_ns = SOURCE_MAX_AGE_NS - source_age_ns;
  if (remaining_ns > ANCHOR_MAX_AGE_NS - anchor_age_ns)
    remaining_ns = ANCHOR_MAX_AGE_NS - anchor_age_ns;
  ftm_follow_up_tx_t next = {
      .anchor_ps = (uint64_t)anchor_us * 1000000,
      /* Only frames sent after this was prepared use it. */
      .not_before_ps = ((uint64_t)anchor_us + 1) * 1000000,
      .expires_us = esp_timer_get_time() + remaining_ns / 1000,
      .rate_q32 = transfer->rate_q32,
      .generation = transfer->generation,
      .valid = true,
  };
  int64_t cumulative_rate = (transfer->rate_q32 - INT64_C(4294967296)) * 512;
  ftm_follow_up_fields_t fields;
  if (!ftm_follow_up_relay(transfer->follow_up, sizeof(transfer->follow_up),
                           transfer->source_port, 0, -3, residence_ns * 65536,
                           cumulative_rate, next.ie) ||
      !ftm_follow_up_parse(next.ie, sizeof(next.ie), &fields)) {
    s_status.transfers_rejected++;
    return;
  }
  next.correction_scaled = fields.correction_scaled;
  portENTER_CRITICAL(&s_lock);
  if (s_admitted.valid && s_admitted.host_boot == transfer->host_boot &&
      !memcmp(s_admitted.clock_identity, transfer->source_port, 8)) {
    unsigned slot = s_next_snapshot++ % SNAPSHOT_COUNT;
    s_snapshots[slot] = next;
    s_prepared_us[slot] = prepared_us;
  }
  portEXIT_CRITICAL(&s_lock);
}

/* Per-port Follow_Up sequenceId (802.1AS 10.5.7). Caller holds s_lock. */
static IRAM_ATTR uint16_t follow_up_sequence(const uint8_t source_port[10]) {
  uint16_t port = ((uint16_t)source_port[8] << 8) | source_port[9];
  unsigned free_slot = FTM_ASSOC_MAX;
  for (unsigned i = 0; i < FTM_ASSOC_MAX; i++) {
    if (s_sequences[i].port == port)
      return ++s_sequences[i].sequence;
    if (!s_sequences[i].port && free_slot == FTM_ASSOC_MAX)
      free_slot = i;
  }
  if (free_slot == FTM_ASSOC_MAX)
    free_slot = port % FTM_ASSOC_MAX;
  s_sequences[free_slot].port = port;
  s_sequences[free_slot].sequence = 1;
  return 1;
}

/* The FTM responder's element callback, on the Wi-Fi task: fills the
 * Follow_Up for the measurement this frame reports (followup token,
 * departure t1). A declined frame carries the element empty. */
static IRAM_ATTR int fill_frame(const uint8_t peer[6], uint8_t token,
                                uint8_t followup, uint64_t departure,
                                uint64_t arrival, uint8_t *buffer,
                                uint16_t *length) {
  (void)token;
  (void)arrival;
  if (*length < FTM_FOLLOW_UP_IE_SIZE)
    return 1;
  int64_t now_us = esp_timer_get_time();
  const ftm_follow_up_tx_t *chosen = NULL;
  uint8_t source_port[10];
  bool list_ready = false, fresh = false;
  portENTER_CRITICAL_ISR(&s_lock);
  if (followup && departure)
    s_latest_departure_ps = departure;
  list_ready = s_radio.forwarded && s_admitted_generation &&
               s_admitted_generation == s_radio.generation;
  if (list_ready) {
    for (unsigned count = 0; count < SNAPSHOT_COUNT; count++) {
      unsigned slot = (s_next_snapshot - 1 - count) % SNAPSHOT_COUNT;
      const ftm_follow_up_tx_t *snapshot = &s_snapshots[slot];
      if (snapshot->valid && departure >= snapshot->not_before_ps &&
          now_us <= snapshot->expires_us) {
        fresh = true;
        if (ftm_association_select(&s_admitted, peer, now_us,
                                   s_prepared_us[slot], source_port))
          chosen = snapshot;
        /* Older publications cannot cross a newer association boundary. */
        break;
      }
    }
  }
  uint16_t sequence = chosen ? follow_up_sequence(source_port) : 0;
  bool emitted = ftm_follow_up_emit(chosen, departure, now_us, followup,
                                    sequence, buffer, *length);
  if (emitted) {
    for (unsigned i = 0; i < 10; i++)
      buffer[26 + i] = source_port[i];
    s_status.filled++;
  } else if (followup) {
    s_status.declined++;
    if (!list_ready || (fresh && !chosen))
      s_status.not_admitted++;
    else
      s_status.no_snapshot++; /* none fresh, or too old for this t1 */
  }
  portEXIT_CRITICAL_ISR(&s_lock);
  *length = FTM_FOLLOW_UP_IE_SIZE;
  return emitted ? 0 : 1;
}

/* Every request is granted as asked; the Wi-Fi task must answer an ASAP
 * request within 10 ms, so this only counts. */
static wifi_ftm_resp_status_t answer_request(
    const wifi_ftm_peer_request_t *request, uint8_t *comeback_sec) {
  (void)request;
  (void)comeback_sec;
  portENTER_CRITICAL(&s_lock);
  s_status.requests++;
  portEXIT_CRITICAL(&s_lock);
  return WIFI_FTM_RESP_ACCEPT;
}

/* A host-driven Wi-Fi restart drops the callbacks; register them again
 * whenever the access point starts. */
static void on_ap_start(void *arg, esp_event_base_t base, int32_t id,
                        void *data) {
  s_register_callbacks = true;
  if (s_task)
    xTaskNotifyGive(s_task);
}

static bool register_callbacks(void) {
  wifi_ftm_resp_cb_config_t callbacks = {
      .vendor_ie_cb = fill_frame,
      .vendor_ie_len = FTM_FOLLOW_UP_IE_SIZE,
      .req_cb = answer_request,
      /* IEEE 802.1, also on frames the callback leaves empty. */
      .vendor_ie_oui = {0x00, 0x80, 0xc2},
  };
  return esp_wifi_get_tsf_time(WIFI_IF_AP) > 0 &&
         esp_wifi_ftm_register_callbacks(&callbacks) == ESP_OK;
}

/* The transfer receiver (ftm_time_transfer_cp.c) hands over each one. */
void ftm_time_responder_transfer(const ftm_time_transfer_t *transfer) {
  if (s_transfers)
    xQueueSend(s_transfers, transfer, 0);
}

static void on_message(uint32_t msg_id, const uint8_t *data, size_t length,
                       void *user) {
  if (msg_id == HOSTED_AVB_MSG_FTM_ASSOCIATIONS &&
      length == FTM_ASSOC_ADMISSION_SIZE)
    receive_associations(data, length);
}

static void send_status(void) {
  ftm_time_responder_status_t status;
  portENTER_CRITICAL(&s_lock);
  status = s_status;
  portEXIT_CRITICAL(&s_lock);
  status.version = FTM_TIME_RESPONDER_STATUS_VERSION;
  status.radio_boot = s_radio_boot;
  hosted_avb_peer_send(HOSTED_AVB_MSG_FTM_RESPONDER_STATUS,
                       (const uint8_t *)&status, sizeof(status));
}

static void responder_task(void *arg) {
  /* esp_hosted_avb starts with esp-hosted and owns the radio's identity;
   * the clock link starts with it. */
  hosted_avb_mac_map_t map;
  while (!hosted_avb_clock_link_map_mac(&map))
    vTaskDelay(pdMS_TO_TICKS(500));
  s_radio_boot = hosted_avb_radio_boot();
  while (hosted_avb_peer_register(HOSTED_AVB_MSG_FTM_ASSOCIATIONS, on_message,
                                  NULL) != ESP_OK ||
         esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                    on_ap_start, NULL) != ESP_OK)
    vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, "FTM time responder ready");
  int64_t next_status = esp_timer_get_time() + STATUS_PERIOD_MS * 1000LL;
  for (;;) {
    ftm_time_transfer_t transfer;
    if (xQueueReceive(s_transfers, &transfer, pdMS_TO_TICKS(500)) == pdTRUE)
      prepare_snapshot(&transfer);
    ulTaskNotifyTake(pdTRUE, 0);
    if (s_register_callbacks && register_callbacks()) {
      s_register_callbacks = false;
      ESP_LOGI(TAG, "Filling FTM frames with a Follow_Up");
    }
    if (esp_timer_get_time() >= next_status) {
      send_status();
      next_status += STATUS_PERIOD_MS * 1000LL;
    }
  }
}

__attribute__((constructor)) static void start_responder(void) {
  s_transfers = xQueueCreate(4, sizeof(ftm_time_transfer_t));
  if (!s_transfers ||
      xTaskCreate(responder_task, "ftm_responder", 4096, NULL, 4, &s_task) !=
          pdPASS)
    ESP_LOGE(TAG, "No task; FTM frames carry no Follow_Up");
}

#endif
