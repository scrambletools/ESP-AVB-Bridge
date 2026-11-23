/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Time transfer, host side (see ftm_time_responder.h). Every 200 ms it
 * sends the coprocessor gPTP time on the coprocessor's clock-link timer,
 * from the link's fit while this bridge's PTP clock follows the
 * grandmaster, for the FTM time responder and the beacon time element.
 * For the responder it also sends the upstream Follow_Up to relay and
 * the stations esp_ptp has admitted on the Wi-Fi port, and logs the
 * coprocessor's status (FTMRESP lines) every 10 s.
 */
#include "ftm_time_responder.h"

#include "sdkconfig.h"

#if CONFIG_FTM_TIME_TRANSFER && CONFIG_IDF_TARGET_ESP32P4

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ftm_radio_association.h"
#include "ftm_time_responder_wire.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"
#include "ptp_timing_snapshot.h"
#include "ptp_wifi_association.h"

#define PUBLISH_PERIOD_MS 200
#define SOURCE_MAX_OFFSET_NS 1000 /* only while the clock follows the GM */
#define WIFI_PORT_INDEX 1
#define WIFI_PORT_NUMBER 2

static const char *TAG = "ftm_responder";

static TaskHandle_t s_task;
static uint32_t s_host_boot;

#if CONFIG_BEACON_TIME_FROM_LINK
static void on_beacon_status(uint32_t msg_id, const uint8_t *data,
                             size_t length, void *user) {
  beacon_time_status_t status;
  if (msg_id != HOSTED_AVB_MSG_BEACON_TIME_STATUS || length != sizeof(status))
    return;
  memcpy(&status, data, sizeof(status));
  if (status.version != BEACON_TIME_STATUS_VERSION)
    return;
  ESP_LOGI(TAG,
           "BEACONTIME,filled %" PRIu32 ",left to the host %" PRIu32
           " (no gPTP time %" PRIu32 ", no TSF read %" PRIu32
           "),TSF read %" PRIu32 "-%" PRIu32 " ns,rate %+.3f ppm,"
           "TSF less MAC clock %" PRId32 "..%" PRId32 " us before %" PRId32
           "..%" PRId32 " us after",
           status.filled, status.no_time + status.no_tsf, status.no_time,
           status.no_tsf, status.read_min_ns, status.read_max_ns,
           status.rate_ppb / 1000.0, status.tsf_less_mac_before_min,
           status.tsf_less_mac_before_max, status.tsf_less_mac_after_min,
           status.tsf_less_mac_after_max);
}
#endif

#if CONFIG_FTM_TIME_RESPONDER
static void on_status(uint32_t msg_id, const uint8_t *data, size_t length,
                      void *user) {
  ftm_time_responder_status_t status;
  if (msg_id != HOSTED_AVB_MSG_FTM_RESPONDER_STATUS || length != sizeof(status))
    return;
  memcpy(&status, data, sizeof(status));
  if (status.version != FTM_TIME_RESPONDER_STATUS_VERSION)
    return;
  ESP_LOGI(TAG,
           "FTMRESP,requests %" PRIu32 ",frames filled %" PRIu32
           " declined %" PRIu32 " (station not admitted %" PRIu32
           ", no fresh time %" PRIu32 "),transfers %" PRIu32
           " rejected %" PRIu32 ",MAC maps failed %" PRIu32,
           status.requests, status.filled, status.declined, status.not_admitted,
           status.no_snapshot, status.transfers, status.transfers_rejected,
           status.maps_failed);
}


/* The stations esp_ptp has on its Wi-Fi port, with their logical ports
 * and whether they asked Sync to stop. */
static void publish_associations(const ptp_timing_snapshot_t *source) {
  static uint32_t publication;
  ptp_wifi_association_snapshot_t snapshot;
  if (!ptpd_wifi_association_snapshot(WIFI_PORT_INDEX, &snapshot) ||
      !snapshot.radio_boot || !snapshot.radio_generation)
    return;
  if (!++publication)
    ++publication;
  ftm_association_packet_t packet = {
      .host_boot = s_host_boot,
      .radio_boot = snapshot.radio_boot,
      .publication = publication,
      .count = snapshot.count,
  };
  memcpy(packet.clock_identity, source->own_identity, 8);
  for (unsigned i = 0; i < snapshot.count && i < FTM_ASSOC_MAX; i++) {
    memcpy(packet.entries[i].mac, snapshot.entries[i].mac, 6);
    packet.entries[i].port_number = snapshot.entries[i].port_number;
    packet.entries[i].association = snapshot.entries[i].association;
    packet.entries[i].sync_stopped = snapshot.entries[i].sync_stopped;
    packet.entries[i].sync_revision = snapshot.entries[i].sync_revision;
  }
  uint8_t wire[FTM_ASSOC_ADMISSION_SIZE];
  if (ftm_radio_association_encode(snapshot.radio_generation, &packet, wire,
                                   sizeof(wire)))
    hosted_avb_peer_send(HOSTED_AVB_MSG_FTM_ASSOCIATIONS, wire, sizeof(wire));
}
#endif

static void publish_time(void) {
  static uint32_t publication;
  ptp_timing_snapshot_t source;
  bool source_valid = ptpd_time_source_snapshot(&source);
#if CONFIG_FTM_TIME_RESPONDER
  if (source_valid)
    publish_associations(&source);
#endif
  ftm_time_transfer_t transfer = {
      .version = FTM_TIME_TRANSFER_VERSION,
      .host_boot = s_host_boot,
      .publication = ++publication,
      .generation = source_valid ? source.generation : 0,
  };
  int64_t cp_ns, host_ns;
  double rate;
  uint32_t coprocessor_boot;
  if (source_valid && source.hardware_clock &&
      source.offset_ns >= -SOURCE_MAX_OFFSET_NS &&
      source.offset_ns <= SOURCE_MAX_OFFSET_NS &&
      hosted_avb_clock_link_fit(&cp_ns, &host_ns, &rate, &coprocessor_boot)) {
    transfer.coprocessor_boot = coprocessor_boot;
    transfer.cp_anchor_ns = cp_ns;
    transfer.reference_ns = host_ns; /* this clock follows the GM */
    transfer.rate_q32 = llround(rate * 4294967296.0);
    transfer.upstream_receive_ns = source.local_receive_ns;
    transfer.peer_delay_ns = source.peer_delay_ns;
    memcpy(transfer.follow_up, source.follow_up, sizeof(transfer.follow_up));
    memcpy(transfer.source_port, source.own_identity, 8);
    transfer.source_port[9] = WIFI_PORT_NUMBER;
    memcpy(transfer.btc_identity, source.btc_identity, 8);
    transfer.valid = 1;
  }
  hosted_avb_peer_send(HOSTED_AVB_MSG_FTM_TRANSFER, (const uint8_t *)&transfer,
                       sizeof(transfer));
}

static void responder_task(void *arg) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PUBLISH_PERIOD_MS));
    publish_time();
  }
}

esp_err_t ftm_time_responder_start(void) {
  if (s_task)
    return ESP_OK;
  s_host_boot = esp_random() | 1;
#if CONFIG_FTM_TIME_RESPONDER
  esp_err_t err = hosted_avb_peer_register(HOSTED_AVB_MSG_FTM_RESPONDER_STATUS,
                                           on_status, NULL);
  if (err != ESP_OK)
    return err;
#endif
#if CONFIG_BEACON_TIME_FROM_LINK
  esp_err_t beacon_err = hosted_avb_peer_register(
      HOSTED_AVB_MSG_BEACON_TIME_STATUS, on_beacon_status, NULL);
  if (beacon_err != ESP_OK)
    return beacon_err;
#endif
  if (xTaskCreate(responder_task, "ftm_responder", 4096, NULL, 3, &s_task) !=
      pdPASS)
    return ESP_ERR_NO_MEM;
  ESP_LOGI(TAG, "Feeding the coprocessor gPTP time"
#if CONFIG_FTM_TIME_RESPONDER
                ", FTM responder"
#endif
#if CONFIG_BEACON_TIME_FROM_LINK
                ", beacon time element"
#endif
  );
  return ESP_OK;
}

#endif
