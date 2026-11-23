/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Receives the host's time transfer on the coprocessor (see
 * ftm_time_transfer_cp.h): hands every one to the FTM time responder,
 * when built, and keeps the latest usable one for the beacon time
 * element.
 */
#include "sdkconfig.h"

#if CONFIG_FTM_TIME_TRANSFER && CONFIG_IDF_TARGET_ESP32C6

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ftm_time_transfer_cp.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

/* gPTP ns per timer ns within ±2000 ppm, as Q32 */
#define RATE_MIN_Q32 INT64_C(4286377361)
#define RATE_MAX_Q32 INT64_C(4303557231)

static const char *TAG = "time_transfer";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static ftm_time_transfer_t s_latest;
static int64_t s_latest_us;
static bool s_have;
static uint32_t s_host, s_publication;

static bool usable(const ftm_time_transfer_t *transfer) {
  return transfer->valid && transfer->generation &&
         transfer->coprocessor_boot ==
             hosted_avb_clock_link_coprocessor_boot() &&
         transfer->rate_q32 >= RATE_MIN_Q32 &&
         transfer->rate_q32 <= RATE_MAX_Q32 && transfer->reference_ns >= 0 &&
         transfer->reference_ns <= INT64_MAX - INT64_C(4000000000);
}

bool ftm_time_transfer_latest(ftm_time_transfer_t *transfer,
                              int64_t *received_us) {
  portENTER_CRITICAL(&s_lock);
  bool have = s_have;
  if (have) {
    *transfer = s_latest;
    *received_us = s_latest_us;
  }
  portEXIT_CRITICAL(&s_lock);
  return have;
}

static void on_transfer(uint32_t msg_id, const uint8_t *data, size_t length,
                        void *user) {
  ftm_time_transfer_t transfer;
  if (msg_id != HOSTED_AVB_MSG_FTM_TRANSFER || length != sizeof(transfer))
    return;
  memcpy(&transfer, data, sizeof(transfer));
#if CONFIG_FTM_TIME_RESPONDER
  ftm_time_responder_transfer(&transfer);
#endif
  if (transfer.version != FTM_TIME_TRANSFER_VERSION)
    return;
  /* Only newer publications from the same host boot. */
  if (s_host == transfer.host_boot &&
      (int32_t)(transfer.publication - s_publication) <= 0)
    return;
  s_host = transfer.host_boot;
  s_publication = transfer.publication;
  bool ok = usable(&transfer);
  int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_lock);
  s_have = ok;
  if (ok) {
    s_latest = transfer;
    s_latest_us = now_us;
  }
  portEXIT_CRITICAL(&s_lock);
}

static void register_task(void *arg) {
  while (hosted_avb_peer_register(HOSTED_AVB_MSG_FTM_TRANSFER, on_transfer,
                                  NULL) != ESP_OK)
    vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, "Receiving the host's gPTP time");
  vTaskDelete(NULL);
}

__attribute__((constructor)) static void start_transfer(void) {
  if (xTaskCreate(register_task, "time_transfer", 2560, NULL, 4, NULL) !=
      pdPASS)
    ESP_LOGE(TAG, "No task; no gPTP time from the host");
}

#endif
