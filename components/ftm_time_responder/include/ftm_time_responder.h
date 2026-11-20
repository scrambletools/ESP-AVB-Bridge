/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * FTM time responder for the bridge (CONFIG_FTM_TIME_RESPONDER): the
 * access point side of 802.1AS FTM time transfer. The coprocessor, the
 * FTM responder, puts a Follow_Up in every FTM frame it sends an
 * admitted station, giving gPTP time at that frame's measurement. The
 * host keeps it fed with gPTP time on the coprocessor's clock-link timer
 * and the stations esp_ptp has admitted. Both chips need it and
 * CONFIG_HOSTED_AVB_CLOCK_LINK; the coprocessor also needs
 * CONFIG_ESP_WIFI_ENABLE_FTM_PTP.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Host: starts feeding the coprocessor; call after esp_ptp's Wi-Fi port
 * and the clock link have started. The coprocessor side starts itself. */
esp_err_t ftm_time_responder_start(void);

#ifdef __cplusplus
}
#endif
