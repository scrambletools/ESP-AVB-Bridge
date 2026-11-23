/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor side of the host's time transfer (ftm_time_transfer_t,
 * every 200 ms): one receiver for the FTM time responder and the beacon
 * time element.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ftm_time_responder_wire.h"

/* The latest usable transfer (valid, for this coprocessor boot, a sane
 * rate) and when it arrived (esp_timer_get_time). False before the first
 * or after the host sent one that isn't usable. */
bool ftm_time_transfer_latest(ftm_time_transfer_t *transfer,
                              int64_t *received_us);

/* The FTM time responder's intake: every transfer, usable or not, as it
 * arrives (on esp_hosted_avb's receive task). */
void ftm_time_responder_transfer(const ftm_time_transfer_t *transfer);
