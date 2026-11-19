/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Offers the coprocessor's firmware to ATDECC controllers as a memory
 * object (CONFIG_ESP_AVB_COPROCESSOR_FW_OBJECT); call before avb_start.
 * Does nothing when the option is off. */
void coprocessor_fw_register(void);

#ifdef __cplusplus
}
#endif
