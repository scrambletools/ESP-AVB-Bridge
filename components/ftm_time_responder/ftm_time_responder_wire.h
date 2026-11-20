/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * What the bridge's host and coprocessor send each other for the FTM time
 * responder, over esp_hosted_avb (IDs in hosted_avb_proto.h). Both chips
 * build this file; the layouts are fixed little-endian.
 */
#pragma once

#include <stdint.h>

#define FTM_TIME_TRANSFER_VERSION 1U

/* Host to coprocessor, every 200 ms: gPTP time on the coprocessor's
 * clock-link timer, and the upstream Follow_Up to relay. */
typedef struct __attribute__((packed)) {
  uint32_t version, host_boot, coprocessor_boot, generation, publication;
  uint32_t valid;
  int64_t cp_anchor_ns;        /* the clock-link timer at the anchor */
  int64_t reference_ns;        /* gPTP time there */
  int64_t rate_q32;            /* gPTP ns per timer ns, Q32 */
  int64_t upstream_receive_ns; /* the upstream Sync's receipt, gPTP */
  int32_t peer_delay_ns;
  uint32_t reserved;
  uint8_t follow_up[76];   /* the upstream Follow_Up */
  uint8_t source_port[10]; /* the bridge's identity on the Wi-Fi port */
  uint8_t btc_identity[8];
  uint8_t padding[2];
} ftm_time_transfer_t;

_Static_assert(sizeof(ftm_time_transfer_t) == 160, "FTM time transfer");

#define FTM_TIME_RESPONDER_STATUS_VERSION 1U

/* Coprocessor to host, every 10 s: what the responder did. */
typedef struct __attribute__((packed)) {
  uint32_t version, radio_boot;
  uint32_t requests;     /* FTM requests answered */
  uint32_t filled;       /* frames sent with a Follow_Up */
  uint32_t declined;     /* frames sent without one, by cause: */
  uint32_t not_admitted; /*   the station isn't on the admitted list */
  uint32_t no_snapshot;  /*   no fresh time for that frame */
  uint32_t transfers, transfers_rejected, maps_failed;
} ftm_time_responder_status_t;

_Static_assert(sizeof(ftm_time_responder_status_t) == 40,
               "FTM time responder status");
