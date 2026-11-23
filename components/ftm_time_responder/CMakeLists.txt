set(srcs)
set(requirements esp_timer esp_wifi esp_event esp_hw_support esp_hosted_avb
                 ftm_follow_up)
if(IDF_TARGET STREQUAL "esp32p4")
  list(APPEND requirements esp_ptp)
endif()
if(CONFIG_FTM_TIME_TRANSFER)
  if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND srcs "ftm_time_responder_host.c")
  else()
    list(APPEND srcs "ftm_time_transfer_cp.c")
    if(CONFIG_FTM_TIME_RESPONDER)
      list(APPEND srcs "ftm_time_responder_radio.c")
    endif()
    if(CONFIG_BEACON_TIME_FROM_LINK)
      list(APPEND srcs "beacon_time_link.c")
    endif()
  endif()
endif()
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES ${requirements}
                       WHOLE_ARCHIVE)
