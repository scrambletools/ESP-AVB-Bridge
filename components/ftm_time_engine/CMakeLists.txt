set(srcs)
if(CONFIG_FTM_TIME_ENGINE)
  list(APPEND srcs "ftm_time_engine.c")
endif()
idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES esp_wifi esp_event esp_hw_support esp_timer
                                     esp_hosted_avb ftm_follow_up
                       WHOLE_ARCHIVE)
