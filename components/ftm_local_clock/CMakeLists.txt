set(clock_sources)
if(CONFIG_FTM_LOCAL_CLOCK)
    list(APPEND clock_sources "ftm_local_clock.c")
endif()
idf_component_register(SRCS ${clock_sources} INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi esp_ptp WHOLE_ARCHIVE)
