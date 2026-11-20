set(srcs "avb_bridge_main.c")

if(CONFIG_AVB_BRIDGE_PERF_TEST_MODE)
    list(APPEND srcs "avb_bridge_perf_test.c")
endif()

# The board's flash needs its own chip driver for auto-suspend.
if(CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST)
    list(APPEND srcs "flash_chips.c")
endif()

# The project builds only these and what they require (see the
# top-level CMakeLists.txt), so list everything main includes; the perf
# test uses NVS.
idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES esp_avb esp_ptp esp_hosted_avb esp_eth
                                     ftm_time_responder
                                     esp_netif esp_event esp_timer esp_wifi
                                     esp_driver_gpio esp_driver_uart esp_rom heap
                                     log nvs_flash spi_flash
                                     espressif__ip101 espressif__esp_hosted)
