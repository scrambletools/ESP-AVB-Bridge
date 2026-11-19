idf_component_register(SRCS "coprocessor_fw.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_avb espressif__esp_hosted
                                     esp_app_format bootloader_support log)
