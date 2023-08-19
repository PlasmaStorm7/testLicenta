idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "lora" "log" "freertos" "console" "fatfs" "nvs_flash" "cmd_nvs" "cmd_system" "cmd_wifi" "soc" "igrr__libnmea" "nmea_example"
                    # "libnmea"
                    )
