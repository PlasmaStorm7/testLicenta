idf_component_register(SRCS "nmea_example_uart.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "igrr__libnmea")
# if(CONFIG_EXAMPLE_NMEA_INTERFACE_UART)
#     target_sources(${COMPONENT_LIB} PRIVATE nmea_example_uart.c)
# elseif(CONFIG_EXAMPLE_NMEA_INTERFACE_I2C)
#     target_sources(${COMPONENT_LIB} PRIVATE nmea_example_i2c.c)
# endif()
