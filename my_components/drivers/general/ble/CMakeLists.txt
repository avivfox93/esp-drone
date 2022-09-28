idf_component_register(SRCS "ble_esp32.cpp"
                       INCLUDE_DIRS "." "include/."
                     REQUIRES crazyflie platform config ble_hal)