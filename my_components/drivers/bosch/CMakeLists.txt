idf_component_register(SRCS "bstdr_comm_support.c" "bmp280.c"
                       INCLUDE_DIRS "." "interface"
                     REQUIRES i2c_bus crazyflie platform config)