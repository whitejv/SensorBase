# Component CMakeLists.txt for sensor_system
idf_component_register(SRCS "src/system_init.c"
                            "src/watchdog.c"
                            "src/error_recovery.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver)
