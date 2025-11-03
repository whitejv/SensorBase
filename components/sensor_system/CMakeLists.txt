# Component CMakeLists.txt for sensor_system
idf_component_register(SRCS "src/system_init.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver)
