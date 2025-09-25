idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver led motors esp_wifi nvs_flash json
                       PRIV_REQUIRES mpu6050 i2c_bus
                       LDFRAGMENTS linker_fragment.lf)