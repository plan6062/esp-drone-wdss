idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver led motors esp_wifi nvs_flash json
                       LDFRAGMENTS linker_fragment.lf)