idf_component_register(SRCS "gcs_comm.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_system log json led motors esp_wifi esp_netif nvs_flash)