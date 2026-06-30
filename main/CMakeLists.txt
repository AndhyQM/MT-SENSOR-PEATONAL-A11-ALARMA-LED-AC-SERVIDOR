set(srcs "main.c")

idf_component_register(SRCS "main.c" "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nimble_server nvs_flash freertos json driver)