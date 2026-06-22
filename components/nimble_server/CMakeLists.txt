idf_component_register(
    SRCS "nimble_server.c"
    INCLUDE_DIRS "include"
    REQUIRES nimble_common bt nvs_flash
)
