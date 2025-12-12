idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES ld2410_enhanced light_driver zcl_utility
    PRIV_REQUIRES nvs_flash esp-zigbee-lib driver
)
