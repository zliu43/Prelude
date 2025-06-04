idf_component_register(
    SRCS "MCU.c" "EVE_API.c" "EVE_HAL.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
