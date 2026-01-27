# sdk_generate_component_tag()
if (CONFIG_HOSAL_SOC_IDLE_SLEEP)
sdk_generate_library(zboss_port_rfd)
else()
sdk_generate_library(zboss_port_ffd)
endif()

sdk_add_include_directories(
    include
)

sdk_library_add_sources(
    src/zb_crypto.c
    src/zb_freertos.c
    src/zb_nvram.c
    src/zb_powersaving.c
    src/zb_radio.c
    src/zb_serial.c
    src/zb_timer.c
)

if (CONFIG_HOSAL_SOC_IDLE_SLEEP)
    add_definitions(
        -DZB_ED_ROLE
        -DZB_USE_SLEEP
    )
else()
endif()