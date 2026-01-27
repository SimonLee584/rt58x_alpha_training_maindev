sdk_generate_library(rt582_system)

if((${CONFIG_CHIP} STREQUAL "RT582") OR (${CONFIG_CHIP} STREQUAL "RT581"))
sdk_set_linker_script(${CMAKE_CURRENT_LIST_DIR}/ldscripts/rt582.ld)
endif()

if(${CONFIG_CHIP} STREQUAL "RT583")
sdk_set_linker_script(${CMAKE_CURRENT_LIST_DIR}/ldscripts/rt583.ld)
endif()
add_definitions(-D__STARTUP_CLEAR_BSS)
sdk_add_include_directories(Include)

if(CONFIG_BOOTLOADER)
    sdk_library_add_sources(
        Src/system_mcu.c
    )
else()
    sdk_library_add_sources(
        Startup/startup.S
        Src/system_mcu.c
        Src/mp_sector.c
    )
endif()
