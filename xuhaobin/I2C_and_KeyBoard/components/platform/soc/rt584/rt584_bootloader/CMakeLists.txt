sdk_generate_library(rt584h_bootloader)
sdk_set_linker_script(${CMAKE_CURRENT_LIST_DIR}/bootloader/ld/rt584.ld)
sdk_add_include_directories(bootloader/src)
sdk_add_include_directories(include)
add_definitions(-DCHIP_NAME="584" )

if(CONFIG_BOOTLOADER_DEBUG)
sdk_library_add_sources(
    bootloader/src/boot/gcc/startup.c
)
else()
sdk_library_add_sources(
    bootloader/src/boot/gcc/startup.c
)
endif()