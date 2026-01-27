message("\t+ mesh-it-up components")

set(OT_PLATFORM "external" CACHE STRING "")
option(OT_BUILD_EXECUTABLES "" OFF)
option(BUILD_TESTING "" OFF)

# === OpenThread library selection ===
if(CONFIG_MIU_DEVICE_TYPE_FTD)
    sdk_add_link_libraries(openthread-ftd)
    sdk_add_link_libraries(openthread-cli-ftd)
    option(OT_FTD "" ON)
    option(OT_MTD "" OFF)
    option(OT_RCP "" OFF)
elseif(CONFIG_MIU_DEVICE_TYPE_MTD)
    sdk_add_link_libraries(openthread-mtd)
    sdk_add_link_libraries(openthread-cli-mtd)
    option(OT_FTD "" OFF)
    option(OT_MTD "" ON)
    option(OT_RCP "" OFF)
elseif(CONFIG_MIU_DEVICE_TYPE_RCP)
    sdk_add_link_libraries(openthread-rcp)
    sdk_add_link_libraries(openthread-cli-radio)
    option(OT_FTD "" OFF)
    option(OT_MTD "" OFF)
    option(OT_RCP "" ON)
endif()

# === common include path ===
sdk_add_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/miu_openthread/include/openthread
    ${CMAKE_CURRENT_LIST_DIR}/miu_openthread/include/openthread/platform
    ${CMAKE_CURRENT_LIST_DIR}/miu_openthread/src/core
    ${CMAKE_CURRENT_LIST_DIR}/miu_openthread/src/core/config
    ${CMAKE_CURRENT_LIST_DIR}/miu_openthread/src
    ${CMAKE_CURRENT_LIST_DIR}/miu_port/include
)

# === version info ===
if (${CONFIG_CHIP} MATCHES "RT58[123]")
    set(OT_PACKAGE_NAME "OT_RT58X" CACHE STRING "OpenThread Package Name")
elseif (${CONFIG_CHIP} MATCHES "RT584[HLS]")
    set(OT_PACKAGE_NAME "OT_RT584" CACHE STRING "OpenThread Package Name")
endif()

set(GIT_COMMIT_HASH "00000000")
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE GIT_HASH_RESULT
    OUTPUT_VARIABLE GIT_COMMIT_HASH_TMP
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(GIT_HASH_RESULT EQUAL 0)
    set(GIT_COMMIT_HASH "${GIT_COMMIT_HASH_TMP}")
endif()

execute_process(
    COMMAND python -c
    "from datetime import datetime; t=datetime.now(); print('{:08X}'.format(((t.year-2000)&0x7F)<<25 | ((t.month)&0x0F)<<21 | ((t.day)&0x1F)<<16 | ((t.hour)&0x1F)<<11 | ((t.minute)&0x3F)<<5 | ((t.second)&0x1F)))"
    OUTPUT_VARIABLE BUILD_TIME_4BYTE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(OT_PACKAGE_VERSION "${GIT_COMMIT_HASH}-${BUILD_TIME_4BYTE}" CACHE STRING "OpenThread Package Version")

# === mbedTLS setting ===
add_compile_options(
    -I${CMAKE_CURRENT_LIST_DIR}/miu_port/include
    -I${CMAKE_CURRENT_LIST_DIR}/miu_openthread/third_party/mbedtls/repo/include
    -DMBEDTLS_AES_ALT=1
    -DMBEDTLS_SHA256_ALT=1
    -DMBEDTLS_ECJPAKE_ALT=1
)

# === OT config ===
add_compile_options(-DOPENTHREAD_CONFIG_PLATFORM_INFO="${CONFIG_CHIP}")

if(CONFIG_MIU_DEVICE_TYPE_FTD)
    set(APP_CONFIG_ROOT_DIR "${CMAKE_SOURCE_DIR}/examples/sub-g/mesh-it-up/miu-ftd/include")
elseif(CONFIG_MIU_DEVICE_TYPE_MTD)
    set(APP_CONFIG_ROOT_DIR "${CMAKE_SOURCE_DIR}/examples/sub-g/mesh-it-up/miu-mtd/include")
elseif(CONFIG_MIU_DEVICE_TYPE_RCP)
    set(APP_CONFIG_ROOT_DIR "${CMAKE_SOURCE_DIR}/examples/sub-g/mesh-it-up/miu-sniffer/include")
endif()

if (EXISTS "${APP_CONFIG_ROOT_DIR}/app_miu_config.h")
    message(STATUS "Found Application Config at: ${APP_CONFIG_ROOT_DIR}")
    sdk_add_include_directories("${APP_CONFIG_ROOT_DIR}")
    add_compile_options(-DOPENTHREAD_CONFIG_FILE=\"${APP_CONFIG_ROOT_DIR}/app_miu_config.h\")
else()
    message(WARNING "Application config file not found. Check relative path.")
endif()

add_compile_options(
    -D__int64_t_defined=1
    -DOT_FREERTOS_ENABLE=1
    -Wimplicit-fallthrough=0 -Wno-switch-default
)

# === subdir ===
sdk_add_subdirectory_ifdef(CONFIG_BUILD_COMPONENT_MESH_IT_UP ${CMAKE_CURRENT_LIST_DIR}/miu_openthread)

# === build miu_port static lib ===
sdk_add_subdirectory_ifdef(CONFIG_BUILD_COMPONENT_MESH_IT_UP ${CMAKE_CURRENT_LIST_DIR}/miu_port)
sdk_add_link_libraries(miu_port_lib)
