sdk_generate_component_tag()

sdk_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/fota)
sdk_add_subdirectory_ifdef(CONFIG_FREERTOS
                           ${CMAKE_CURRENT_LIST_DIR}/rtos_port)