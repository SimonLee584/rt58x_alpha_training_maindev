

if((${CONFIG_CHIP} STREQUAL "RT582") OR (${CONFIG_CHIP} STREQUAL "RT581") OR (${CONFIG_CHIP} STREQUAL "RT583"))
    sdk_add_subdirectory_ifdef(CONFIG_HOSAL ${CMAKE_CURRENT_LIST_DIR}/rt582_hosal)
elseif((${CONFIG_CHIP} STREQUAL "RT584H") OR (${CONFIG_CHIP} STREQUAL "RT584L"))
    sdk_add_subdirectory_ifdef(CONFIG_HOSAL ${CMAKE_CURRENT_LIST_DIR}/rt584_hosal)
endif()

if((${CONFIG_CHIP} STREQUAL "RT582_NONE_OS"))
    sdk_add_subdirectory_ifdef(CONFIG_HOSAL ${CMAKE_CURRENT_LIST_DIR}/rt582_hosal)
elseif((${CONFIG_CHIP} STREQUAL "RT584_NONE_OS"))
    sdk_add_subdirectory_ifdef(CONFIG_HOSAL ${CMAKE_CURRENT_LIST_DIR}/rt584_hosal)
endif()
