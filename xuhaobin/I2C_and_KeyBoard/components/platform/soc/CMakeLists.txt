if(CONFIG_BOOTLOADER)
    if((${CONFIG_CHIP} STREQUAL "RT582_NONE_OS"))
        sdk_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_bootloader)
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_driver) 
        sdk_add_subdirectory_ifdef(CONFIG_CRYPTO ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_crypto)
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_system)
    elseif((${CONFIG_CHIP} STREQUAL "RT584_NONE_OS"))
        sdk_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_bootloader)
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_driver)
        sdk_add_subdirectory_ifdef(CONFIG_CRYPTO ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_crypto)
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_system)
    endif()
else()
if((${CONFIG_CHIP} STREQUAL "RT582") OR (${CONFIG_CHIP} STREQUAL "RT581") OR (${CONFIG_CHIP} STREQUAL "RT583"))
       
    sdk_add_subdirectory_ifdef(CONFIG_CRYPTO ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_crypto)
    sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_driver) 
    
    if((${CONFIG_BOARD} STREQUAL "DONGLE"))
        sdk_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_dongle)
    elseif((${CONFIG_BOARD} STREQUAL "EVB"))
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt582/rt582_system)
    endif()
endif()

if((${CONFIG_CHIP} STREQUAL "RT584H") OR (${CONFIG_CHIP} STREQUAL "RT584L"))
    

    sdk_add_subdirectory_ifdef(CONFIG_CRYPTO ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_crypto)
    sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_driver)
    
    if((${CONFIG_BOARD} STREQUAL "DONGLE"))
        sdk_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_dongle)
    elseif((${CONFIG_BOARD} STREQUAL "EVB"))
        sdk_add_subdirectory_ifdef(CONFIG_DRIVER ${CMAKE_CURRENT_LIST_DIR}/rt584/rt584_system)
    endif()

endif()

endif()



