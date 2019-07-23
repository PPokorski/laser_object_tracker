macro(configureExternalProject EXTERNAL_PROJECT_NAME CMAKE_FILE INCLUDE_PATH)
    configure_file(${CMAKE_FILE} ${EXTERNAL_PROJECT_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
                    RESULT_VARIABLE result
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${EXTERNAL_PROJECT_NAME} )
    if(result)
        message(FATAL_ERROR "CMake step for ${EXTERNAL_PROJECT_NAME} failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
                    RESULT_VARIABLE result
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${EXTERNAL_PROJECT_NAME} )
    if(result)
        message(FATAL_ERROR "Build step for ${EXTERNAL_PROJECT_NAME} failed: ${result}")
    endif()

    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/${EXTERNAL_PROJECT_NAME}-src
                     ${CMAKE_CURRENT_BINARY_DIR}/${EXTERNAL_PROJECT_NAME}-build
                     EXCLUDE_FROM_ALL)

    set(${EXTERNAL_PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/${EXTERNAL_PROJECT_NAME}-src/${INCLUDE_PATH})
endmacro()