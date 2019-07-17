cmake_minimum_required(VERSION 3.5)

project(MHT NONE)

include(ExternalProject)
ExternalProject_Add(MHT
        GIT_REPOSITORY    https://github.com/PPokorski/MHT.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_CURRENT_BINARY_DIR}/MHT-src"
        BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/MHT-build"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
        TEST_COMMAND      "")