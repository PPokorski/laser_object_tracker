cmake_minimum_required(VERSION 3.5)

project(mht C CXX)

include(ExternalProject)
ExternalProject_Add(mht
        GIT_REPOSITORY    https://github.com/PPokorski/MHT.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_CURRENT_BINARY_DIR}/mht-src"
        BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/mht-build"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
        TEST_COMMAND      "")