#[[
@file CMakeLists.txt
@project PrecisionLander
@brief Build configuration for PrecisionLander project.
@author Alex Smith
@date 2026-01-09
@license [ADD LICENSE TYPE, e.g., MIT, Apache 2.0]
]]

cmake_minimum_required(VERSION 3.20)

project(PrecisionLander LANGUAGES CXX)

# ---- Build settings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# ---- Dependencies ----
set(OpenCV_DIR "" CACHE PATH "Path to OpenCVConfig.cmake (optional)")
find_package(OpenCV REQUIRED)

# ---- Target ----
add_executable(PrecisionLander
        src/main.cpp
)

# OpenCV includes (scoped to target)
target_include_directories(PrecisionLander PRIVATE ${OpenCV_INCLUDE_DIRS})

# ---- ArUco (restore previous working style, but keep target-based usage) ----
# Match your prior config:
set(ARUCO_INCLUDE_DIR "/usr/local/include/aruco" CACHE PATH "Aruco include directory")
set(ARUCO_LIBRARY_DIR "/usr/local/lib" CACHE PATH "Aruco library directory")
set(ARUCO_LIB_NAME "aruco" CACHE STRING "Aruco library name (e.g., aruco)")

target_include_directories(PrecisionLander PRIVATE ${ARUCO_INCLUDE_DIR})
target_link_directories(PrecisionLander PRIVATE ${ARUCO_LIBRARY_DIR})
target_link_libraries(PrecisionLander PRIVATE ${OpenCV_LIBS} ${ARUCO_LIB_NAME})

# Ensure the runtime loader can find libaruco.*.dylib when launching from the build tree
set_target_properties(PrecisionLander PROPERTIES
        BUILD_RPATH "${ARUCO_LIBRARY_DIR}"
        INSTALL_RPATH "${ARUCO_LIBRARY_DIR}"
)

# ---- Warnings (optional) ----
if(MSVC)
    target_compile_options(PrecisionLander PRIVATE /W4)
else()
    target_compile_options(PrecisionLander PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- Runtime assets ----
set(ASSETS_DIR "${CMAKE_SOURCE_DIR}/assets")
set(SCRIPTS_DIR "${CMAKE_SOURCE_DIR}/scripts")

set(RUNTIME_FILES
        "${ASSETS_DIR}/camera_result.xml"
        "${ASSETS_DIR}/overlay_image.png"
        "${SCRIPTS_DIR}/publisher.py"
)

add_custom_command(
        TARGET PrecisionLander POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${RUNTIME_FILES}
        $<TARGET_FILE_DIR:PrecisionLander>
        COMMENT "Copying runtime assets to output directory"
)
