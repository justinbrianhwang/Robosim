cmake_minimum_required(VERSION 3.20)
project(RoboSim VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build options
option(ROBOSIM_BUILD_GUI "Build with OpenGL GUI support" ON)
option(ROBOSIM_BUILD_PYTHON "Build Python bindings" OFF)
option(ROBOSIM_BUILD_EXAMPLES "Build examples" ON)
option(ROBOSIM_USE_ONNX "Build with ONNX Runtime support" OFF)

# Include dependency management
include(cmake/dependencies.cmake)

# ============================================================================
# Core Library
# ============================================================================
file(GLOB_RECURSE CORE_SOURCES
    src/core/*.cpp
    src/config/*.cpp
    src/physics/*.cpp
    src/robot/*.cpp
    src/env/*.cpp
    src/reward/*.cpp
    src/terrain/*.cpp
    src/sensor/*.cpp
    src/task/*.cpp
    src/randomization/*.cpp
    src/checkpoint/*.cpp
    src/logging/*.cpp
    src/parallel/*.cpp
    src/ai/*.cpp
)

add_library(robosim_core STATIC ${CORE_SOURCES})
target_include_directories(robosim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(robosim_core PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    spdlog::spdlog
    ${BULLET_LIBRARIES}
    pthread
)

if(ROBOSIM_USE_ONNX AND onnxruntime_FOUND)
    target_link_libraries(robosim_core PUBLIC onnxruntime)
    target_compile_definitions(robosim_core PUBLIC ROBOSIM_HAS_ONNX=1)
endif()

# ============================================================================
# Renderer Library (optional GUI)
# ============================================================================
if(ROBOSIM_BUILD_GUI)
    file(GLOB_RECURSE RENDER_SOURCES src/render/*.cpp)

    add_library(robosim_render STATIC
        ${RENDER_SOURCES}
        ${GLAD_SOURCE}
        ${IMGUI_SOURCES}
    )
    target_include_directories(robosim_render PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${GLAD_INCLUDE_DIR}
        ${IMGUI_INCLUDE_DIR}
    )
    target_link_libraries(robosim_render PUBLIC
        robosim_core
        glfw
        ${OPENGL_LIBRARIES}
        dl
    )
    target_compile_definitions(robosim_render PUBLIC ROBOSIM_HAS_GUI=1)
endif()

# ============================================================================
# Main Executable
# ============================================================================
add_executable(robosim src/main.cpp)
target_link_libraries(robosim PRIVATE robosim_core)

if(ROBOSIM_BUILD_GUI)
    target_link_libraries(robosim PRIVATE robosim_render)
endif()

# Copy shaders to build directory
file(GLOB SHADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*)
file(COPY ${SHADER_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/shaders)

# Copy config to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# ============================================================================
# Python Bindings (optional)
# ============================================================================
if(ROBOSIM_BUILD_PYTHON)
    find_package(pybind11 CONFIG)
    if(pybind11_FOUND)
        pybind11_add_module(_robosim_core python/robosim_bindings.cpp)
        target_link_libraries(_robosim_core PRIVATE robosim_core)
        if(ROBOSIM_BUILD_GUI)
            target_link_libraries(_robosim_core PRIVATE robosim_render)
        endif()
    endif()
endif()

# ============================================================================
# Examples
# ============================================================================
if(ROBOSIM_BUILD_EXAMPLES)
    add_executable(basic_sim examples/basic_sim.cpp)
    target_link_libraries(basic_sim PRIVATE robosim_core)
endif()

# Installation
install(TARGETS robosim RUNTIME DESTINATION bin)
install(DIRECTORY include/robosim DESTINATION include)
install(DIRECTORY config DESTINATION share/robosim)
install(DIRECTORY shaders DESTINATION share/robosim)
