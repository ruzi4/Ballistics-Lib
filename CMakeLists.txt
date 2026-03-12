cmake_minimum_required(VERSION 4.0)
project(BallisticsLib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_NAME ballistics)

# Default to Release for real-time performance
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------

set(BALLISTICS_INCLUDES
    include/${PROJECT_NAME}/atmosphere.hpp
    include/${PROJECT_NAME}/munition.hpp
    include/${PROJECT_NAME}/trajectory.hpp
    include/${PROJECT_NAME}/fire_control.hpp
    include/${PROJECT_NAME}/ballistics.hpp
    include/${PROJECT_NAME}/math/vec3.hpp
    include/${PROJECT_NAME}/math/math_constants.hpp
)

set(BALLISTICS_SOURCES
    ${BALLISTICS_INCLUDES}
    src/atmosphere.cpp
    src/munition.cpp
    src/trajectory.cpp
    src/fire_control.cpp
)

add_library(ballistics ${BALLISTICS_SOURCES})

target_include_directories(ballistics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)

target_include_directories (${PROJECT_NAME}
    PUBLIC include
)

# Aggressive optimisation flags for real-time performance
target_compile_options(ballistics PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -march=native -ffast-math>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /fp:fast>
)

# ---------------------------------------------------------------------------
# Optional sub-directories
# ---------------------------------------------------------------------------
option(BALLISTICS_BUILD_TESTS    "Build unit tests"      ON)
option(BALLISTICS_BUILD_EXAMPLES "Build example programs" ON)

if(BALLISTICS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BALLISTICS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
