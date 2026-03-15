cmake_minimum_required(VERSION 3.20...4.1)
project(BallisticsLib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure UTF-8 source and execution character sets on MSVC so that
# µ (U+00B5) in printf strings is handled correctly regardless of the
# system ANSI code page.
if(MSVC)
    add_compile_options(/utf-8)
endif()

set(PROJECT_NAME ballistics)

# Default to Release for real-time performance
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

# CMake 4.x requires cmake_minimum_required >= 3.5, but some fetched
# dependencies (raylib 5.0, nlohmann_json 3.11) declare lower versions.
# Override their minimum policy version so they build cleanly under CMake 4.x.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

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
    include/${PROJECT_NAME}/async_solver.hpp
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
    src/async_solver.cpp
)

add_library(ballistics ${BALLISTICS_SOURCES})

target_include_directories(ballistics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json Threads::Threads)

target_include_directories (${PROJECT_NAME}
    PUBLIC include
)

# Optimisation flags for real-time performance.
#
# -ffast-math was previously used here but is intentionally avoided:
# it implies -ffinite-math-only (making std::isnan/isinf UB) and allows
# FP reordering that can silently change trajectory results.  The per-step
# cost is already ~200 ns, so the marginal gain does not justify the risk.
#
# -fno-math-errno and -freciprocal-math provide most of the benefit safely:
#   -fno-math-errno  — skip setting errno on math calls (e.g. sqrt)
#   -freciprocal-math — allow x/y → x*(1/y) where safe
target_compile_options(ballistics PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -march=native -fno-math-errno -freciprocal-math>
    $<$<CXX_COMPILER_ID:MSVC>:/O2>
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
