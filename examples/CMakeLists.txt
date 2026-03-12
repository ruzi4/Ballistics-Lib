add_executable(realtime_demo realtime_demo.cpp)
target_link_libraries(realtime_demo PRIVATE ballistics)

add_executable(fire_control_async fire_control_async.cpp)
find_package(Threads REQUIRED)
target_link_libraries(fire_control_async PRIVATE ballistics Threads::Threads)

# Copy munitions data next to each built binary so demos can find it
foreach(target realtime_demo fire_control_async)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/data
            $<TARGET_FILE_DIR:${target}>/data
    )
endforeach()

# ---------------------------------------------------------------------------
# Ballistics Trajectory Renderer
# Requires: raylib 5.0 (3D rendering + windowing), raygui 4.0 (immediate-mode GUI)
# ---------------------------------------------------------------------------
include(FetchContent)

# raylib
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.0
    GIT_SHALLOW    TRUE
)
set(BUILD_EXAMPLES    OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES       OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING     OFF CACHE BOOL "" FORCE)
set(CUSTOMIZE_BUILD   OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

# raygui (single-header immediate-mode GUI companion for raylib)
FetchContent_Declare(
    raygui
    GIT_REPOSITORY https://github.com/raysan5/raygui.git
    GIT_TAG        4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raygui)

add_executable(ballistics_renderer renderer.cpp)

target_link_libraries(ballistics_renderer PRIVATE
    ballistics
    raylib
    Threads::Threads
)

# raygui is header-only; just add its src/ directory to the include path
target_include_directories(ballistics_renderer PRIVATE
    ${raygui_SOURCE_DIR}/src
)

target_compile_features(ballistics_renderer PRIVATE cxx_std_17)

# Copy munitions data next to the renderer binary
add_custom_command(TARGET ballistics_renderer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data
        $<TARGET_FILE_DIR:ballistics_renderer>/data
)
