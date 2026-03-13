add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE ballistics)

# Copy munitions data next to the test binary.
# $<TARGET_FILE_DIR:...> resolves to the correct per-config subdirectory
# (e.g. tests/Release/ or tests/Debug/) under MSVC multi-config generators,
# so CTest can find munitions.json regardless of build configuration.
add_custom_command(TARGET test_trajectory POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data
        $<TARGET_FILE_DIR:test_trajectory>/data
)

add_test(NAME trajectory_tests COMMAND test_trajectory)
