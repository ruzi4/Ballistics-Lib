add_executable(realtime_demo realtime_demo.cpp)
target_link_libraries(realtime_demo PRIVATE ballistics)

# Copy munitions data next to the built binary so the demo can find it
add_custom_command(TARGET realtime_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data
        $<TARGET_FILE_DIR:realtime_demo>/data
)
