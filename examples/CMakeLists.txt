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
