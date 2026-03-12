add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE ballistics)

add_test(NAME trajectory_tests COMMAND test_trajectory)
