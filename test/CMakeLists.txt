find_package(rostest REQUIRED)

add_rostest_gtest(test_joint_trajectory test/joint_trajectory.test
    src/joint_trajectory.cpp)
target_link_libraries(test_joint_trajectory ${catkin_LIBRARIES})

add_rostest_gtest(test_cmd_vel test/cmd_vel.test
    src/cmd_vel.cpp)
target_link_libraries(test_cmd_vel ${catkin_LIBRARIES})
