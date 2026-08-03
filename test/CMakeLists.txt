find_package(rostest REQUIRED)

catkin_add_gtest(test_joint_index_monitor src/joint_index_monitor.cpp)

add_rostest_gtest(test_joint_index_check test/joint_index_check.test
    src/joint_index_check.cpp)
target_link_libraries(test_joint_index_check ${catkin_LIBRARIES})

add_rostest_gtest(test_joint_trajectory test/joint_trajectory.test
    src/joint_trajectory.cpp)
target_link_libraries(test_joint_trajectory ${catkin_LIBRARIES})

add_rostest_gtest(test_cmd_vel test/cmd_vel.test
    src/cmd_vel.cpp)
target_link_libraries(test_cmd_vel ${catkin_LIBRARIES})
