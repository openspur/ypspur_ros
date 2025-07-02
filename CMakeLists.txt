cmake_minimum_required(VERSION 3.0.2)
project(ypspur_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation

  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

add_message_files(
  FILES
  DigitalOutput.msg
  DigitalInput.msg
  ControlMode.msg
  JointPositionControl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  message_runtime

  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS chrono thread atomic REQUIRED)

find_package(ypspur 1.20.0 REQUIRED)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 14)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ypspur_INCLUDE_DIRS}
)

add_executable(ypspur_ros src/ypspur_ros.cpp)
target_link_libraries(ypspur_ros
  ${catkin_LIBRARIES}
  ypspur
  libypspur-coordinator
)
add_dependencies(ypspur_ros ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_tf_publisher src/joint_tf_publisher.cpp)
target_link_libraries(joint_tf_publisher ${catkin_LIBRARIES})
add_dependencies(joint_tf_publisher ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_position_to_joint_trajectory src/joint_position_to_joint_trajectory.cpp)
target_link_libraries(joint_position_to_joint_trajectory ${catkin_LIBRARIES})
add_dependencies(joint_position_to_joint_trajectory ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS
    ypspur_ros
    joint_tf_publisher
    joint_position_to_joint_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
