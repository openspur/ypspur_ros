cmake_minimum_required(VERSION 2.8.3)
project(ypspur_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  rostest
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf
  message_generation
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
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS chrono thread atomic REQUIRED)

find_package(ypspur REQUIRED)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. ")
endif()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ypspur_INCLUDE_DIRS}
)

add_executable(ypspur_ros src/ypspur_ros.cpp)
target_link_libraries(ypspur_ros
  ${catkin_LIBRARIES}
  ypspur
)
add_dependencies(ypspur_ros ypspur_ros_gencpp)

add_executable(joint_tf_publisher src/joint_tf_publisher.cpp)
target_link_libraries(joint_tf_publisher ${catkin_LIBRARIES})

add_executable(joint_position_to_joint_trajectory src/joint_position_to_joint_trajectory.cpp)
target_link_libraries(joint_position_to_joint_trajectory ${catkin_LIBRARIES})
add_dependencies(joint_position_to_joint_trajectory ypspur_ros_gencpp)

#############
## Install ##
#############

install(TARGETS ypspur_ros ypspur_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  #launch/.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_cpp()
if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
