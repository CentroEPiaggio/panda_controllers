cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Cose che ho aggiunto contrassegnate con ##
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  franka_msgs##
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

## added DesiredProjectTrajectory msg
add_message_files(FILES
  JointTorqueComparison.msg
  RobotState.msg
  DesiredImpedance.msg
  DesiredTrajectory.msg
  DesiredProjectTrajectory.msg
  EEpose.msg
)

##
generate_messages(
  DEPENDENCIES##
  franka_msgs##
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
)

##
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    franka_msgs##
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(${PROJECT_NAME}   
  src/cartesian_velocity_controller.cpp
  src/pdcontroller.cpp #PD controller
  src/computed_torque.cpp #Computed Torque controller
  # src/backstepping.cpp #Backstepping controller
  src/variable_impedance_controller.cpp #Backstepping controller
  src/project_impedance_controller.cpp #Project variable impedance controller
  src/planner.cpp #Planner
  src/Jacobians_ee.cpp #end-effector Jacobians
) 

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  panda_controllers ${catkin_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

##
add_executable(command_publisher src/command_publisher.cpp)

add_executable(trajectory src/trajectory.cpp) # in case of not object oriented method
add_executable(trajectory_oo src/trajectory_oo.cpp) # in case of object oriented method

add_executable(planner src/planner.cpp)
add_executable(test_jac src/test_jac.cpp)
##

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
##

target_link_libraries(command_publisher ${PROJECT_NAME})

target_link_libraries(trajectory ${PROJECT_NAME}) #In case of not object oriented method
target_link_libraries(trajectory_oo ${PROJECT_NAME}) #In case of object oriented method

target_link_libraries(planner ${PROJECT_NAME})
target_link_libraries(test_jac ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

## Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
