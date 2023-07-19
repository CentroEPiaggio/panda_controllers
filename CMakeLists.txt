cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	controller_interface
	dynamic_reconfigure
	franka_hw
	geometry_msgs
	franka_msgs
	hardware_interface
	message_generation
	pluginlib
	realtime_tools
	roscpp
	rospy
	sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(Boost REQUIRED) # for spline
find_package(CasADi REQUIRED) # for regressor

## to add custom msg
add_message_files(FILES
	desTrajEE.msg
	log_backstepping.msg
)
## to create msg, specify dependecies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${CasADi_INCLUDE_DIRS}
)

find_library(CASADI_LIB casadi)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	controller_interface
	dynamic_reconfigure
	franka_hw
	geometry_msgs
	franka_msgs
	hardware_interface
	message_runtime
	pluginlib
	realtime_tools
	roscpp
	sensor_msgs
	DEPENDS Franka
)

add_library(${PROJECT_NAME}   
	src/computed_torque.cpp #Computed Torque controller
	src/backstepping.cpp #Backstepping controller
	src/cartesian_impedance_controller_softbots.cpp #CartesianImpendanceSoftBots controller
	src/frameLib.cpp #Library of frame
	src/RegBasic.cpp #Library of RegBasic 
	src/SLregressor.cpp #Class of SLregressor
	src/gen_regr_fun.cpp
)

add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	panda_controllers ${catkin_EXPORTED_TARGETS}
)

add_executable(menu src/menu.cpp)
target_link_libraries(menu 
	${catkin_LIBRARIES}
)

add_executable(controller_publisher src/controller_publisher.cpp)
target_link_libraries(controller_publisher 
	${catkin_LIBRARIES} 
	${Boost_LIBRARIES} 
	${CASADI_LIB}
)
add_dependencies(controller_publisher 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
install(TARGETS controller_publisher RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(${PROJECT_NAME} PUBLIC
	${Franka_LIBRARIES}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES} 
	${CASADI_LIB}
)

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
