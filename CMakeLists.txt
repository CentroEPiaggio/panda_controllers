cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
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
	visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
#find_package(Boost REQUIRED)

#find_package(CasADi REQUIRED) # for regressor
#find_library(CASADI_LIB casadi)

## to add custom msg
add_message_files(FILES
	desTrajEE.msg
	log_adaptive_cartesian.msg
	log_adaptive_joints.msg
	point.msg
	link_params.msg
	flag.msg
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
	#${Boost_INCLUDE_DIRS}
#	${CasADi_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/include
)

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

# - all in one - #
# file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
# # Definisci la tua libreria
# add_library(library ${SOURCES})
# # Specifica il nome dell'output della libreria
# set_target_properties(library PROPERTIES OUTPUT_NAME "library")
# # Indica che altri progetti possono trovare questa libreria
# target_include_directories(library PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}   
	# src/computed_torque.cpp #Computed Torque controller
	src/backstepping.cpp #Backstepping controller
	#src/cartesian_impedance_controller_softbots.cpp #CartesianImpendanceSoftBots controller
	src/franka_gen.cpp
	src/thunder_franka.cpp
	src/gen_traj_fun.cpp
)

add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	panda_controllers ${catkin_EXPORTED_TARGETS}
)

add_executable(command_cartesian
	src/command_cartesian.cpp
	src/gen_traj_fun.cpp
)
target_link_libraries(command_cartesian
	${catkin_LIBRARIES}
)
add_dependencies(command_cartesian
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
# add_executable(CLIK_node
# 	src/CLIK_node.cpp
# 	src/ThunderPanda.cpp
# 	src/gen_regr_fun.cpp
# )
# target_link_libraries(CLIK_node
# 	${catkin_LIBRARIES}
# )
# add_dependencies(CLIK_node
# 	${${PROJECT_NAME}_EXPORTED_TARGETS} 
# 	${catkin_EXPORTED_TARGETS}
# )
add_executable(adaptiveFlag_node
	src/adaptiveFlag_node.cpp
)
target_link_libraries(adaptiveFlag_node
	${catkin_LIBRARIES}
)
add_dependencies(adaptiveFlag_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

add_executable(command_joints_node
	src/command_joints.cpp
)
target_link_libraries(command_joints_node
	${catkin_LIBRARIES}
)
add_dependencies(command_joints_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
add_executable(trajectory_rviz
	src/trajectory_rviz.cpp
)
target_link_libraries(trajectory_rviz
	${catkin_LIBRARIES}	
)
add_dependencies(trajectory_rviz
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
	${Franka_LIBRARIES}
	${catkin_LIBRARIES}
	#${Boost_LIBRARIES}
	#${CASADI_LIB}
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