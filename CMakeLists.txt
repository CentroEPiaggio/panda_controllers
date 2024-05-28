cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(NLOPT_LIBRARY "/home/tommy/nlopt-2.7.1/build/libnlopt.so")
# SET(NLOPT_LIBRARIES "/home/tommy/nlopt-2.7.1/build/libnlopt.so.0" "/home/tommy/nlopt-2.7.1/build//libnlopt.so.0.10.0")
# set(nlopt_DIR nlopt)

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
find_package(NLopt REQUIRED CONFIG)
find_package(yaml-cpp REQUIRED)

# Stampa il percorso alle directory di inclusione della libreria Nlopt
# message(STATUS "NLopt include directories: ${EIGEN3_INCLUDE_DIRS}")

# Stampa le librerie Nlopt
# message(STATUS "NLopt libraries: ${NLOPT_LIBRARIES}")
# find_package(nlopt REQUIRED)
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
	Vec7D.msg
	udata.msg
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
	${NLOPT_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}  # Include yaml-cpp directories
	#${Boost_INCLUDE_DIRS}
#	${CasADi_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
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

add_library(${PROJECT_NAME}   
	src/computed_torque.cpp #Computed Torque controller
	src/computed_torque_mod.cpp #Computed Torque modified
	src/CT_mod_OS.cpp #Computed Torque modified Operative space
	src/backstepping.cpp #Backstepping controller
	#src/cartesian_impedance_controller_softbots.cpp #CartesianImpendanceSoftBots controller
	# src/gen_regr_fun.cpp
	# src/ThunderPanda.cpp
	src/panda_2_gen.cpp
	src/thunder_panda_2.cpp
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
# 	src/thunder_panda_2.cpp
# 	# src/gen_regr_fun.cpp
# 	src/panda_2_gen.cpp
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

add_executable(com_joints_sin_node
	src/com_joints_sin.cpp
)
target_link_libraries(com_joints_sin_node
	${catkin_LIBRARIES}
)
add_dependencies(com_joints_sin_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

add_executable(command_opt
	src/command_opt.cpp
	src/thunder_panda_2.cpp
	# src/gen_regr_fun.cpp
	src/panda_2_gen.cpp
)
target_link_libraries(command_opt
	${catkin_LIBRARIES}
	${NLOPT_LIBRARIES}
	yaml-cpp
)
add_dependencies(command_opt
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

# add_executable(close_hand_node 
# 	src/close_hand_node.cpp
# )

add_executable(adaptive_throw
 	src/adaptive_throw.cpp
	src/min_jerk.cpp
	src/thunder_panda_2.cpp
	# src/gen_regr_fun.cpp
	src/panda_2_gen.cpp
)

target_link_libraries(adaptive_throw 
	${catkin_LIBRARIES}
	${NLOPT_LIBRARIES}
	yaml-cpp
)

add_dependencies(adaptive_throw
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
	yaml-cpp
	#${Boost_LIBRARIES}
	#${CASADI_LIB}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	${Franka_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	# ${nlopt_INCLUDE_DIRS}
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