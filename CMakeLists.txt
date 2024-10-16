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
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  Commands.msg
  CommandParams.msg
)

generate_messages(
 	DEPENDENCIES
 	sensor_msgs
 )


# generate_messages(
# 	DEPENDENCIES
# 	# franka_msgs
# 	std_msgs
# 	# geometry_msgs
# )

# include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
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
	DEPENDS Franka
)



add_library(${PROJECT_NAME}   
	src/computed_torque.cpp #Computed Torque controller
)

# Specifica i file sorgente dipendenti per ilc_controller_wrist.cpp
target_sources(${PROJECT_NAME}
    PRIVATE
    src/thunder_frankawrist.cpp
    src/frankawrist_gen.cpp
)

# Aggiungi le librerie necessarie per il linking (aggiorna con le tue librerie specifiche)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
)

# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   panda_controllers ${catkin_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	panda_controllers ${catkin_EXPORTED_TARGETS}
)

add_executable(menu src/menu.cpp)

target_link_libraries(menu ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PUBLIC
	${Franka_LIBRARIES}
	${catkin_LIBRARIES}
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
# install(DIRECTORY data
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(FILES ${PROJECT_NAME}_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
