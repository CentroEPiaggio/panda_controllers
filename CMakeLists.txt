cmake_minimum_required(VERSION 3.10.2)
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
    tf2_ros
    std_msgs
    sensor_msgs
)

# --- 1. NUOVO: GENERAZIONE MESSAGGI CUSTOM ---
# Questo cerca il file .msg nella cartella msg/ del tuo pacchetto
add_message_files(
    FILES
    MpcSolution.msg
)

# Questo genera effettivamente gli header C++ dipendenti da std_msgs e sensor_msgs
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.2 REQUIRED) 
find_package(yaml-cpp REQUIRED)

# --- 2. CONFIGURAZIONE MANUALE CASADI ---
set(CASADI_ROOT_DIR "/home/frankino/casadi")
find_library(CASADI_LIBRARIES casadi PATHS ${CASADI_ROOT_DIR} NO_DEFAULT_PATH)

if(NOT CASADI_LIBRARIES)
    message(FATAL_ERROR "Libreria libcasadi.so non trovata in: ${CASADI_ROOT_DIR}")
endif()

# --- 3. ACADOS SETUP ---
set(ACADOS_INSTALL_DIR "/home/frankino/acados") 
set(ACADOS_GENERATED_DIR "${PROJECT_SOURCE_DIR}/../../c_generated_code_tracking")

# --- 4. TROVA I SORGENTI ACADOS  ---
file(GLOB_RECURSE ACADOS_SOLVER_SRCS "${ACADOS_GENERATED_DIR}/*.c")
list(FILTER ACADOS_SOLVER_SRCS EXCLUDE REGEX "/main_.*\\.c$")

# --- 5. Include Directories Globali ---
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}   
    /usr/include/eigen3      
    ${CASADI_ROOT_DIR}          
    ${CASADI_ROOT_DIR}/include  
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include
    ${ACADOS_INSTALL_DIR}/include/hpipm/include
    ${ACADOS_GENERATED_DIR}
	${PROJECT_SOURCE_DIR}/scripts
    ${PROJECT_SOURCE_DIR}/frankino_generatedFiles
)

# --- 6. Link Directories ---
link_directories(
    ${ACADOS_INSTALL_DIR}/lib
    ${CASADI_ROOT_DIR}          
)

# --- 7. Catkin Package ---
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    franka_hw
    franka_msgs
    hardware_interface
    roscpp
    message_runtime
)

# --- 8. Libreria del Controller (panda_controllers) ---

file(GLOB FRANKINO_GEN_SRCS 
    "${PROJECT_SOURCE_DIR}/frankino_generatedFiles/*.cpp"
    "${PROJECT_SOURCE_DIR}/frankino_generatedFiles/*.c"
)

add_library(${PROJECT_NAME}   
    src/computed_torque.cpp
    src/cartesian_impedance_controller_softbots.cpp
    ${FRANKINO_GEN_SRCS}  
    ${ACADOS_SOLVER_SRCS} # Ora questa variabile è piena grazie al punto 4
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${Franka_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CASADI_LIBRARIES} # Link diretto al file trovato
    yaml-cpp
    acados
    hpipm
    blasfeo
    m
)

# --- 9. Eseguibile Standalone (menu_acados) ---
add_executable(menu_acados src/menu_acados.cpp)

# Linkando ${PROJECT_NAME}, eredita tutto (Casadi, Acados, ecc.)
target_link_libraries(menu_acados 
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(mpc_integrator_node src/mpc_integrator_node.cpp)
target_link_libraries(mpc_integrator_node 
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${PROJECT_NAME}
)
add_dependencies(mpc_integrator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- 10. Installation ---
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)