cmake_minimum_required(VERSION 2.8.3)
project(sphero_rvr_hw)

add_compile_options(-std=c++17 -fconcepts)

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts
  roslib
  geometry_msgs
  rospy
  message_generation
  serial
  tf
  )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime serial
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_ROOT}/rvr++/include
  ${catkin_INCLUDE_DIRS}
)

set(RVRSOURCES
  ${PROJECT_ROOT}/rvr++/src/SerialPort.cpp
  ${PROJECT_ROOT}/rvr++/src/SensorsStream.cpp
  ${PROJECT_ROOT}/rvr++/src/Response.cpp
  ${PROJECT_ROOT}/rvr++/src/Request.cpp
  ${PROJECT_ROOT}/rvr++/src/ApiAndConn.cpp
  ${PROJECT_ROOT}/rvr++/src/Drive.cpp
  ${PROJECT_ROOT}/rvr++/src/Blackboard.cpp
  ${PROJECT_ROOT}/rvr++/src/SensorsDirect.cpp
  ${PROJECT_ROOT}/rvr++/src/SendPacket.cpp
  ${PROJECT_ROOT}/rvr++/src/SystemInfo.cpp
  ${PROJECT_ROOT}/rvr++/src/Power.cpp
  ${PROJECT_ROOT}/rvr++/src/ReadPacket.cpp
  ${PROJECT_ROOT}/rvr++/src/IoLed.cpp
  )


# Hardware Interface
add_library(rvr++ ${RVRSOURCES})

add_library(sphero_rvr_hw_interface
  ${PROJECT_ROOT}/src/sphero_rvr_hw_interface.cpp
  )

add_executable(sphero_rvr_hw_node ${PROJECT_ROOT}/src/sphero_rvr_hw_node.cpp ${PROJECT_ROOT}/src/sphero_rvr_hw_interface.cpp ${PROJECT_ROOT}/src/sphero_rvr_hw_interface.h)
target_link_libraries(sphero_rvr_hw_node sphero_rvr_hw_interface ${catkin_LIBRARIES} rvr++)

add_executable(base_controller ${PROJECT_ROOT}/src/base_controller.cpp)
target_link_libraries(base_controller rvr++ ${catkin_LIBRARIES})

catkin_install_python(
        PROGRAMS
            scripts/rvr-ros.py
            scripts/rvr-ros-restarter.py
        DESTINATION
            ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


#############
## Install ##
#############
install(TARGETS
  sphero_rvr_hw_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

# Install executables
install(TARGETS
  sphero_rvr_hw_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Install rospy nodes
catkin_install_python(PROGRAMS
  scripts/rvr-ros.py
  scripts/rvr-ros-sim.py
  scripts/emergency_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Install python modules
catkin_python_setup()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rvr_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
