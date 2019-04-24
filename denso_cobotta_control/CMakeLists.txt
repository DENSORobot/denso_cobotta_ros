cmake_minimum_required(VERSION 2.8.3)
project(denso_cobotta_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  std_msgs
  transmission_interface
  denso_cobotta_lib
  denso_cobotta_driver
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES denso_cobotta_control
  CATKIN_DEPENDS controller_manager hardware_interface joint_limits_interface roscpp std_msgs transmission_interface denso_cobotta_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/denso_cobotta_control.cpp
  src/denso_cobotta_hw.cpp
)

add_dependencies(${PROJECT_NAME} denso_cobotta_driver_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
