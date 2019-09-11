cmake_minimum_required(VERSION 2.8.3)
project(denso_cobotta_gripper)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_limits_interface
  message_generation
  roscpp
  std_msgs
  transmission_interface
  actionlib
  actionlib_msgs
  denso_cobotta_lib
  denso_cobotta_driver
)
 add_action_files(
   FILES
   GripperMove.action
   VacuumMove.action
 )

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES denso_cobotta_gripper
  CATKIN_DEPENDS controller_manager hardware_interface joint_limits_interface message_runtime roscpp std_msgs transmission_interface actionlib actionlib_msgs denso_cobotta_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/denso_cobotta_gripper.cpp
  src/gripper_base.cpp
  src/gripper_parallel.cpp
  src/gripper_vacuum.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

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

install(FILES
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
