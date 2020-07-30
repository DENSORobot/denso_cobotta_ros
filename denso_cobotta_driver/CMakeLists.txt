cmake_minimum_required(VERSION 2.8.3)
project(denso_cobotta_driver)

add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  realtime_tools
  std_msgs
  denso_cobotta_lib
)

add_service_files(
   FILES
   SetMotorState.srv
   GetMotorState.srv
   ClearError.srv
   ClearRobotError.srv
   ClearSafeState.srv
   SetBrakeState.srv
   GetBrakeState.srv
   SetLEDState.srv
   ExecCalset.srv
)

add_message_files(
   FILES
   RobotState.msg
   SafeState.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  LIBRARIES denso_cobotta_lib  
)

###########
## Build ##
###########

include_directories(
  include
# include/${PROJECT_NAME}/cobotta
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/denso_cobotta_driver.cpp
  )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
