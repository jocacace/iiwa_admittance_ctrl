cmake_minimum_required(VERSION 2.8.3)
project(iiwa_admittance_ctrl)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  iiwa_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  roslib
	message_generation
	ctrl_msgs
)

## Generate messages in the 'msg' folder
#add_message_files(
#	FILES
#	desired_command.msg
#	classification_result.msg	
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#	 #std_msgs
#   geometry_msgs#   iiwa_msgs#   sensor_msgs#   std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_admittance_ctrl
#  CATKIN_DEPENDS geometry_msgs iiwa_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#---Admittance control kuka IIWA
add_executable(iiwa_admittance_ctrl src/admittance_ctrl.cpp )
target_link_libraries(iiwa_admittance_ctrl ${catkin_LIBRARIES} iiwa_kinematic lapack iiwa_kinematic)
#---

#add_executable(force_from_joy src/force_client/force_from_joy.cpp)
#target_link_libraries(force_from_joy ${catkin_LIBRARIES}  )

#add_executable(admittance_stiffness src/variable_stiffness/stiffness_calculation.cpp )
#target_link_libraries(admittance_stiffness ${catkin_LIBRARIES} )

#add_executable(training_data src/training/training_data.cpp ${libsvm_dir}/svm.cpp)
#target_link_libraries(training_data ${catkin_LIBRARIES} )

#include_directories( ${OpenCV_INCLUDE_DIRS} )

#find_package( OpenCV REQUIRED )

#add_executable(nn_classification src/training/classification.cpp )
#target_link_libraries(nn_classification ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(cooperative_validation src/validation_data/validation.cpp )
#target_link_libraries(cooperative_validation ${catkin_LIBRARIES})
