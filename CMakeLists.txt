cmake_minimum_required(VERSION 2.8.3)
project(rocup)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Werror=return-type")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# KDL
#############################################################################
find_package(orocos_kdl REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_generation
   tf
   tf_conversions
   kdl_parser
   kdl_conversions
   visualization_msgs
   genmsg
   actionlib_msgs
   actionlib   
 )


catkin_python_setup()

########################
########################
########################
## External Libraries ##
########################
########################
########################

include_directories(${catkin_INCLUDE_DIRS})

#############################################
## Eigen
#############################################
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#############################################
## Build Vision Modules?
#############################################

##############
## Actions ##
##############

add_action_files(
  DIRECTORY action FILES 
  RobotMotion.action
  TextCommand.action
)

##############
## Messages ##
##############

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UiEvent.msg
  UiFeedback.msg 
  RobotFollow.msg
  RobotFollowStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   #
   IKService.srv
   SFMCommand.srv 
   #
   RobotFollowSetting.srv 
   StringSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   actionlib_msgs
)


catkin_package(
  #CATKIN_DEPENDS
  DEPENDS orocos_kdl
  #INCLUDE_DIRS include
  #LIBRARIES comau_ros
)

###############
## Libraries ##
###############

## Comau Library
include_directories(include include/bonmet_robot/ikine6s_full)
add_library(comau_robot
  src/comau_robot/ComauSmartSix.cpp
)
target_link_libraries(comau_robot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Bonmet Library
add_library(ikine6s_full 
  src/bonmet_robot/ikine6s_full/ikine6s_full.cpp
  src/bonmet_robot/ikine6s_full/ikine6s_full_initialize.cpp
  src/bonmet_robot/ikine6s_full/acos.cpp
  src/bonmet_robot/ikine6s_full/eye.cpp
  src/bonmet_robot/ikine6s_full/norm.cpp
  src/bonmet_robot/ikine6s_full/rotx.cpp
  src/bonmet_robot/ikine6s_full/rotz.cpp
  src/bonmet_robot/ikine6s_full/rt_nonfinite.cpp
  src/bonmet_robot/ikine6s_full/rtGetInf.cpp
  src/bonmet_robot/ikine6s_full/rtGetNaN.cpp
  src/bonmet_robot/ikine6s_full/sqrt.cpp
)

add_library(bonmet_robot
  src/bonmet_robot/bonmet_c60.cpp 
)
target_link_libraries(bonmet_robot ikine6s_full ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
###########
## Nodes ##
###########

## Comau IK service
add_executable(comau_ik_service src/nodes/comau/comau_ik_service.cpp)
target_link_libraries(comau_ik_service ${catkin_LIBRARIES} comau_robot)
add_dependencies(comau_ik_service ${PROJECT_NAME}_gencpp)

## Comau Target Follow
add_executable(comau_target_follower src/nodes/comau/comau_target_follower.cpp)
target_link_libraries(comau_target_follower ${catkin_LIBRARIES} comau_robot)
add_dependencies(comau_target_follower ${PROJECT_NAME}_gencpp)


## Schunk IK service
add_executable(schunk_ik_service src/nodes/schunk/schunk_ik_service.cpp)
target_link_libraries(schunk_ik_service ${catkin_LIBRARIES})
add_dependencies(schunk_ik_service ${PROJECT_NAME}_gencpp)

## Schunk Target Follow
add_executable(schunk_target_follower src/nodes/schunk/schunk_target_follower.cpp)
target_link_libraries(schunk_target_follower ${catkin_LIBRARIES})
add_dependencies(schunk_target_follower ${PROJECT_NAME}_gencpp)

## Grasshopper IK service
add_executable(grasshopper_ik_service src/nodes/grasshopper/grasshopper_ik_service.cpp)
target_link_libraries(grasshopper_ik_service ${catkin_LIBRARIES})
add_dependencies(grasshopper_ik_service ${PROJECT_NAME}_gencpp)

## Grasshopper Target Follow
add_executable(grasshopper_target_follower src/nodes/grasshopper/grasshopper_target_follower.cpp)
target_link_libraries(grasshopper_target_follower ${catkin_LIBRARIES})
add_dependencies(grasshopper_target_follower ${PROJECT_NAME}_gencpp)

## Bonmet IK service
add_executable(bonmet_ik_service src/nodes/bonmet/bonmet_ik_service.cpp)
target_link_libraries(bonmet_ik_service ${catkin_LIBRARIES} bonmet_robot)
add_dependencies(bonmet_ik_service ${PROJECT_NAME}_gencpp)

## Bonmet Target Follow
add_executable(bonmet_target_follower src/nodes/bonmet/bonmet_target_follower.cpp)
target_link_libraries(bonmet_target_follower ${catkin_LIBRARIES} bonmet_robot)
add_dependencies(bonmet_target_follower ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_comau_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
