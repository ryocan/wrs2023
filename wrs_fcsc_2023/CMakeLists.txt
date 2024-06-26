cmake_minimum_required(VERSION 3.0.2)
project(wrs_fcsc_2023)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  realsense2_camera # realsense kai additional
  tf2_ros
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  rviz_visual_tools
  ur3_moveit_config
  robotiq_2f_gripper_control
  message_generation  # custom msg
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Custom.msg
  sample_message.msg
  real_pub.msg
  pub_flag_ur3.msg
#   Message1.msg
#   Message2.msg
)

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
generate_messages(
  DEPENDENCIES
  sensor_msgs#   
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wrs_fcsc_2023
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wrs_fcsc_2023.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/wrs_fcsc_2023_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wrs_fcsc_2023.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Training for aruco marker
add_executable(aruco src/test_code/arucoMarker/aruco.cpp)
target_link_libraries(aruco  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(aruco_generator src/test_code/arucoMarker/aruco_generator.cpp)
target_link_libraries(aruco_generator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(aruco_3d src/test_code/arucoMarker/aruco_3d.cpp)
target_link_libraries(aruco_3d ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# kai test -> trash
add_executable(ur3_test src/test_code/kai_test/ur3_test.cpp)
target_link_libraries(ur3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(realsense_sub src/test_code/kai_test/realsense_sub.cpp)
target_link_libraries(realsense_sub  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(opencv_cap2 src/test_code/kai_test/opencv_cap2.cpp)
target_link_libraries(opencv_cap2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# for robotiq gripper
add_executable(gripper_pub src/test_code/robotiq_gripper/gripper_pub.cpp)
target_link_libraries(gripper_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# demo for champion data
add_executable(demo_img_proc src/test_code/demo/demo_img_proc.cpp)
target_link_libraries(demo_img_proc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)
add_executable(demo_arm_gripper src/test_code/demo/demo_arm_gripper.cpp)
target_link_libraries(demo_arm_gripper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# subscribe from realsense and publish 3dpoint
add_executable(pixelToPoint src/test_code/pixelToPoint.cpp)
target_link_libraries(pixelToPoint ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)
# add_executable(ur_moveit_command_controller src/test_code/ur_moveit_command_controller.cpp)
# target_link_libraries(ur_moveit_command_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# calc pca 
add_executable(pca src/test_code/PCA/pca.cpp)
target_link_libraries(pca ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# hsv
add_executable(hsv src/test_code/hsvCheck.cpp)
target_link_libraries(hsv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# realsense image save
add_executable(realsense_image_saver src/test_code/realsense_image_saver.cpp)
target_link_libraries(realsense_image_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

# command_test
add_executable(robotiq_usb_command src/robotiq_usb_command.cpp)
target_link_libraries(robotiq_usb_command ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)


# python
install(PROGRAMS scripts/Robotiq2FGripperRtuNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# -------------------------------------------------------------------------------------------------------- #
add_executable(ur3_drawShelf src/ur3_drawShelf.cpp)
target_link_libraries(ur3_drawShelf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

add_executable(ur3_disposal src/ur3_disposal.cpp)
target_link_libraries(ur3_disposal ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

add_executable(ur3_stock src/ur3_stock.cpp)
target_link_libraries(ur3_stock ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)

add_executable(systemManager src/systemManager.cpp)
target_link_libraries(systemManager ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} realsense2)
