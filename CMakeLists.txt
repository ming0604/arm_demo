cmake_minimum_required(VERSION 3.0.2)
project(demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  Eigen3
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tm_msgs
)



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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs#   tm_msgs
# )

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
#  LIBRARIES demo
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tm_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/demo_node.cpp)
add_executable(demo_send_script   src/demo_send_script.cpp)
add_executable(demo_ask_sta       src/demo_ask_sta.cpp)
add_executable(demo_connect_tm    src/demo_connect_tm.cpp)
add_executable(demo_set_event     src/demo_set_event.cpp)
add_executable(demo_set_io        src/demo_set_io.cpp)
add_executable(demo_set_positions src/demo_set_positions.cpp)
add_executable(demo_write_item    src/demo_write_item.cpp)
add_executable(demo_ask_item      src/demo_ask_item.cpp)
add_executable(demo_get_feedback      get_status_demo_src/demo_get_feedback.cpp)
add_executable(demo_get_sct_response  get_status_demo_src/demo_get_sct_response.cpp)
add_executable(demo_get_sta_response  get_status_demo_src/demo_get_sta_response.cpp)
add_executable(demo_get_svr_response  get_status_demo_src/demo_get_svr_response.cpp)
add_executable(demo_leave_listen_node src/demo_leave_listen_node.cpp)
add_executable(demo_send_script_speed_set src/demo_send_script_speed_set.cpp)
add_executable(demo_send_script_speed_start src/demo_send_script_speed_start.cpp)
add_executable(demo_send_script_speed_stop src/demo_send_script_speed_stop.cpp)
add_executable(demo_send_script_speed_circle src/demo_send_script_speed_circle.cpp)
add_executable(save_state get_status_demo_src/save_state.cpp)
add_executable(demo_send_script_speed_sin src/demo_send_script_speed_sin.cpp)
add_executable(demo_send_script_line_sin src/demo_send_script_line_sin.cpp)
add_executable(demo_send_script_speed_pid_1 src/demo_send_script_speed_pid_1.cpp)
add_executable(demo_send_script_speed_sin_position src/demo_send_script_speed_sin_position.cpp)
add_executable(demo_send_script_speed_sin_feedback src/demo_send_script_speed_sin_feedback.cpp)
add_executable(demo_keyboard_input_pid src/demo_keyboard_input_pid.cpp)

add_executable(demo_send_script_speed_pid_GUI_multi src/demo_send_script_speed_pid_GUI_multi.cpp)
add_executable(demo_send_script_speed_pid_GUI_multi_file src/demo_send_script_speed_pid_GUI_multi_file.cpp)

add_executable(demo_send_script_speed_pid_GUI_multi_file_move_to_init src/demo_send_script_speed_pid_GUI_multi_file_move_to_init.cpp)
add_executable(demo_send_script_speed_pid_GUI_multi_file_zhun src/demo_send_script_speed_pid_GUI_multi_file_zhun.cpp)
add_executable(demo_send_script_speed_pid_GUI_multi_file_move_to_end src/demo_send_script_speed_pid_GUI_multi_file_move_to_end.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_ask_sta ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_connect_tm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_set_event ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_set_io ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_set_positions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_write_item ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_ask_item ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_get_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_get_sct_response ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_get_sta_response ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_get_svr_response ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_leave_listen_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_start ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_stop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_circle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(save_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_sin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_line_sin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_pid_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_sin_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_sin_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_keyboard_input_pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_pid_GUI_multi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_pid_GUI_multi_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(demo_send_script_speed_pid_GUI_multi_file_move_to_init ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_pid_GUI_multi_file_zhun ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_send_script_speed_pid_GUI_multi_file_move_to_end ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
 target_link_libraries(demo_send_script
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_ask_sta
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_connect_tm
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_set_event
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_set_io
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_set_positions
   ${catkin_LIBRARIES}
 ) 
 target_link_libraries(demo_write_item
   ${catkin_LIBRARIES}
 ) 
 target_link_libraries(demo_ask_item
   ${catkin_LIBRARIES}
 ) 
 target_link_libraries(demo_get_feedback
   ${catkin_LIBRARIES}
 )  
 target_link_libraries(demo_get_sct_response
   ${catkin_LIBRARIES}
 ) 
 target_link_libraries(demo_get_sta_response
   ${catkin_LIBRARIES}
 )
 target_link_libraries(demo_get_svr_response
   ${catkin_LIBRARIES}
 )  
 target_link_libraries(demo_leave_listen_node
   ${catkin_LIBRARIES}
 ) 

 target_link_libraries(demo_send_script_speed_set
 ${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_start
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_stop
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_circle
${catkin_LIBRARIES}
) 
target_link_libraries(save_state
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_sin
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_pid_1
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_line_sin
${catkin_LIBRARIES}
Eigen3::Eigen
) 
target_link_libraries(demo_send_script_speed_sin_position
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_sin_feedback
${catkin_LIBRARIES}
) 
target_link_libraries(demo_keyboard_input_pid
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_pid_GUI_multi
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_pid_GUI_multi_file
${catkin_LIBRARIES}
) 


target_link_libraries(demo_send_script_speed_pid_GUI_multi_file_move_to_init
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_pid_GUI_multi_file_zhun
${catkin_LIBRARIES}
) 
target_link_libraries(demo_send_script_speed_pid_GUI_multi_file_move_to_end
${catkin_LIBRARIES}
) 
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
