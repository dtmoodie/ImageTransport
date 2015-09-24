cmake_minimum_required(VERSION 2.8.3)
project(image_transport_tutorial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    dynamic_reconfigure
)
generate_dynamic_reconfigure_options(cfg/MyStuff.cfg)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED core highgui)
FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options log log_setup)
set(CMAKE_VERBOSE_MAKEFILE ON)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE(Gstreamer)
FIND_PACKAGE(PkgConfig)
pkg_search_module(GLIB2 glib-2.0)
IF(GLIB2_FOUND)
  INCLUDE_DIRECTORIES(${GSTREAMER_gst_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${GSTREAMER_gstconfig_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${Glib_INCLUDE_DIR})
  INCLUDE_DIRECTORIES("/usr/lib/x86_64-linux-gnu/glib-2.0/include")
  INCLUDE_DIRECTORIES("/usr/include/glib-2.0")
ENDIF(GLIB2_FOUND)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_transport_tutorial
#  CATKIN_DEPENDS cv_bridge image_transport
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
  ${CMAKE_CURRENT_SOURCE_DIR}
)
message(${CMAKE_CURRENT_SOURCE_DIR})

## Declare a cpp library
# add_library(image_transport_tutorial
#   src/${PROJECT_NAME}/image_transport_tutorial.cpp
# )

## Declare a cpp executable

add_executable(my_publisher src/my_publisher.cpp)
add_dependencies(my_publisher ${PROJECT_NAME}_gencfg)
include_directories("${CMAKE_SOURCE_DIR}/image_transport_tutorial/cfg/cpp/image_transport_tutorial")
message("Dir: ${CMAKE_SOURCE_DIR}/image_transport_tutorial/cfg/cpp/image_transport_tutorial")
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(image_transport_tutorial_node image_transport_tutorial_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(my_publisher
  ${catkin_LIBRARIES}
  opencv_highgui
  opencv_core
  ${GSTREAMER_gstapp_LIBRARY}
  ${GSTREAMER_gstaudio_LIBRARY}
  ${GSTREAMER_gstbase_LIBRARY}
  ${GSTREAMER_gstcontroller_LIBRARY}
  ${GSTREAMER_gstpbutils_LIBRARY}
  ${GSTREAMER_gstreamer_LIBRARY}
  ${GSTREAMER_gstriff_LIBRARY}
  ${GSTREAMER_gstrtp_LIBRARY}
  ${GSTREAMER_gstrtsp_LIBRARY}
  ${GSTREAMER_gstsdp_LIBRARY}
  ${GSTREAMER_gsttag_LIBRARY}
  ${GSTREAMER_gstvideo_LIBRARY}
  ${Glib_LIBRARY}
  ${GOBJECT_LIBRARY}
  /usr/lib/x86_64-linux-gnu/libgobject-2.0.so
  /usr/lib/x86_64-linux-gnu/libglib-2.0.so
  ${Boost_LIBRARIES}

 )

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
# install(TARGETS image_transport_tutorial image_transport_tutorial_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_transport_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
