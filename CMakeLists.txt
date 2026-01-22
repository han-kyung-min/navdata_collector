cmake_minimum_required(VERSION 3.0.2)
project(navdata_collector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  nav_msgs
  map_msgs
  roscpp
  rospy
  std_msgs
  costmap_2d
  move_base
  geometry_msgs
  message_filters
  sensor_msgs
  tf
  rosbag
  message_generation
  #slam_toolbox
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

set(SLAMTBX_PREFIX  "$ENV{HOME}/catkin_ws/install")
set(SLAMTBX_LIBDIR  "${SLAMTBX_PREFIX}/lib")
set(SLAMTBX_INCDIR  "${SLAMTBX_PREFIX}/include")

add_message_files(
  FILES
  rgbd.msg
  #waypoint_stamped.msg
  navdata_stamped.msg
  navstep_stamped.msg
)


generate_messages(
  DEPENDENCIES  std_msgs geometry_msgs sensor_msgs nav_msgs 
  # actionlib_msgs#      nav_msgs#   #   std_msgs
)


catkin_package(
  INCLUDE_DIRS include ${TBB_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs nav_msgs roscpp rospy geometry_msgs sensor_msgs costmap_2d move_base rosbag message_runtime
  DEPENDS OpenCV  #OpenMP
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${CATKIN_DEVEL_PREFIX}/include
  #${CATKIN_DEVEL_PREFIX}/../install/include
  ${Boost_INCLUDE_DIRS}
  ${SLAMTBX_INCDIR}
)

set(NAVDATA_COLLECTOR_PROJ_DIRECTORY ${PROJECT_SOURCE_DIR} )
file(GLOB_RECURSE SRC_FILES 
${PROJECT_SOURCE_DIR}/src/*.cpp
)

message("pkg: " ${catkin_INCLUDE_DIRS})

# add_library(toolbox_common SHARED IMPORTED GLOBAL)
# set_target_properties(toolbox_common PROPERTIES
#   IMPORTED_LOCATION "${SLAMTBX_LIBDIR}/libtoolbox_common.so"
#   INTERFACE_INCLUDE_DIRECTORIES "${SLAMTBX_INCDIR}"
# )

add_library(navdata_collector SHARED ${SRC_FILES})
target_link_libraries(navdata_collector 
${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
toolbox_common sync_slam_toolbox
#${TBB_LIBRARIES} # ${OpenMP_LIBS}
)

set_target_properties( navdata_collector PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${NAVDATA_COLLECTOR_PROJ_DIRECTORY}/lib 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/navdata_collector_node.cpp)

#add_executable(robot_publisher_node nodes/robot_publisher_node.cpp)
add_executable(navdata_collector_node nodes/navdata_collector_node.cpp)
add_executable(path_generator_node nodes/path_generator_node.cpp)
add_executable(viz_helper_node nodes/viz_helper_node.cpp)
add_executable(dump_posegraph src/dump_posegraph.cpp)


add_dependencies(navdata_collector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(viz_helper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dump_posegraph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(robot_publisher_node navdata_collector)
target_link_libraries(navdata_collector_node navdata_collector)
target_link_libraries(path_generator_node navdata_collector)
target_link_libraries(viz_helper_node navdata_collector)
target_link_libraries(dump_posegraph toolbox_common ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)


#############
## Install ##
#############


catkin_install_python(PROGRAMS  nodes/slam_publisher_node.py 
                                run_script/data_collector/run_manual_data_collection.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS navdata_collector_node 
    path_generator_node viz_helper_node  
    dump_posegraph
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/../
  PATTERN ".git" EXCLUDE)

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  PATTERN ".git" EXCLUDE)
  
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
PATTERN ".git" EXCLUDE)

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
PATTERN ".git" EXCLUDE)

install(DIRECTORY worlds/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
PATTERN ".git" EXCLUDE)
