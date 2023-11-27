cmake_minimum_required(VERSION 3.0.2)
project(tf_cleaner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    visualization_msgs
    roscpp
    tf
    pcl_ros
    pcl_conversions
)

#find_package(PCL 1.2 REQUIRED)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp visualization_msgs
 DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 #${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tf_demo_trans src/tf_demo_trans.cpp
  src/tf_cleaner.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tf_demo_trans ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tf_demo_trans
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS tf_demo_trans
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)