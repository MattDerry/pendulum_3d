cmake_minimum_required(VERSION 2.8.3)
project(pendulum_3d)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy std_msgs geometry_msgs sensor_msgs tf std_srvs message_generation dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Pendulum.cfg
)

add_message_files(FILES
  Score.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

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
   INCLUDE_DIRS include
   #LIBRARIES pendulum_3d_demo
   CATKIN_DEPENDS roscpp roslib rospy std_msgs geometry_msgs sensor_msgs tf std_srvs
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(keyboard_interface src/keyboard_interface.cpp)
add_dependencies(keyboard_interface pendulum_3d_gencfg)
target_link_libraries(keyboard_interface
  ${catkin_LIBRARIES}
)
