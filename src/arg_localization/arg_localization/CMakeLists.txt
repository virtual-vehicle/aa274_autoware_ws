cmake_minimum_required(VERSION 2.8.3)
project(arg_localization)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vifware_data_tugraz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
   scripts/set_initial_pose.py
   scripts/gps_to_localization_pose.py
   scripts/odom_publisher.py
   scripts/map_base_link_tf_broadcaster.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

