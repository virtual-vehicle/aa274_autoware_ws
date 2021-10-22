cmake_minimum_required(VERSION 2.8.3)
project(arg_launch)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vifware_launch
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
   scripts/gps_to_ndt_pose_noise.py
   scripts/localization_evaluation.py
   scripts/map_base_link_tf_broadcaster.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

