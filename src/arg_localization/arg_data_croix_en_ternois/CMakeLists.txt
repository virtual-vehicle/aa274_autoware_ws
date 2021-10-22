cmake_minimum_required(VERSION 2.8.3)
project(arg_data_croix_en_ternois)

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

install(DIRECTORY pcd/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pcd
)

install(DIRECTORY bagfile/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bagfile
)

