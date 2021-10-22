cmake_minimum_required(VERSION 2.8.3)
project(vifware_data_tugraz)

find_package(catkin REQUIRED)

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

install(DIRECTORY vector_map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vector_map
)

