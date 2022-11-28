cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++14)
project(lidar_localizer)

find_package(PCL REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DOPENMP_FOUND)
endif ()



find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()


find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_filters
        tf
        tf2
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs
        std_msgs
        nav_msgs
        ndt_msgs
        sensor_msgs
        pcl_conversions
        #velodyne_pointcloud
        pcl_ros
        pcl_conversions
        ndt_cpu
        autoware_lidar_localizer
        ${PCL_OPENMP_PACKAGES}
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS std_msgs
                       tf2_ros
                       tf2_sensor_msgs
                       tf2_geometry_msgs
                       #velodyne_pointcloud
                       ndt_cpu
                       ndt_msgs
                       autoware_lidar_localizer
                       ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

###########
## Build ##
###########

if($ENV{AVL_VEHICLE})
  add_definitions( -D_AVL=\"true\" )
  message("AVL vehicle")
endif()

include_directories(include ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_matching PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")

endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))


add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(queue_counter ${catkin_EXPORTED_TARGETS})


install(TARGETS
            ndt_matching
            queue_counter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
