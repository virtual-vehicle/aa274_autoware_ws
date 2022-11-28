if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
endif()

# Enable support for C++11
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

message(STATUS "CUDA compilation status: $ENV{AUTOWARE_COMPILE_WITH_CUDA}.")

macro(AW_CHECK_CUDA)
  if ($ENV{AUTOWARE_COMPILE_WITH_CUDA})
    find_package(CUDA REQUIRED)
    find_package(Eigen3 REQUIRED)

    if(NOT ${CUDA_VERSION} VERSION_LESS "10.0"
            AND NOT ${CUDA_VERSION} VERSION_EQUAL "10.0" )
      message(FATAL_ERROR "GPU support on Melodic requires CUDA<=10.0")
    endif()
    if(${CUDA_VERSION} VERSION_GREATER "9.1"
          AND ${CMAKE_VERSION} VERSION_LESS "3.12.3")
      unset(CUDA_cublas_device_LIBRARY CACHE)
      set(CUDA_cublas_device_LIBRARY ${CUDA_cublas_LIBRARY})
      set(CUDA_CUBLAS_LIBRARIES ${CUDA_cublas_LIBRARY})
    endif()
    if ("$ENV{ROS_DISTRO}" STREQUAL "melodic" AND ${EIGEN3_VERSION_STRING} VERSION_LESS "3.3.7")
      message(FATAL_ERROR "GPU support on Melodic requires Eigen version>= 3.3.7")
    endif()
    set(USE_CUDA ON)
  else()
    message(WARNING "CUDA support is disabled. Set the AUTOWARE_COMPILE_WITH_CUDA environment variable and recompile to enable it")
    set(USE_CUDA OFF)
  endif()
endmacro()