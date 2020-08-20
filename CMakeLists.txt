cmake_minimum_required(VERSION 2.8)
project(GLSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV 3.4.0 REQUIRED)#QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")

find_package(Pangolin REQUIRED)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_definitions(${PCL_DEFINITIONS})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/ThirdParty
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(GLSLAM SHARED
    ThirdParty/pclomp/gicp_omp.cpp
    ThirdParty/pclomp/ndt_omp.cpp
    ThirdParty/pclomp/voxel_grid_covariance_omp.cpp

    src/SimpleIcpSLAM.cc
    src/LidarUtils.cc
    src/PointCloudProjector.cc
    src/GSegmentation.cc
)

target_link_libraries(GLSLAM 
  ${PCL_LIBRARY_DIRS}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Build examples

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

add_executable(test
  example/main.cpp)
target_link_libraries(test 
  GLSLAM
  ${Pangolin_LIBRARIES}
)

add_executable(segment
  example/segmentation_test.cpp)
target_link_libraries(segment 
  GLSLAM
  ${Pangolin_LIBRARIES}
)

add_executable(project_lidar
  example/project_lidar.cpp)
target_link_libraries(project_lidar 
  GLSLAM
)

