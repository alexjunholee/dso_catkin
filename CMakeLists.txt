cmake_minimum_required(VERSION 2.4.6)
PROJECT(dso_ros)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	cv_bridge

	std_msgs
	sensor_msgs
	geometry_msgs
)

set(DSO_PATH /home/jhlee/git/dso)
set(DSO_LIBRARY dso ${DSO_PATH}/build/lib/libdso.a)
message("---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")

find_package(Boost REQUIRED COMPONENTS system thread) 
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3.4.2 REQUIRED)

# SSE flags
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${catkin_INCLUDE_DIRS}
  ${DSO_PATH}/thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)  

catkin_package(
  CATKIN_DEPENDS roscpp rospy cv_bridge std_msgs sensor_msgs geometry_msgs
  DEPENDS EIGEN3 Pangolin OpenCV Boost
  INCLUDE_DIRS include
)

add_executable(dso_live src/main.cpp)
add_library(${DSO_LIBRARY})
set_target_properties(dso PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(dso_live ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

