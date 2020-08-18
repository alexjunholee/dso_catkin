cmake_minimum_required(VERSION 2.4.6)
PROJECT(dso_ros)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
)
set(BUILD_TYPE RelWithDebInfo)
if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

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
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/cmake)
message("---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")

find_package(SuiteParse REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread) 
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3.4.2 REQUIRED)
find_package(libzip CONFIG REQUIRED)
add_definitions(-DHAS_ZIPLIB=1)
add_definitions("-DENABLE_SSE")
set(HAS_PANGOLIN 1)
set(BOOST_THREAD_LIBRARY boost_thread)


# SSE flags
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${DSO_PATH}/thirdparty/sse2neon
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${LIBZIP_INCLUDE_DIR_ZIP}
  ${LIBZIP_INCLUDE_DIR_ZIPCONF}
)  

catkin_package(
  CATKIN_DEPENDS roscpp rospy cv_bridge std_msgs sensor_msgs geometry_msgs
  DEPENDS EIGEN3 Pangolin OpenCV
  INCLUDE_DIRS include
)

add_library(${DSO_LIBRARY})
set_target_properties(dso PROPERTIES LINKER_LANGUAGE CXX)
add_executable(dso_ros src/main.cpp)
target_link_libraries(dso_ros dso boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY}  ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

