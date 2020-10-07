#####################################
# Enable C++11
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS "-lstdc++fs")
SET(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
SET(CMAKE_HAVE_THREADS_LIBRARY 1)
SET(CMAKE_USE_WIN32_THREADS_INIT 0)
SET(CMAKE_USE_PTHREADS_INIT 1)
SET(THREADS_PREFER_PTHREAD_FLAG ON)
cmake_minimum_required (VERSION 2.8) 
project (raspicam_test)

#####################################
# Setup package for AWS
set(BUILD_SHARED_LIBS ON)
find_package(AWSSDK REQUIRED COMPONENTS dynamodb)

find_package(raspicam REQUIRED)
find_package(OpenCV)
IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
MESSAGE(STATUS "COMPILING OPENCV TESTS")
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable (frameratetest main.cpp)  
MESSAGE( STATUS  "LIBRARIES ${raspicam_CV_LIBS}")
target_link_libraries (frameratetest ${OpenCV_LIBS} ${raspicam_CV_LIBS})
target_link_libraries (frameratetest -lstdc++fs)
target_link_libraries (frameratetest ${AWSSDK_LINK_LIBRARIES})
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(frameratetest ${Boost_LIBRARIES})
ELSE()
MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#####################################