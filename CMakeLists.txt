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
# Setup headers for files
SET(HEADER_FILES Socket.cpp Socket.h MPU6050.cpp MPU6050.h)

#####################################
# Setup package for AWS
set(BUILD_SHARED_LIBS ON)
find_package(AWSSDK REQUIRED COMPONENTS dynamodb)

FIND_LIBRARY(wiringPi_LIB wiringPi)
find_package(raspicam REQUIRED)
find_package(OpenCV)
find_package(yaml-cpp REQUIRED)

IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
MESSAGE(STATUS "COMPILING OPENCV TESTS")
include_directories( ${OpenCV_INCLUDE_DIRS} )


FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

add_executable (frameratetest frameratetest.cpp) 
add_executable (server server.cpp ${HEADER_FILES}) 
add_executable (client client.cpp ${HEADER_FILES}) 
MESSAGE( STATUS  "LIBRARIES ${raspicam_CV_LIBS}")



message(STATUS "Used yaml dir: ${YAML_CPP_LIBRARIES}")

target_link_libraries (frameratetest ${OpenCV_LIBS} ${raspicam_CV_LIBS})
target_link_libraries (frameratetest -lstdc++fs)
target_link_libraries (frameratetest ${AWSSDK_LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(frameratetest ${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES(client -lstdc++fs yaml-cpp ${Boost_LIBRARIES} ${wiringPi_LIB})
TARGET_LINK_LIBRARIES(server -lstdc++fs yaml-cpp ${Boost_LIBRARIES} ${wiringPi_LIB})

ELSE()
MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#####################################