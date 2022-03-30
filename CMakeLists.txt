######################################
## Compiler setting
######################################

cmake_minimum_required( VERSION 2.8 )

# Create Project
project( KinectImageCapture )
message(STATUS ${PROJECT_SOURCE_DIR})

file( GLOB PROJ_SHARED_DATA_SOURCE ${PROJECT_SOURCE_DIR}/src/*.cpp )
file( GLOB PROJ_SHARED_DATA_INCLUDE ${PROJECT_SOURCE_DIR}/include/*.h )
include_directories( ${PROJ_SHARED_DATA_INCLUDE} )
add_executable(project ${PROJ_SHARED_DATA_SOURCE} ${PROJ_SHARED_DATA_INCLUDE})

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Compiler Option
set( FILESYSTEM )
if( "${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" )
  set( FILESYSTEM "stdc++fs" )
elseif( "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" )
  set( FILESYSTEM "c++fs" )
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

######################################
## External libraries
######################################

#set( OpenCV_DIRS "C:/opencv-3.4.8/build" )
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  message("Using OpenCV 2.4.3:  ${OpenCV_LIBS} ")
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
else()
  include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()


# Kinect Azure SDK
find_package( k4a REQUIRED )
find_package( k4arecord REQUIRED )



# Link all libraries to target
target_link_libraries(
	project
	${OpenCV_LIBS}
    ${FILESYSTEM}
)
target_link_libraries( project k4a::k4a )
target_link_libraries( project k4a::k4arecord )


