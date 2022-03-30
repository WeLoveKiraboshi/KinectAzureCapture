cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 17 ) # require C++17 (or later) for filesystem
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Compiler Option
set( FILESYSTEM )
if( "${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" )
  set( FILESYSTEM "stdc++fs" )
elseif( "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" )
  set( FILESYSTEM "c++fs" )
endif()

# Project
project( recorder LANGUAGES CXX )
add_executable( recorder captureRGBD.cpp captureRGBD.h DirectoryConfig.h)

# (Option) Build with Static CRT
if(WIN32)
  option( BUILD_STATIC_CRT "build with static crt" OFF )
  if( BUILD_STATIC_CRT )
    set( CXX_FLAGS
      CMAKE_CXX_FLAGS
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_RELEASE
    )
    foreach( CXX_FLAG ${CXX_FLAGS} )
      string( REPLACE "/MD" "/MT" ${CXX_FLAG} "${${CXX_FLAG}}" )
    endforeach()
  endif()
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT " record " )

# Find Package
find_package( OpenCV REQUIRED )
find_package( k4a REQUIRED )
find_package( k4arecord REQUIRED )
#if(NOT WIN32)
#  find_package( TBB REQUIRED )
#endif()

# Set Package to Project
if( k4a_FOUND AND k4arecord_FOUND AND OpenCV_FOUND )
  target_link_libraries( recorder k4a::k4a )
  target_link_libraries( recorder k4a::k4arecord )
  target_link_libraries( recorder ${OpenCV_LIBS} )
  target_link_libraries( recorder ${FILESYSTEM} )
endif()
#
#if( TBB_FOUND )
#  target_link_libraries( recorder TBB::tbb )
#endif()
