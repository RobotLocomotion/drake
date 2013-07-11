cmake_minimum_required(VERSION 2.6.0)

find_package( Boost REQUIRED  )
include_directories ( ${Boost_INCLUDE_DIR} )

add_library(urdf_interface joint.cpp link.cpp)

