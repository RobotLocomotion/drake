cmake_minimum_required(VERSION 2.6.0)

find_package( Boost REQUIRED  )
include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

#install(FILES *.h DESTINATION include/urdf_interface)

add_library(urdf_interface joint.cpp link.cpp)
target_link_libraries(urdf_interface ${Boost_LIBRARIES})

