
find_package( Boost REQUIRED  )
include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

add_library(urdf_interface joint.cpp link.cpp)
target_link_libraries(urdf_interface ${Boost_LIBRARIES})

