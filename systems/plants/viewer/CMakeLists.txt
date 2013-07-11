cmake_minimum_required (VERSION 2.6)

add_executable(drake_viewer main.cpp)
# add_executable(lcmgl_test lcmgl_test.cpp)

find_package( Boost REQUIRED filesystem system )

include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

add_library(drake_urdf_renderer drake_urdf_renderer.cpp)
target_link_libraries(drake_viewer drake_urdf_renderer lcmtypes_drake RigidBodyManipulator URDFRigidBodyManipulator urdf_interface ${Boost_LIBRARIES})

pods_use_pkg_config_packages(drake_viewer
  bot2-vis  
  bot2-lcmgl-renderer
  eigen3)
