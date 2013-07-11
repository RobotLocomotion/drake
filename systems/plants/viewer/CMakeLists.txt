cmake_minimum_required (VERSION 2.6)

add_executable(drake_viewer main.cpp)
# add_executable(lcmgl_test lcmgl_test.cpp)

include_directories ( .. )

add_library(drake_urdf_renderer drake_urdf_renderer.cpp)
target_link_libraries(drake_viewer drake_urdf_renderer lcmtypes_drake URDFRigidBodyManipulator)

pods_use_pkg_config_packages(drake_viewer
  bot2-lcmgl-renderer
  eigen3)
