cmake_minimum_required (VERSION 2.6)

add_executable(urdf_kin_test urdf_kin_test.cpp)
include_directories( .. )
target_link_libraries(urdf_kin_test URDFRigidBodyManipulator urdf_interface)

pods_use_pkg_config_packages(urdf_kin_test
  bot2-vis)
