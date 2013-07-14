cmake_minimum_required (VERSION 2.6)

if (eigen3_FOUND AND Boost_FOUND)
  add_executable(urdf_kin_test urdf_kin_test.cpp)
  include_directories( .. )
  target_link_libraries(urdf_kin_test URDFRigidBodyManipulator urdf_interface)
endif()
