cmake_minimum_required(VERSION 2.6.0)

pods_install_headers(RigidBodyManipulator.h RigidBody.h URDFRigidBodyManipulator.h DESTINATION plants)

add_library(RigidBodyManipulator RigidBodyManipulator.cpp)
add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp)
target_link_libraries(URDFRigidBodyManipulator RigidBodyManipulator urdf_interface tinyxml)

add_subdirectory(urdf_interface)
add_subdirectory(viewer)


pods_use_pkg_config_packages(RigidBodyManipulator
  eigen3)

pods_use_pkg_config_packages(URDFRigidBodyManipulator
  bot2-vis  
  eigen3)
