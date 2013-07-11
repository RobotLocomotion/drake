cmake_minimum_required(VERSION 2.6.0)

find_package( Boost REQUIRED filesystem system )
include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

#install(FILES RigidBodyManipulator.h RigidBody.h URDFRigidBodyManipulator.h DESTINATION includes/plants)

add_library(RigidBodyManipulator RigidBodyManipulator.cpp)
add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp)
target_link_libraries(URDFRigidBodyManipulator RigidBodyManipulator urdf_interface tinyxml ${Boost_LIBRARIES})

#add_mex(constructModelmex constructModelmex.cpp)

add_subdirectory(urdf_interface)
add_subdirectory(viewer)
add_subdirectory(test)

pods_use_pkg_config_packages(RigidBodyManipulator
  eigen3)

pods_use_pkg_config_packages(URDFRigidBodyManipulator
  bot2-vis)

# todo:  pod install (headers and libs) for rigidbodymanipulator?