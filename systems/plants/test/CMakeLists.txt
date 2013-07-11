cmake_minimum_required (VERSION 2.6)

add_executable(urdf_kin_test urdf_kin_test.cpp)

find_package( Boost REQUIRED filesystem system )

include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

target_link_libraries(urdf_kin_test RigidBodyManipulator URDFRigidBodyManipulator urdf_interface ${Boost_LIBRARIES})

pods_use_pkg_config_packages(urdf_kin_test
  bot2-vis  
  eigen3)
