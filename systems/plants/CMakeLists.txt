cmake_minimum_required(VERSION 2.6.0)

find_package( Boost REQUIRED filesystem system )
include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

add_library(RigidBodyManipulator OBJECT RigidBodyManipulator.cpp)

add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp $<TARGET_OBJECTS:RigidBodyManipulator>)
target_link_libraries(URDFRigidBodyManipulator urdf_interface tinyxml ${Boost_LIBRARIES})
pods_use_pkg_config_packages(URDFRigidBodyManipulator bot2-vis)

macro(add_rbm_mex)
  add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:RigidBodyManipulator> $<TARGET_OBJECTS:drakeUtil>)
endmacro()

add_rbm_mex(constructModelmex)
add_rbm_mex(HandCmex)
add_rbm_mex(doKinematicsmex)
add_rbm_mex(forwardKinmex)
add_rbm_mex(bodyKinmex)
add_rbm_mex(approximateIKmex)
pods_use_pkg_config_packages(approximateIKmex gurobi)
#	approximateIKEIQPmex.cpp
#	collisionmex.cpp 

macro(add_planar_rbm_mex)
  add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
endmacro()

add_subdirectory(@PlanarRigidBodyManipulator/private)
add_planar_rbm_mex(deleteModelpmex)
add_planar_rbm_mex(doKinematicspmex)
add_planar_rbm_mex(HandCpmex)
add_planar_rbm_mex(forwardKinpmex)
add_planar_rbm_mex(forwardKinVelpmex)

add_subdirectory(urdf_interface)
add_subdirectory(viewer)
add_subdirectory(test)

# todo:  pod install (headers and libs) for rigidbodymanipulator?
