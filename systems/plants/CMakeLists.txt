cmake_minimum_required(VERSION 2.6.0)

find_package( Boost COMPONENTS filesystem system )

if (Boost_FOUND)
  include_directories ( ${Boost_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/util )
  link_directories( ${Boost_LIBRARY_DIRS} )
endif()

pods_find_pkg_config(bot2-vis)
if (bot2-vis_FOUND)
  add_definitions( -DBOT_VIS_SUPPORT )   
endif()

#add_library(RigidBodyManipulator OBJECT RigidBodyManipulator.cpp)
#add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp $<TARGET_OBJECTS:RigidBodyManipulator>)
#target_link_libraries(URDFRigidBodyManipulator urdf_interface tinyxml ${Boost_LIBRARIES})

if (eigen3_FOUND)
  add_library(drakeRBM STATIC RigidBodyManipulator.cpp RigidBody.cpp)
  set_target_properties(drakeRBM PROPERTIES COMPILE_FLAGS -fPIC)

  if (Boost_FOUND)
    add_library(URDFRigidBodyManipulator URDFRigidBodyManipulator.cpp)
    target_link_libraries(URDFRigidBodyManipulator drakeRBM urdf_interface tinyxml ${Boost_LIBRARIES})

    if (bot2-vis_FOUND)
	   pods_use_pkg_config_packages(URDFRigidBodyManipulator bot2-vis)
    endif()

    add_subdirectory(urdf_interface)
    add_subdirectory(viewer)
  endif(Boost_FOUND)

  macro(add_rbm_mex)
    # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
    #  add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
    #  target_link_libraries(${ARGV} drakeRBM)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeUtil)
  endmacro()

  add_rbm_mex(constructModelmex)
  add_rbm_mex(HandCmex)
  add_rbm_mex(doKinematicsmex)
  add_rbm_mex(forwardKinmex)
  add_rbm_mex(bodyKinmex)

  pods_find_pkg_config(gurobi)
  if (gurobi_FOUND)
    add_rbm_mex(approximateIKmex)
    pods_use_pkg_config_packages(approximateIKmex gurobi)
  endif()

  #	approximateIKEIQPmex.cpp
  #	collisionmex.cpp 

  macro(add_planar_rbm_mex)
    # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
    #  add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeUtil)
  endmacro()

  add_subdirectory(@PlanarRigidBodyManipulator/private)
  add_planar_rbm_mex(deleteModelpmex)
  add_planar_rbm_mex(doKinematicspmex)
  add_planar_rbm_mex(HandCpmex)
  add_planar_rbm_mex(forwardKinpmex)
  add_planar_rbm_mex(forwardKinVelpmex)

  add_subdirectory(test)

  pods_install_libraries(drakeRBM)
  pods_install_headers(RigidBodyManipulator.h RigidBody.h DESTINATION drake)
  pods_install_pkg_config_file(drake-rbm
    LIBS -ldrakeRBM
    REQUIRES  
    VERSION 0.0.1)

endif(eigen3_FOUND)

