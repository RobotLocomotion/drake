

include_directories (${CMAKE_SOURCE_DIR}/util tinyxml)

# IMPORTANT NOTE!!
# matlab has it's own boost libraries.  DO NOT let any mex file depend
# on the system boost (directly nor indirectly), or you're asking for trouble.
find_package( Boost COMPONENTS filesystem system regex)

if (Boost_FOUND)
  include_directories ( ${Boost_INCLUDE_DIR} )
  link_directories( ${Boost_LIBRARY_DIRS} )
endif()

if (eigen3_FOUND)
  # todo: check eigen version.  3.1.0 didn't work for clang.  3.2.0 did.
 add_library(drakeRBM SHARED RigidBodyManipulator.cpp RigidBody.cpp RigidBodyManipulatorURDF.cpp RigidBodyManipulatorContact.cpp tinyxml/tinyxml.cpp tinyxml/tinyxmlparser.cpp tinyxml/tinyxmlerror.cpp)
 target_link_libraries(drakeRBM drakeCollision)
 include_directories( joints )
 target_link_libraries(drakeRBM drakeJoints)
 target_link_libraries(drakeRBM spruce)
  pods_install_libraries(drakeRBM)
  pods_install_headers(RigidBodyManipulator.h RigidBody.h RigidBodyFrame.h KinematicPath.h DESTINATION drake)
  pods_install_pkg_config_file(drake-rbm
    LIBS -ldrakeRBM
    REQUIRES
    VERSION 0.0.1)

  add_subdirectory(shapes)
  add_subdirectory(collision)
  add_subdirectory(constraint)
  add_subdirectory(joints)

  if (Boost_FOUND)
    add_subdirectory(viewer)
  endif(Boost_FOUND)

  macro(add_rbm_mex)
    # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
#    add_mex(${ARGV} ${ARGV}.cpp $<TARGET_OBJECTS:drakeUtil>)
#    target_link_libraries(${ARGV} drakeRBM)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeUtil)
  endmacro()

  add_rbm_mex(constructModelmex)
  add_rbm_mex(HandCmex)
  add_rbm_mex(doKinematicsmex)
  add_rbm_mex(forwardKinmex)
  add_rbm_mex(bodyKinmex)
  add_rbm_mex(collisionDetectmex)
  add_rbm_mex(smoothDistancePenaltymex)
  add_rbm_mex(collisionRaycastmex)
  add_rbm_mex(allCollisionsmex)
  add_rbm_mex(getCMMmex)
  add_rbm_mex(findKinematicPathmex)
  add_rbm_mex(geometricJacobianmex)
  add_rbm_mex(doKinematicsNewmex)
  add_rbm_mex(forwardKinVmex)
  add_rbm_mex(forwardKinPositionGradientmex)
  add_rbm_mex(centerOfMassmex)
  add_rbm_mex(centroidalMomentumMatrixmex)
  add_rbm_mex(massMatrixmex)
  add_rbm_mex(inverseDynamicsmex)
  add_rbm_mex(geometricJacobianDotTimesVmex)
  add_rbm_mex(centroidalMomentumMatrixDotTimesVmex)
  add_rbm_mex(centerOfMassJacobianDotTimesVmex)
  add_rbm_mex(forwardJacDotTimesVmex)
  add_rbm_mex(contactConstraintsmex)
  add_rbm_mex(surfaceTangentsmex)
  add_rbm_mex(jointLimitConstraintsmex)
  add_rbm_mex(positionConstraintsmex)

  include_directories( constraint )
  add_library(drakeIKoptions SHARED IKoptions.cpp)
  target_link_libraries(drakeIKoptions drakeRBM)
  macro(add_ikoptions_mex)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeUtil drakeIKoptions)
  endmacro()
  add_ikoptions_mex(IKoptionsmex)
  add_ikoptions_mex(testIKoptionsmex)

  pods_find_pkg_config(gurobi)
  pods_find_pkg_config(snopt_c)

  macro(add_constraint_mex)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeUtil drakeRigidBodyConstraint)
  endmacro()

  set(drakeIK_SRC_FILES IKoptions.cpp)
  set(drakeIK_PODS_PKG )
  if(gurobi_FOUND)
    set(drakeIK_SRC_FILES ${drakeIK_SRC_FILES} approximateIK.cpp)
    set(drakeIK_PODS_PKG ${drakeIK_PODS_PKG} gurobi)
  endif()
  if(snopt_c_FOUND)
    set(drakeIK_SRC_FILES ${drakeIK_SRC_FILES} inverseKin.cpp inverseKinPointwise.cpp inverseKinTraj.cpp inverseKinBackend.cpp)
    set(drakeIK_PODS_PKG ${drakeIK_PODS_PKG} snopt_c)
  endif()
  add_library(drakeIK SHARED ${drakeIK_SRC_FILES})
  pods_use_pkg_config_packages(drakeIK ${drakeIK_PODS_PKG})
  target_link_libraries(drakeIK drakeRBM drakeRigidBodyConstraint drakeIKoptions)
  set_target_properties(drakeIK PROPERTIES COMPILE_FLAGS -fPIC)
  pods_install_libraries(drakeIK drakeIKoptions)
  pods_install_headers(RigidBodyIK.h IKoptions.h DESTINATION drake)

  if (gurobi_FOUND)
    add_mex(approximateIKmex approximateIKmex.cpp)
    target_link_libraries(approximateIKmex drakeIK drakeRBM drakeUtil drakeRigidBodyConstraint)
    pods_use_pkg_config_packages(approximateIKmex gurobi)
  endif()

  macro(add_IK_mex)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeIK drakeRBM drakeUtil drakeRigidBodyConstraint)
    pods_use_pkg_config_packages(${ARGV} snopt_c)
  endmacro()

  if (snopt_c_FOUND)
    add_IK_mex(inverseKinmex)
    add_IK_mex(inverseKinPointwisemex)
    add_IK_mex(inverseKinTrajmex)
  endif()

  add_subdirectory(test)

endif(eigen3_FOUND)
