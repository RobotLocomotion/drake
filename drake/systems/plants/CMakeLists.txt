# IMPORTANT NOTE!!
# matlab has it's own boost libraries.  DO NOT let any mex file depend
# on the system boost (directly nor indirectly), or you're asking for trouble.

add_library_with_exports(LIB_NAME drakeXMLUtil SOURCE_FILES xmlUtil.cpp)
target_link_libraries(drakeXMLUtil drakeCommon tinyxml2 spruce)
pods_install_libraries(drakeXMLUtil)
drake_install_headers(xmlUtil.h)
pods_install_pkg_config_file(drake-xml-util
  LIBS -ldrakeXMLUtil -ldrakeCommmon -ltinyxml2 -lspruce
  REQUIRES
  VERSION 0.0.1)

add_library_with_exports(LIB_NAME drakeRBM SOURCE_FILES
  parser_model_instance_id_table.cc
  parser_common.cc
  parser_sdf.cc
  parser_urdf.cc
  RigidBodyTree.cpp
  RigidBody.cpp
  RigidBodyTreeContact.cpp
  rigid_body_actuator.cc
  rigid_body_frame.cc
  rigid_body_loop.cc
  $<TARGET_OBJECTS:drakeUtil>)
target_link_libraries(drakeRBM
        drakeCollision drakeCommon drakeJoints drakeXMLUtil)
pods_install_libraries(drakeRBM)
drake_install_headers(
  ConstraintWrappers.h
  ForceTorqueMeasurement.h
  KinematicPath.h
  KinematicsCache.h
  material_map.h
  pose_map.h
  rigid_body_actuator.h
  RigidBodyFrame.h
  RigidBody.h
  RigidBodyTree.h
  parser_model_instance_id_table.h
  parser_common.h
  parser_sdf.h
  parser_urdf.h
  rigid_body_loop.h
  robot_state_tap.h)

pods_install_pkg_config_file(drake-rbm
  LIBS -ldrakeRBM -ldrakeCollision -ldrakeJoints -ldrakeXMLUtil
  REQUIRES
  VERSION 0.0.1)

pods_find_pkg_config(gurobi)

add_subdirectory(shapes)
add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(joints)

if(Matlab_FOUND)
  drake_add_mex(rbmMexFunctions rigidBodyTreeMexFunctions.cpp)
  pods_install_libraries(rbmMexFunctions)
  target_link_libraries(rbmMexFunctions drakeRBM drakeMexUtil)
  # These warnings are not worth fixing.
  target_compile_options(rbmMexFunctions PRIVATE ${CXX_FLAGS_NO_SIGN_COMPARE})

  macro(add_rbm_mex)
    drake_add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} rbmMexFunctions)
    # These warnings are not worth fixing.
    target_compile_options(${ARGV} PRIVATE ${CXX_FLAGS_NO_SIGN_COMPARE})
  endmacro()

  add_rbm_mex(constructModelmex)
  add_rbm_mex(collisionDetectmex)
  add_rbm_mex(smoothDistancePenaltymex)
  add_rbm_mex(collisionRaycastmex)
  add_rbm_mex(allCollisionsmex)
  add_rbm_mex(collidingPointsmex)
  add_rbm_mex(collidingPointsCheckOnlymex)
  add_rbm_mex(findKinematicPathmex)
  add_rbm_mex(geometricJacobianmex)
  add_rbm_mex(doKinematicsmex)
  add_rbm_mex(forwardKinmex)
  add_rbm_mex(forwardKinJacobianmex)
  add_rbm_mex(forwardKinPositionGradientmex)
  add_rbm_mex(centerOfMassmex)
  add_rbm_mex(centerOfMassJacobianmex)
  add_rbm_mex(centroidalMomentumMatrixmex)
  add_rbm_mex(massMatrixmex)
  add_rbm_mex(dynamicsBiasTermmex)
  add_rbm_mex(geometricJacobianDotTimesVmex)
  add_rbm_mex(centroidalMomentumMatrixDotTimesVmex)
  add_rbm_mex(centerOfMassJacobianDotTimesVmex)
  add_rbm_mex(forwardJacDotTimesVmex)
  add_rbm_mex(contactConstraintsmex)
  add_rbm_mex(surfaceTangentsmex)
  add_rbm_mex(jointLimitConstraintsmex)
  add_rbm_mex(positionConstraintsmex)
  add_rbm_mex(testFrameParsermex)
  add_rbm_mex(computeS1Trajmex)
  add_rbm_mex(createKinematicsCachemex)
  add_rbm_mex(createKinematicsCacheAutoDiffmex)
  add_rbm_mex(velocityToPositionDotMappingmex)
  add_rbm_mex(positionDotToVelocityMappingmex)
  add_rbm_mex(collisionDetectFromPointsmex)
  target_link_libraries(computeS1Trajmex drakeZMPUtil)

  if(gurobi_FOUND)
    drake_add_mex(solveLCPmex solveLCPmex.cpp)
    target_compile_definitions(solveLCPmex PRIVATE "PATHLCP_MEXFILE=\"${PROJECT_SOURCE_DIR}/thirdParty/misc/pathlcp/lcppath.${Matlab_MEX_EXTENSION}\"")
    target_link_libraries(solveLCPmex drakeRBM drakeMexUtil drakeQP)
  endif()
endif()

add_library_with_exports(LIB_NAME drakeIKoptions SOURCE_FILES IKoptions.cpp)
target_link_libraries(drakeIKoptions drakeRBM)

if(Matlab_FOUND)
  macro(add_ikoptions_mex)
    drake_add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeMexUtil drakeIKoptions)
    # These warnings are not worth fixing.
    target_compile_options(${ARGV} PRIVATE ${CXX_FLAGS_NO_SIGN_COMPARE})
  endmacro()
  add_ikoptions_mex(IKoptionsmex)

  macro(add_constraint_mex)
    drake_add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeRigidBodyConstraint)
  endmacro()
endif()

set(drakeIK_SRC_FILES ik_trajectory_helper.cc inverseKin.cpp
  inverseKinBackend.cpp inverseKinPointwise.cpp inverseKinTraj.cpp
  inverseKinTrajBackend.cpp)
set(drakeIK_PODS_PKG)
if(gurobi_FOUND)
  set(drakeIK_SRC_FILES ${drakeIK_SRC_FILES} approximateIK.cpp)
  set(drakeIK_PODS_PKG ${drakeIK_PODS_PKG} gurobi)
endif()

if(drakeIK_SRC_FILES)
  add_library_with_exports(LIB_NAME drakeIK SOURCE_FILES ${drakeIK_SRC_FILES})
  if(drakeIK_PODS_PKG)
    pods_use_pkg_config_packages(drakeIK ${drakeIK_PODS_PKG})
  endif()
  target_link_libraries(drakeIK drakeRBM drakeOptimization drakeRigidBodyConstraint drakeIKoptions)
  pods_install_libraries(drakeIK)
  pods_install_libraries(drakeIKoptions)
  drake_install_headers(RigidBodyIK.h IKoptions.h ik_trajectory_helper.h
    inverseKinBackend.h)
  pods_install_pkg_config_file(drake-ik
    LIBS -ldrakeIK -ldrakeRigidBodyConstraint -ldrakeIKoptions
    REQUIRES drake-rbm
    VERSION 0.0.1)
endif()

if(Matlab_FOUND)
  if(gurobi_FOUND)
    drake_add_mex(approximateIKmex approximateIKmex.cpp)
    target_link_libraries(approximateIKmex drakeIK drakeRBM drakeMexUtil drakeRigidBodyConstraint)
    pods_use_pkg_config_packages(approximateIKmex gurobi)
  endif()

  macro(add_ik_mex)
    drake_add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeIK drakeRBM drakeMexUtil drakeRigidBodyConstraint)
    # These warnings are not worth fixing.
    target_compile_options(${ARGV} PRIVATE ${CXX_FLAGS_NO_SIGN_COMPARE})
  endmacro()

  find_package(snopt_c)

  if(snopt_c_FOUND)
    add_ik_mex(inverseKinmex)
    add_ik_mex(inverseKinPointwisemex)
    add_ik_mex(inverseKinTrajmex)
  endif()
endif()

add_library_with_exports(LIB_NAME drakeRBSystem SOURCE_FILES RigidBodySystem.cpp)
target_link_libraries(drakeRBSystem drakeRBM drakeOptimization drakeRigidBodyConstraint)

drake_install_headers(RigidBodySystem.h)
pods_install_libraries(drakeRBSystem)
pods_install_pkg_config_file(drake-rbsystem
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRBSystem -ldrakeRBM -ldrakeOptimization -ldrakeRigidBodyConstraint
  VERSION 0.0.1)

if(lcm_FOUND)
  add_executable(rigidBodyLCMNode rigidBodyLCMNode.cpp)
  target_link_libraries(rigidBodyLCMNode gflags drakeRBSystem drakeLCMSystem Threads::Threads)
  pods_install_executables(rigidBodyLCMNode)
  drake_install_headers(BotVisualizer.h)
endif()

add_subdirectory(rigid_body_plant)
add_subdirectory(test)

#System2.0 Plants
add_subdirectory(spring_mass_system)
