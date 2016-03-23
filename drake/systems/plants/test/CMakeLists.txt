cmake_minimum_required (VERSION 2.6)

pods_find_pkg_config(bullet)

if (MATLAB_FOUND)
  add_rbm_mex(compareParsersmex)
endif (MATLAB_FOUND)

add_executable(urdfKinTest urdfKinTest.cpp)
target_link_libraries(urdfKinTest drakeRBM)
if (bullet_FOUND)
  add_executable(urdfCollisionTest urdfCollisionTest.cpp)
  target_link_libraries(urdfCollisionTest drakeRBM)
endif()

add_executable(urdfManipulatorDynamicsTest urdfManipulatorDynamicsTest.cpp)
target_link_libraries(urdfManipulatorDynamicsTest drakeRBM)

if (NOT WIN32)
  # this test does not compile on MSVC 2010 because of the use of variadic templates
  add_executable(testKinematicsCacheChecks testKinematicsCacheChecks.cpp)
  target_link_libraries(testKinematicsCacheChecks drakeRBM)
  add_test(NAME testKinematicsCacheChecks WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND testKinematicsCacheChecks)
endif()

macro(add_ik_cpp)
  add_executable(${ARGV} ${ARGV}.cpp)
  set_target_properties(${ARGV} PROPERTIES COMPILE_FLAGS "-fPIC")
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK)
  add_test( NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

if (gurobi_FOUND)
  add_ik_cpp(testApproximateIK)
endif()

if (snopt_c_FOUND)
  add_ik_cpp(testIK)
  add_ik_cpp(testIKMoreConstraints)
  add_ik_cpp(testIKpointwise)
  add_ik_cpp(testIKtraj)
endif()

add_executable(debugManipulatorDynamics debugManipulatorDynamics.cpp)
target_link_libraries(debugManipulatorDynamics drakeRBM)

add_executable(benchmarkRigidBodyTree benchmarkRigidBodyTree.cpp)
target_link_libraries(benchmarkRigidBodyTree drakeRBM)

if (MATLAB_FOUND)
  macro(add_ikoptions_mex)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeMexUtil drakeIKoptions)
  endmacro()
  add_ikoptions_mex(testIKoptionsmex)
endif()

add_executable(bullet_collision_zero_rad_sphere_test bullet_collision_zero_rad_sphere_test.cpp)
target_link_libraries(bullet_collision_zero_rad_sphere_test drakeRBM)
add_test(NAME bullet_collision_zero_rad_sphere_test COMMAND bullet_collision_zero_rad_sphere_test)

if (LCM_FOUND)
  add_executable(testMassSpringDamper testMassSpringDamper.cpp)
  target_link_libraries(testMassSpringDamper drakeRBSystem)
  add_test(NAME testMassSpringDamper COMMAND testMassSpringDamper)

  if (bullet_FOUND)
    add_executable(testLidar lidarTest.cpp)
    target_link_libraries(testLidar drakeRBSystem)
    add_test(NAME testLidar COMMAND testLidar)
  endif()

  add_executable(testAccelerometer testAccelerometer.cpp)
  target_link_libraries(testAccelerometer drakeRBSystem)
  add_test(NAME testAccelerometer COMMAND testAccelerometer)

  add_executable(testGyroscope testGyroscope.cpp)
  target_link_libraries(testGyroscope drakeRBSystem)
  add_test(NAME testGyroscope COMMAND testGyroscope)

  add_executable(testMagnetometer testMagnetometer.cpp)
  target_link_libraries(testMagnetometer drakeRBSystem)
  add_test(NAME testMagnetometer COMMAND testMagnetometer)

  add_executable(compareRigidBodySystems compareRigidBodySystems.cpp)
  target_link_libraries(compareRigidBodySystems drakeRBSystem)
  add_test(NAME compareAcrobotURDFtoSDF COMMAND compareRigidBodySystems Acrobot.urdf Acrobot.sdf WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/examples/Acrobot")
endif()

