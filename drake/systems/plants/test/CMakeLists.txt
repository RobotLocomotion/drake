drake_find_package(Bullet MODULE)

add_executable(urdfKinTest urdfKinTest.cpp)
target_link_libraries(urdfKinTest drakeRBM)
if(Bullet_FOUND)
  add_executable(urdfCollisionTest urdfCollisionTest.cpp)
  target_link_libraries(urdfCollisionTest drakeRBM)
endif()

add_executable(urdfManipulatorDynamicsTest urdfManipulatorDynamicsTest.cpp)
target_link_libraries(urdfManipulatorDynamicsTest drakeRBM)

add_executable(testKinematicsCacheChecks testKinematicsCacheChecks.cpp)
target_link_libraries(testKinematicsCacheChecks drakeRBM)
drake_add_test(NAME testKinematicsCacheChecks WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND testKinematicsCacheChecks)

macro(add_ik_cpp)
  add_executable(${ARGV} ${ARGV}.cpp)
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK)
  drake_add_test(NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

macro(add_ik_gtest)
  add_executable(${ARGV} ${ARGV}.cpp)
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK
    GTest::GTest GTest::Main)
  drake_add_test(NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

if(gurobi_FOUND)
  add_ik_cpp(testApproximateIK)
endif()

pods_find_pkg_config(ipopt)
drake_find_package(snopt_c CONFIG)

if(snopt_c_FOUND OR ipopt_FOUND)
  add_ik_gtest(testIK)
  add_ik_gtest(testIKpointwise)
endif()
if(snopt_c_FOUND)
  add_ik_gtest(testIKMoreConstraints)
  add_ik_gtest(testIKtraj)
endif()

add_executable(debugManipulatorDynamics debugManipulatorDynamics.cpp)
target_link_libraries(debugManipulatorDynamics drakeRBM)

add_executable(benchmarkRigidBodyTree benchmarkRigidBodyTree.cpp)
target_link_libraries(benchmarkRigidBodyTree drakeRBM)

if(lcm_FOUND)
  drake_add_cc_test(testMassSpringDamper EXTENSION cpp)
  target_link_libraries(testMassSpringDamper drakeRBSystem)

  if(Bullet_FOUND AND bot2-core_FOUND)
    drake_add_cc_test(lidarTest EXTENSION cpp)
    target_link_libraries(lidarTest drakeRBSystem drakeLCMTypes lcmtypes_bot2-core-cpp lcm)
  endif()

  drake_add_cc_test(NAME testAccelerometer EXTENSION cpp)
  target_link_libraries(testAccelerometer drakeRBSystem)

  drake_add_cc_test(NAME testGyroscope EXTENSION cpp)
  target_link_libraries(testGyroscope drakeRBSystem)

  drake_add_cc_test(testMagnetometer EXTENSION cpp)
  target_link_libraries(testMagnetometer drakeRBSystem)

  add_executable(compareRigidBodySystems compareRigidBodySystems.cpp)
  target_link_libraries(compareRigidBodySystems drakeRBSystem GTest::GTest GTest::Main)
  drake_add_test(NAME compareAcrobotURDFtoSDF COMMAND compareRigidBodySystems Acrobot.urdf Acrobot.sdf WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/examples/Acrobot")

  if(snopt_c_FOUND)
    drake_add_test(NAME comparePriusURDFtoSDF COMMAND compareRigidBodySystems prius.sdf prius.urdf WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/automotive/models/prius")
  endif()

  drake_add_cc_test(load_model_test)
  target_link_libraries(load_model_test drakeRBSystem)
endif()

drake_add_cc_test(NAME testXmlUtil EXTENSION cpp)
target_link_libraries(testXmlUtil drakeXMLUtil)

drake_add_cc_test(urdf_parser_test)
target_link_libraries(urdf_parser_test drakeRBM)

drake_add_cc_test(rigid_body_test)
target_link_libraries(rigid_body_test drakeRBSystem)

add_subdirectory(drake_joint)
add_subdirectory(rigid_body_system)
add_subdirectory(rigid_body_tree)

# System2.0 Tests
drake_add_cc_test(spring_mass_system_test)
target_link_libraries(spring_mass_system_test drakeSpringMassSystemPlant drakeSystemFramework)
