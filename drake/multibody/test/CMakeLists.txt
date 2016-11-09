# RigidBodyTree unit tests.
add_executable(testKinematicsCacheChecks testKinematicsCacheChecks.cpp)
target_link_libraries(testKinematicsCacheChecks drakeRBM)
drake_add_test(NAME testKinematicsCacheChecks WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND testKinematicsCacheChecks)

add_executable(benchmarkRigidBodyTree benchmarkRigidBodyTree.cpp)
target_link_libraries(benchmarkRigidBodyTree drakeRBM)

drake_add_cc_test(rigid_body_test)
target_link_libraries(rigid_body_test drakeRBM)

add_executable(debugManipulatorDynamics debugManipulatorDynamics.cpp)
target_link_libraries(debugManipulatorDynamics drakeRBM)

# RigidBodyTree parsers unit tests.
drake_add_cc_test(NAME testXmlUtil EXTENSION cpp)
target_link_libraries(testXmlUtil drakeXMLUtil)

add_executable(urdfKinTest urdfKinTest.cpp)
target_link_libraries(urdfKinTest drakeRBM)
if (Bullet_FOUND)
  add_executable(urdfCollisionTest urdfCollisionTest.cpp)
  target_link_libraries(urdfCollisionTest drakeRBM)
endif ()

add_executable(urdfManipulatorDynamicsTest urdfManipulatorDynamicsTest.cpp)
target_link_libraries(urdfManipulatorDynamicsTest drakeRBM)

drake_add_cc_test(urdf_parser_test)
target_link_libraries(urdf_parser_test drakeRBM)

# Inverse kinematics unit tests.
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

if (gurobi_FOUND)
  add_ik_cpp(testApproximateIK)
endif ()

if (snopt_FOUND OR ipopt_FOUND)
  add_ik_gtest(testIK)
  add_ik_gtest(testIKpointwise)
endif ()
if (snopt_FOUND)
  add_ik_gtest(testIKMoreConstraints)
  add_ik_gtest(testIKtraj)
endif ()

add_subdirectory(rigid_body_tree)
