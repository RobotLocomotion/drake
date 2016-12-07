# RigidBodyTree unit tests.
drake_add_cc_test(test_kinematics_cache_checks)
target_link_libraries(test_kinematics_cache_checks drakeRBM)

add_executable(benchmark_rigid_body_tree benchmark_rigid_body_tree.cc)
target_link_libraries(benchmark_rigid_body_tree drakeRBM)

drake_add_cc_test(rigid_body_test)
target_link_libraries(rigid_body_test drakeRBM)

add_executable(debug_manipulator_dynamics debug_manipulator_dynamics.cc)
target_link_libraries(debug_manipulator_dynamics drakeRBM)

# RigidBodyTree parsers unit tests.
drake_add_cc_test(xml_util_test)
target_link_libraries(xml_util_test drakeXMLUtil)

add_executable(urdf_kinematics_test urdf_kinematics_test.cc)
target_link_libraries(urdf_kinematics_test drakeRBM)
if (Bullet_FOUND)
  add_executable(urdf_collision_test urdf_collision_test.cc)
  target_link_libraries(urdf_collision_test drakeRBM)
endif ()

add_executable(urdf_manipulator_dynamics_test urdf_manipulator_dynamics_test.cc)
target_link_libraries(urdf_manipulator_dynamics_test drakeRBM)

# Inverse kinematics unit tests.
macro(add_ik_cpp)
  add_executable(${ARGV} ${ARGV}.cc)
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK)
  drake_add_test(NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

macro(add_ik_gtest)
  add_executable(${ARGV} ${ARGV}.cc)
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK
    GTest::GTest GTest::Main)
  drake_add_test(NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

if (gurobi_FOUND)
  add_ik_cpp(test_approximate_ik)
endif ()

if (snopt_FOUND OR ipopt_FOUND)
  add_ik_gtest(test_ik)
  add_ik_gtest(test_ik_pointwise)
endif ()
if (snopt_FOUND)
  add_ik_gtest(test_ik_more_constraints)
  add_ik_gtest(test_ik_traj)
endif ()

add_subdirectory(parser_urdf_test)
add_subdirectory(rigid_body_tree)
