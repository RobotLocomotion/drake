# RigidBodyTree unit tests.
drake_add_cc_test(test_kinematics_cache_checks)
target_link_libraries(test_kinematics_cache_checks
    drakeMultibodyParsers
    drakeRBM)

add_executable(benchmark_rigid_body_tree benchmark_rigid_body_tree.cc)
target_link_libraries(benchmark_rigid_body_tree
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(rigid_body_frame_test)
target_link_libraries(rigid_body_frame_test
    drakeRBM
    drakeRigidBodyFrameCompareToClone)

drake_add_cc_test(rigid_body_test)
target_link_libraries(rigid_body_test
    drakeRBM
    drakeRigidBodyCompareToClone)

add_executable(debug_manipulator_dynamics debug_manipulator_dynamics.cc)
target_link_libraries(debug_manipulator_dynamics
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(global_inverse_kinematics_test)
target_link_libraries(global_inverse_kinematics_test
        drakeGlobalIK
        drakeRigidBodyPlant
        drakeMultibodyParsers
        drakeCommon)

# Inverse kinematics unit tests.
macro(add_ik_cpp)
  drake_add_cc_test(${ARGV})
  target_link_libraries(${ARGV}
    drakeMultibodyParsers
    drakeRigidBodyConstraint
    drakeIK)
endmacro()

macro(add_ik_gtest)
  drake_add_cc_test(${ARGV})
  target_link_libraries(${ARGV}
    drakeIK
    drakeMultibodyParsers
    drakeRigidBodyConstraint)
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

add_library_with_exports(LIB_NAME drakeRigidBodyCompareToClone
    SOURCE_FILES rigid_body_compare_to_clone.cc)
target_link_libraries(drakeRigidBodyCompareToClone drakeRBM)

add_library_with_exports(LIB_NAME drakeRigidBodyActuatorCompareToClone
    SOURCE_FILES rigid_body_actuator_compare_to_clone.cc)
target_link_libraries(drakeRigidBodyActuatorCompareToClone
    drakeRBM
    drakeRigidBodyCompareToClone)

add_library_with_exports(LIB_NAME drakeRigidBodyFrameCompareToClone
    SOURCE_FILES rigid_body_frame_compare_to_clone.cc)
target_link_libraries(drakeRigidBodyFrameCompareToClone drakeRBM)

add_library_with_exports(LIB_NAME drakeRigidBodyLoopCompareToClone
    SOURCE_FILES rigid_body_loop_compare_to_clone.cc)
target_link_libraries(drakeRigidBodyLoopCompareToClone
  drakeRBM
  drakeRigidBodyFrameCompareToClone)

add_subdirectory(rigid_body_tree)
