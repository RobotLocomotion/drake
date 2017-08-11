add_library_with_exports(LIB_NAME drakeRBM SOURCE_FILES
    kinematics_cache.cc
    rigid_body_tree.cc
    rigid_body.cc
    rigid_body_tree_contact.cc
    rigid_body_actuator.cc
    rigid_body_frame.cc
    rigid_body_loop.cc
    rigid_body_tree_construction.cc
    $<TARGET_OBJECTS:drakeUtil>)

add_library_with_exports(LIB_NAME drakeRigidBodyConstraint SOURCE_FILES rigid_body_constraint.cc)

drake_install_libraries(drakeRigidBodyConstraint)
drake_install_headers(rigid_body_constraint.h)
drake_install_pkg_config_file(drake-rigid-body-constraint
  TARGET drakeRigidBodyConstraint
  LIBS -ldrakeRigidBodyConstraint
  REQUIRES
    drake-geometry-util
    drake-rbm
    eigen3)

target_link_libraries(drakeRBM
    drakeCollision
    drakeCommon
    drakeJoints
    Eigen3::Eigen)
target_link_libraries(drakeRigidBodyConstraint
  drakeGeometryUtil
  drakeRBM
  Eigen3::Eigen)

drake_install_libraries(drakeRBM)
drake_install_headers(
    constraint_wrappers.h
    force_torque_measurement.h
    kinematic_path.h
    kinematics_cache.h
    kinematics_cache-inl.h
    material_map.h
    pose_map.h
    rigid_body.h
    rigid_body_actuator.h
    rigid_body_frame.h
    rigid_body_loop.h
    rigid_body_tree.h
    rigid_body_tree_construction.h)

drake_install_pkg_config_file(drake-rbm
    TARGET drakeRBM
    LIBS -ldrakeRBM
    REQUIRES
        drake-collision
        drake-common
        drake-joints
        eigen3)

add_library_with_exports(LIB_NAME drakeIKoptions SOURCE_FILES ik_options.cc)
target_link_libraries(drakeIKoptions drakeRBM)
drake_install_pkg_config_file(drake-ik-options
    TARGET drakeIKoptions
    LIBS -ldrakeIKoptions
    REQUIRES
        drake-rbm
        eigen3)

set(drakeIK_SRC_FILES
    constraint_wrappers.cc
    ik_trajectory_helper.cc
    inverse_kinematics.cc
    inverse_kinematics_backend.cc
    inverse_kinematics_pointwise.cc
    inverse_kinematics_trajectory.cc
    inverse_kinematics_trajectory_backend.cc)
add_library_with_exports(LIB_NAME drakeIK SOURCE_FILES ${drakeIK_SRC_FILES})
target_link_libraries(drakeIK
    drakeIKoptions
    drakeOptimization
    drakeRBM
    drakeRigidBodyConstraint
    Eigen3::Eigen)
drake_install_libraries(drakeIK)
drake_install_libraries(drakeIKoptions)
drake_install_headers(
    constraint_wrappers.h
    ik_options.h
    ik_trajectory_helper.h
    inverse_kinematics_backend.h
    rigid_body_ik.h)
drake_install_pkg_config_file(drake-ik
    TARGET drakeIK
    LIBS -ldrakeIK
    REQUIRES
        drake-ik-options
        drake-optimization
        drake-rbm
        drake-rigid-body-constraint
        eigen3)

add_library_with_exports(LIB_NAME drakeGlobalIK SOURCE_FILES global_inverse_kinematics.cc)
target_link_libraries(drakeGlobalIK
        drakeOptimization
        drakeRotationConstraint
        drakeRBM
        Eigen3::Eigen)
drake_install_libraries(drakeGlobalIK)
drake_install_headers(global_inverse_kinematics.h)

add_subdirectory(benchmarks)
add_subdirectory(collision)
add_subdirectory(joints)
add_subdirectory(multibody_tree)
add_subdirectory(parsers)
add_subdirectory(rigid_body_plant)
add_subdirectory(shapes)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
