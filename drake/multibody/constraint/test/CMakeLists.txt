drake_add_cc_test(NAME rigid_body_constraint_test)
target_link_libraries(rigid_body_constraint_test
    # TODO(liang.fok) Remove this once the examples-dependency layering
    # violation is resolved.
    drakeExamplesCommon
    drakeRBM
    drakeRigidBodyConstraint)
