add_library_with_exports(LIB_NAME drakeJointCompareToClone SOURCE_FILES
    joint_compare_to_clone.cc)

target_link_libraries(drakeJointCompareToClone
  drakeJoints)

drake_add_cc_test(joint_test)
target_link_libraries(joint_test drakeJointCompareToClone)
