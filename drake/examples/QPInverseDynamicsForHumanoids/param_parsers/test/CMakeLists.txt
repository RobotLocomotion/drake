if (yaml-cpp_FOUND)
  drake_add_cc_test(rigid_body_tree_alias_groups_test)
  target_link_libraries(rigid_body_tree_alias_groups_test
      drakeMultibodyParsers
      drakeRigidBodyAliasGroups)

  drake_add_cc_test(param_parser_test param_parser_test.cc)
  target_link_libraries(param_parser_test
      drakeMultibodyParsers
      drakeQPController
      drakeQPControllerParamParsing
      drakeRigidBodyAliasGroups)
endif()
