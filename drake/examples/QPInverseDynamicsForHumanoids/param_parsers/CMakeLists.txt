if (yaml-cpp_FOUND)
  add_library(drakeRigidBodyAliasGroups rigid_body_tree_alias_groups.cc)
  target_link_libraries(drakeRigidBodyAliasGroups
    drakeRBM
    yaml-cpp)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
