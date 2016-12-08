
drake_install_headers(
    object_with_alternates.h
    rigid_body_tree_with_alternates.h)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
