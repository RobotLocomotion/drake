add_library_with_exports(LIB_NAME drakeMultibodyTree SOURCE_FILES
  rigid_body.cc
  multibody_tree.cc)

target_link_libraries(drakeMultibodyTree
  drakeCommon
  Eigen3::Eigen)
drake_install_libraries(drakeMultibodyTree)
drake_install_headers(
  body.h
  multibody_tree_element.h
  multibody_indexes.h
  multibody_tree.h
  rigid_body.h)

drake_install_pkg_config_file(drake-multibodytree
  TARGET drakeMultibodyTree
  LIBS -ldrakeMultibodyTree
  REQUIRES
  drake-common
  eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(math)
