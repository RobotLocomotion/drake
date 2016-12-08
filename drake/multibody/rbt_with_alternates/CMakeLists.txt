set(source_files
  rigid_body_tree_with_alternates.cc)

add_library_with_exports(LIB_NAME drakeRigidBodyTreeWithAlternates SOURCE_FILES
  ${source_files})
target_link_libraries(drakeRigidBodyTreeWithAlternates
  drakeCommon
  drakeRBM)

drake_install_headers(
  object_with_alternates.h
  rigid_body_tree_with_alternates.h)

drake_install_libraries(drakeRigidBodyTreeWithAlternates)
drake_install_pkg_config_file(drake-rigid-body-plant-with-alternates
  TARGET drakeRigidBodyTreeWithAlternates
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRigidBodyTreeWithAlternates
  REQUIRES
  drake-common
  drake-rbm
  eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
