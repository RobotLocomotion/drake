add_library_with_exports(LIB_NAME drakeRigidBodyConstraint SOURCE_FILES rigid_body_constraint.cc)
target_link_libraries(drakeRigidBodyConstraint drakeRBM drakeGeometryUtil)
pods_install_libraries(drakeRigidBodyConstraint)
drake_install_headers(rigid_body_constraint.h)
pods_install_pkg_config_file(drake-rigid-body-constraint
  LIBS -ldrakeRigidBodyConstraint
  REQUIRES
  VERSION 0.0.1)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
