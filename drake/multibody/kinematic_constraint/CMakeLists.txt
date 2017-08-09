add_library_with_exports(LIB_NAME drakeRigidBodyConstraint SOURCE_FILES rigid_body_constraint.cc)
target_link_libraries(drakeRigidBodyConstraint
  drakeGeometryUtil
  drakeRBM
  Eigen3::Eigen)
drake_install_libraries(drakeRigidBodyConstraint)
drake_install_headers(rigid_body_constraint.h)
drake_install_pkg_config_file(drake-rigid-body-constraint
  TARGET drakeRigidBodyConstraint
  LIBS -ldrakeRigidBodyConstraint
  REQUIRES
    drake-geometry-util
    drake-rbm
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
