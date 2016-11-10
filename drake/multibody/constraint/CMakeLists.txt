add_library_with_exports(LIB_NAME drakeRigidBodyConstraint SOURCE_FILES RigidBodyConstraint.cpp)
target_link_libraries(drakeRigidBodyConstraint drakeRBM drakeGeometryUtil)
pods_install_libraries(drakeRigidBodyConstraint)
drake_install_headers(RigidBodyConstraint.h)
pods_install_pkg_config_file(drake-rigid-body-constraint
  LIBS -ldrakeRigidBodyConstraint
  REQUIRES
  VERSION 0.0.1)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
