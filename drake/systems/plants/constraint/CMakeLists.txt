add_library_with_exports(LIB_NAME drakeRigidBodyConstraint SOURCE_FILES RigidBodyConstraint.cpp)
target_link_libraries(drakeRigidBodyConstraint drakeRBM drakeGeometryUtil)
pods_install_libraries(drakeRigidBodyConstraint)
drake_install_headers(RigidBodyConstraint.h)
pods_install_pkg_config_file(drake-rigid-body-constraint
  LIBS -ldrakeRigidBodyConstraint
  REQUIRES
  VERSION 0.0.1)

add_executable(testConstraintConstructor test/testConstraintConstructor.cpp)
target_link_libraries(testConstraintConstructor drakeRBM drakeRigidBodyConstraint)
drake_add_test(NAME testConstraintConstructor WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test" COMMAND testConstraintConstructor)

add_library_with_exports(LIB_NAME drakeDynamicConstraint
  SOURCE_FILES direct_collocation_constraint.cc)
target_link_libraries(drakeDynamicConstraint
  drakeSystemFramework
  drakeCommon)
drake_install_headers(direct_collocation_constraint.h)
pods_install_libraries(drakeDynamicConstraint)
pods_install_pkg_config_file(drake-dynamic-constraint
  LIBS -ldrakeDynamicConstraint
  REQUIRES
  VERSION 0.0.0)
add_subdirectory(test)
