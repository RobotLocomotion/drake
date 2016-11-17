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

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
