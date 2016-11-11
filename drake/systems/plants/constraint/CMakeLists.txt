add_library_with_exports(LIB_NAME drakeDynamicConstraint
  SOURCE_FILES direct_collocation_constraint.cc)
target_link_libraries(drakeDynamicConstraint
  drakeSystemFramework
  drakeCommon)
drake_install_headers(direct_collocation_constraint.h)
pods_install_libraries(drakeDynamicConstraint)
drake_install_pkg_config_file(drake-dynamic-constraint
  LIBS -ldrakeDynamicConstraint
  REQUIRES
    drake-common
    drake-system-framework)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
