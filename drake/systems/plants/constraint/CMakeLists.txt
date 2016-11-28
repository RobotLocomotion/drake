add_library_with_exports(LIB_NAME drakeDynamicConstraint
  SOURCE_FILES direct_collocation_constraint.cc)
target_link_libraries(drakeDynamicConstraint
  drakeSystemFramework
  drakeCommon)
drake_install_headers(direct_collocation_constraint.h)
drake_install_libraries(drakeDynamicConstraint)
drake_install_pkg_config_file(drake-dynamic-constraint
  TARGET drakeDynamicConstraint
  LIBS -ldrakeDynamicConstraint
  REQUIRES
    drake-common
    drake-system-framework
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
