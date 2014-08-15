

if (eigen3_FOUND)

  add_mex(footstepCollocationConstraintsMex footstepCollocationConstraintsMex.cpp)
  pods_use_pkg_config_packages(footstepCollocationConstraintsMex eigen3)

endif()

pods_find_pkg_config(octomap 1.6.3)

if (octomap_FOUND)
  include_directories (${CMAKE_SOURCE_DIR}/util )

  add_mex(octomapWrapper octomapWrapper.cpp)
  target_link_libraries(octomapWrapper drakeUtil)
  pods_use_pkg_config_packages(octomapWrapper octomap)

endif()
