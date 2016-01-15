add_library_with_exports(LIB_NAME drakeSide SOURCE_FILES Side.cpp)
drake_install_headers(Side.h)
pods_install_libraries(drakeSide)
pods_install_pkg_config_file(drake-side
	LIBS -ldrakeSide
	CFLAGS
	REQUIRES
	VERSION 0.0.1)

if (LCM_FOUND)
  add_library_with_exports(LIB_NAME drakeQPLocomotionPlan SOURCE_FILES QPLocomotionPlan.cpp BodyMotionData.cpp) # TODO: move BodyMotionData to a better place
  target_link_libraries(drakeQPLocomotionPlan drakeSplineGeneration drakeRBM drakeLCMUtil drakeSide drakeConvexHull drakeAtlasUtil drakeZMPUtil)
  pods_use_pkg_config_packages(drakeQPLocomotionPlan lcm)
  pods_install_libraries(drakeQPLocomotionPlan)
  drake_install_headers(QPLocomotionPlan.h BodyMotionData.h)
  pods_install_pkg_config_file(drake-qp-locomotion-plan
  LIBS -ldrakeQPLocomotionPlan
  CFLAGS -I\${prefix}/include/lcmtypes
  REQUIRES
  VERSION 0.0.1)
endif()

if (MATLAB_FOUND)
  add_mex(footstepCollocationConstraintsMex footstepCollocationConstraintsMex.cpp)
  pods_use_pkg_config_packages(footstepCollocationConstraintsMex eigen3)

  pods_find_pkg_config(octomap 1.6.3)

  if (octomap_FOUND)
    add_mex(octomapWrapper octomapWrapper.cpp)
    target_link_libraries(octomapWrapper drakeUtil drakeMexUtil)  # note: drakeUtil depends on eigen
    pods_use_pkg_config_packages(octomapWrapper octomap)
  endif()

  if (LCM_FOUND AND NOT WIN32)
    add_mex(constructQPLocomotionPlanmex constructQPLocomotionPlanmex.cpp)
    target_link_libraries(constructQPLocomotionPlanmex drakeQPLocomotionPlan drakeUtil drakeZMPUtil drakeMexUtil)

    add_subdirectory(@QPLocomotionPlanCPPWrapper)
  endif()
endif()
