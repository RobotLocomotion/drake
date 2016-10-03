add_library_with_exports(LIB_NAME drakeSide SOURCE_FILES Side.cpp)
drake_install_headers(Side.h)
pods_install_libraries(drakeSide)
pods_install_pkg_config_file(drake-side
  LIBS -ldrakeSide
  CFLAGS
  REQUIRES
  VERSION 0.0.1)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeQPLocomotionPlan SOURCE_FILES QPLocomotionPlan.cpp BodyMotionData.cpp) # TODO: move BodyMotionData to a better place
  target_link_libraries(drakeQPLocomotionPlan drakeSplineGeneration drakeRBM drakeLCMUtil drakeSide drakeConvexHull drakeAtlasUtil drakeZMPUtil lcm)
  pods_install_libraries(drakeQPLocomotionPlan)
  drake_install_headers(QPLocomotionPlan.h BodyMotionData.h)
  pods_install_pkg_config_file(drake-qp-locomotion-plan
    LIBS -ldrakeQPLocomotionPlan
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES lcm
    VERSION 0.0.1)
endif()

if(MATLAB_FOUND)
  add_mex(footstepCollocationConstraintsMex footstepCollocationConstraintsMex.cpp)

  find_package(octomap)

  if(octomap_FOUND)
    add_mex(octomapWrapper octomapWrapper.cpp)
    target_include_directories(octomapWrapper PRIVATE ${OCTOMAP_INCLUDE_DIRS})
    target_link_libraries(octomapWrapper drakeUtil drakeMexUtil ${OCTOMAP_LIBRARIES})
  endif()

  if(lcm_FOUND AND NOT WIN32)
    add_mex(constructQPLocomotionPlanmex constructQPLocomotionPlanmex.cpp)
    target_link_libraries(constructQPLocomotionPlanmex drakeQPLocomotionPlan drakeUtil drakeZMPUtil drakeMexUtil)

    add_subdirectory(@QPLocomotionPlanCPPWrapper)
  endif()
endif()

add_subdirectory(calibration/test)
add_subdirectory(test)
