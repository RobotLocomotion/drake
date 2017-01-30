if(Matlab_FOUND)
  drake_add_mex(footstepCollocationConstraintsMex footstepCollocationConstraintsMex.cpp)
  target_link_libraries(footstepCollocationConstraintsMex Eigen3::Eigen)

  if(octomap_FOUND)
    drake_add_mex(octomapWrapper octomapWrapper.cpp $<TARGET_OBJECTS:drakeUtil>)
    target_link_libraries(octomapWrapper
      drakeMexUtil
      Eigen3::Eigen
      octomap)
  endif()

  if(lcm_FOUND)
    drake_add_mex(constructQPLocomotionPlanmex constructQPLocomotionPlanmex.cpp $<TARGET_OBJECTS:drakeUtil>)
    target_link_libraries(constructQPLocomotionPlanmex
      drakeMexUtil
      drakeQPLocomotionPlan
      drakeZMPUtil
      Eigen3::Eigen)

    add_subdirectory(@QPLocomotionPlanCPPWrapper)
  endif()
endif()

add_subdirectory(calibration/test)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
