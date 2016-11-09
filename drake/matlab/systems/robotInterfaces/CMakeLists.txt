if(Matlab_FOUND)
  drake_add_mex(footstepCollocationConstraintsMex footstepCollocationConstraintsMex.cpp)

  if(octomap_FOUND)
    drake_add_mex(octomapWrapper octomapWrapper.cpp $<TARGET_OBJECTS:drakeUtil>)
    target_link_libraries(octomapWrapper drakeMexUtil octomap)
  endif()

  if(lcm_FOUND AND NOT WIN32)
    drake_add_mex(constructQPLocomotionPlanmex constructQPLocomotionPlanmex.cpp $<TARGET_OBJECTS:drakeUtil>)
    target_link_libraries(constructQPLocomotionPlanmex drakeQPLocomotionPlan drakeZMPUtil drakeMexUtil)

    add_subdirectory(@QPLocomotionPlanCPPWrapper)
  endif()
endif()

add_subdirectory(calibration/test)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
