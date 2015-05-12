include_directories(../..)

if (MATLAB_FOUND)
  if (LCM_FOUND)
    add_mex(setDuration setDuration.cpp)
    target_link_libraries(setDuration drakeQPLocomotionPlan drakeMexUtil)
  endif()
endif()
