include_directories(..)

if (MATLAB_FOUND)
  if (LCM_FOUND)
    add_mex(getQPControllerInput getQPControllerInput.cpp)
    target_link_libraries(getQPControllerInput drakeQPLocomotionPlan drakeUtil)
    
    add_mex(start_time start_time.cpp)
    target_link_libraries(start_time drakeQPLocomotionPlan drakeUtil)
    
    add_mex(duration duration.cpp)
    target_link_libraries(duration drakeQPLocomotionPlan drakeUtil)
  endif()
endif()

add_subdirectory(private)
