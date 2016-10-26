if(Matlab_FOUND)
  if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
    drake_add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
    target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeSide drakeYAMLUtil)

    drake_add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
    target_link_libraries(constructQPDataPointerMex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeYAMLUtil drakeAtlasUtil)
  endif()

  drake_add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil)

endif()

