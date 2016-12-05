if(Matlab_FOUND)
  if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
    drake_add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
    target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeSide drakeYAMLUtil)

    drake_add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
    target_link_libraries(constructQPDataPointerMex
        # TODO(liang.fok) Remove this once the examples-dependency layering
        # violation is resolved.
        drakeExamplesCommon
        drakeAtlasUtil
        drakeControlUtil
        drakeMexUtil
        drakeQP
        drakeQPCommon
        drakeYAMLUtil
        )
  endif()

  drake_add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil)

endif()
