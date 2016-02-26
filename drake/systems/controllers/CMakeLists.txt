drake_install_headers(LQR.h)

add_library_with_exports(LIB_NAME drakeControlUtil SOURCE_FILES controlUtil.cpp)
target_link_libraries(drakeControlUtil drakeRBM drakeUtil drakeTrajectories)
pods_install_libraries(drakeControlUtil)
drake_install_headers(controlUtil.h)
pods_install_pkg_config_file(drake-control-util
  LIBS -ldrakeControlUtil
  REQUIRES
  VERSION 0.0.1)

pods_find_pkg_config(gurobi)
if (gurobi_FOUND AND LCM_FOUND AND yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeQPCommon SOURCE_FILES InstantaneousQPController.cpp)
  target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil drakeSide drakeYAMLUtil)
  pods_use_pkg_config_packages(drakeQPCommon lcm)
  pods_install_libraries(drakeQPCommon)
  drake_install_headers(QPCommon.h InstantaneousQPController.h)
  pods_install_pkg_config_file(drake-qp-common
    LIBS -ldrakeQPCommon
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES drake-control-util
    VERSION 0.0.1)
    
  pods_find_pkg_config(bot2-lcmgl-client)
  if (bot2-lcmgl-client_FOUND)
    add_executable(zmpCoMObserverStateVisualizer zmpCoMObserverStateVisualizer.cpp)
    pods_use_pkg_config_packages(zmpCoMObserverStateVisualizer lcm bot2-lcmgl-client)
    target_link_libraries(zmpCoMObserverStateVisualizer drake_lcmtypes)
  endif()
endif()

if (MATLAB_FOUND)
  pods_find_pkg_config(gurobi)

  if (gurobi_FOUND AND LCM_FOUND AND yaml-cpp_FOUND)

    add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
    target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeSide)
    pods_use_pkg_config_packages(instantaneousQPControllermex gurobi)

    add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
    target_link_libraries(constructQPDataPointerMex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeYAMLUtil drakeAtlasUtil)
    pods_use_pkg_config_packages(constructQPDataPointerMex gurobi)
  endif()

  add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil) 

endif(MATLAB_FOUND)

add_library_with_exports(LIB_NAME drakeZMPUtil SOURCE_FILES zmpUtil.cpp)
target_link_libraries(drakeZMPUtil drakeTrajectories)
pods_install_libraries(drakeZMPUtil)
drake_install_headers(zmpUtil.h)


add_subdirectory(test)
