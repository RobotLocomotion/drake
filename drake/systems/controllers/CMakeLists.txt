drake_install_headers(
  LQR.h
  open_loop_trajectory_controller.h
  simple_feedback_trajectory_controller.h)

add_library_with_exports(LIB_NAME drakeControlUtil SOURCE_FILES controlUtil.cpp)
target_link_libraries(drakeControlUtil drakeRBM drakeUtil drakeTrajectories)
pods_install_libraries(drakeControlUtil)
drake_install_headers(controlUtil.h)
pods_install_pkg_config_file(drake-control-util
  LIBS -ldrakeControlUtil
  REQUIRES
  VERSION 0.0.1)

pods_find_pkg_config(gurobi)
find_package(yaml-cpp)
if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeQPCommon SOURCE_FILES InstantaneousQPController.cpp)
  target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil drakeSide drakeYAMLUtil lcm)
  pods_use_pkg_config_packages(drakeQPCommon gurobi)
  pods_install_libraries(drakeQPCommon)
  drake_install_headers(QPCommon.h InstantaneousQPController.h)
  pods_install_pkg_config_file(drake-qp-common
    LIBS -ldrakeQPCommon
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES drake-control-util
    VERSION 0.0.1)

  pods_find_pkg_config(bot2-lcmgl-client)
  if(bot2-lcmgl-client_FOUND)
    add_executable(zmpCoMObserverStateVisualizer zmpCoMObserverStateVisualizer.cpp)
    pods_use_pkg_config_packages(zmpCoMObserverStateVisualizer bot2-lcmgl-client)
    target_link_libraries(zmpCoMObserverStateVisualizer drake_lcmtypes lcm)
  endif()
endif()

if(MATLAB_FOUND)
  pods_find_pkg_config(gurobi)

  if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)

    add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
    target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeSide drakeYAMLUtil)
    pods_use_pkg_config_packages(instantaneousQPControllermex gurobi)

    add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
    target_link_libraries(constructQPDataPointerMex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeYAMLUtil drakeAtlasUtil)
    pods_use_pkg_config_packages(constructQPDataPointerMex gurobi)
  endif()

  add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil)

endif()

add_library_with_exports(LIB_NAME drakeZMPUtil SOURCE_FILES zmpUtil.cpp)
target_link_libraries(drakeZMPUtil drakeTrajectories)
pods_install_libraries(drakeZMPUtil)
drake_install_headers(zmpUtil.h)
pods_install_pkg_config_file(drake-zmp-util
  LIBS -ldrakeZMPUtil
  REQUIRES
  VERSION 0.0.1)


# System 2 controllers
set(system2_sources
    gravity_compensator.cc)
set(system2_installed_headers
    gravity_compensator.h)
add_library_with_exports(LIB_NAME drakeSystemControllers
    SOURCE_FILES ${system2_sources} ${system2_installed_headers})
add_dependencies(drakeSystemControllers drakeCommon drakeRBM drakeSystemFramework)
target_link_libraries(drakeSystemControllers
    drakeCommon
    drakeRBM
    drakeSystemFramework)

drake_install_headers(${system2_installed_headers})

pods_install_libraries(drakeSystemControllers)
pods_install_pkg_config_file(drake-system2-controllers
    LIBS -ldrakeSystem2Controllers -ldrakeSystemFramework -ldrakeRBM
    REQUIRES
    VERSION 0.0.1)
add_subdirectory(test)

