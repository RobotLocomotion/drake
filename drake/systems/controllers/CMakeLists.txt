include_directories(${PROJECT_SOURCE_DIR}/systems/robotInterfaces)
include_directories(${PROJECT_SOURCE_DIR}/util/test)

add_library(drakeControlUtil SHARED controlUtil.cpp)
target_link_libraries(drakeControlUtil drakeRBM drakeUtil drakeTrajectories)
pods_install_libraries(drakeControlUtil)
pods_install_headers(controlUtil.h DESTINATION drake)
pods_install_pkg_config_file(drake-control-util
  LIBS -ldrakeControlUtil
  REQUIRES
  VERSION 0.0.1)

pods_find_pkg_config(gurobi)
if (gurobi_FOUND AND LCM_FOUND)
  include_directories (${PROJECT_SOURCE_DIR}/systems/trajectories )
  include_directories (${PROJECT_SOURCE_DIR}/systems/plants )
  include_directories (${PROJECT_SOURCE_DIR}/solvers )
  add_library(drakeQPCommon SHARED QPCommon.cpp)
  target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil drakeSide)
  pods_use_pkg_config_packages(drakeQPCommon lcm)
  pods_install_libraries(drakeQPCommon)
  pods_install_headers(QPCommon.h DESTINATION drake)
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
  include_directories (${PROJECT_SOURCE_DIR}/util )
  include_directories (${PROJECT_SOURCE_DIR}/systems/trajectories )
  include_directories (${PROJECT_SOURCE_DIR}/systems/plants )
  include_directories (${PROJECT_SOURCE_DIR}/systems/plants/joints )
  include_directories (${PROJECT_SOURCE_DIR}/solvers )
  include_directories (${PROJECT_SOURCE_DIR}/solvers/qpSpline )
  pods_find_pkg_config(gurobi)

  if (gurobi_FOUND AND LCM_FOUND)
    # NOTE: controlMexUtil.h currently includes QPCommon.h, which includes a gurobi header.
    # drakeControlMexUtil also has LCM as a dependency.
    # Lots of things depend on drakeControlMexUtil. We should consider reducing dependencies

    add_mex(drakeControlMexUtil SHARED controlMexUtil.cpp)
    target_link_libraries(drakeControlMexUtil drakeRBM drakeMexUtil drakeTrajectories drakeControlUtil drakeLCMUtil)
    pods_install_libraries(drakeControlMexUtil)
    pods_install_headers(controlMexUtil.h DESTINATION drake)
    pods_install_pkg_config_file(drake-control-mex-util
      LIBS -ldrakeControlMexUtil
      REQUIRES
      VERSION 0.0.1)

    add_mex(QPControllermex QPControllermex.cpp)
    target_link_libraries(QPControllermex drakeQP drakeControlUtil drakeMexUtil drakeControlMexUtil)
    pods_use_pkg_config_packages(QPControllermex gurobi)

    add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
    target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeSide)
    pods_use_pkg_config_packages(instantaneousQPControllermex gurobi)

    add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
    target_link_libraries(constructQPDataPointerMex drakeQP drakeControlUtil drakeQPCommon drakeMexUtil drakeControlMexUtil)
    pods_use_pkg_config_packages(constructQPDataPointerMex gurobi)

    add_mex(encodeQPInputLCMMex encodeQPInputLCMMex.cpp)
    target_link_libraries(encodeQPInputLCMMex drakeControlUtil drakeQPCommon drakeMexUtil drakeControlMexUtil)
    pods_use_pkg_config_packages(encodeQPInputLCMMex lcm)
    
    macro(add_control_mex)
      add_mex(${ARGV} ${ARGV}.cpp)
      target_link_libraries(${ARGV} drakeControlUtil drakeMexUtil drakeRBM drakeControlMexUtil)
    endmacro()
    
    add_control_mex(bodyMotionControlmex)
  endif()

  add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil) 

endif(MATLAB_FOUND)

add_library(drakeZMPUtil zmpUtil.cpp)
target_link_libraries(drakeZMPUtil drakeTrajectories)
pods_install_libraries(drakeZMPUtil)
pods_install_headers(zmpUtil.h DESTINATION drake)


add_subdirectory(test)
