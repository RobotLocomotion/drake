if (MATLAB_FOUND)
  include_directories (${PROJECT_SOURCE_DIR}/util )
  include_directories (${PROJECT_SOURCE_DIR}/systems/trajectories )
  include_directories (${PROJECT_SOURCE_DIR}/solvers/polynomial )
  include_directories (${PROJECT_SOURCE_DIR}/solvers/qpSpline )
  pods_find_pkg_config(gurobi)

  pods_find_pkg_config(terrain-map)

  add_mex(drakeControlUtil SHARED controlUtil.cpp)
  if (terrain-map_FOUND)
    pods_use_pkg_config_packages(drakeControlUtil terrain-map)
    set_target_properties(drakeControlUtil PROPERTIES COMPILE_FLAGS "-DUSE_MAPS")
  endif()
  target_link_libraries(drakeControlUtil drakeRBM drakeUtil drakeTrajectories)
  pods_install_libraries(drakeControlUtil)
  pods_install_headers(controlUtil.h DESTINATION drake)
  pods_install_pkg_config_file(drake-control-util
    LIBS -ldrakeControlUtil
    REQUIRES
    VERSION 0.0.1)

  if (gurobi_FOUND)
    add_mex(drakeQPCommon SHARED QPCommon.cpp Side.cpp BodyMotionData.cpp)
    target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil)
    pods_install_libraries(drakeQPCommon)
    pods_install_headers(QPCommon.h DESTINATION drake)
    pods_install_pkg_config_file(drake-qp-common
      LIBS -ldrakeQPCommon
      CFLAGS -I\${prefix}/include/lcmtypes
      REQUIRES drake-control-util
      VERSION 0.0.1)

    add_mex(QPControllermex QPControllermex.cpp)
    target_link_libraries(QPControllermex drakeQP drakeControlUtil)
    pods_use_pkg_config_packages(QPControllermex gurobi)

    add_mex(getActiveSupportsmex getActiveSupportsmex.cpp)
    target_link_libraries(getActiveSupportsmex drakeQP drakeControlUtil)

    if (LCM_FOUND)
      add_mex(instantaneousQPControllermex instantaneousQPControllermex.cpp)
      target_link_libraries(instantaneousQPControllermex drakeQP drakeControlUtil drakeQPCommon)
      pods_use_pkg_config_packages(instantaneousQPControllermex gurobi)

      add_mex(constructQPDataPointerMex constructQPDataPointerMex.cpp)
      target_link_libraries(constructQPDataPointerMex drakeQP drakeControlUtil drakeQPCommon)
      pods_use_pkg_config_packages(constructQPDataPointerMex gurobi drake-qp-common)

      include_directories (${CMAKE_BINARY_DIR}/lcmgen/lcmtypes )
      add_mex(encodeQPInputLCMMex encodeQPInputLCMMex.cpp)
      target_link_libraries(encodeQPInputLCMMex lcm drakeControlUtil drakeQPCommon)

      add_mex(drakeQPLocomotionPlan SHARED QPLocomotionPlan.cpp)
      target_link_libraries(drakeQPLocomotionPlan drakeQPCommon drakeSplineGeneration)
      pods_install_libraries(drakeQPLocomotionPlan)
      pods_install_headers(QPLocomotionPlan.h DESTINATION drake)
      pods_install_pkg_config_file(drake-qp-locomotion-plan
      LIBS -ldrakeQPLocomotionPlan
      CFLAGS -I\${prefix}/include/lcmtypes
      REQUIRES drake-qp-common
      VERSION 0.0.1)


    endif()
  endif()

  add_mex(bodyMotionControlmex bodyMotionControlmex.cpp)
  target_link_libraries(bodyMotionControlmex drakeRBM drakeControlUtil)

  add_mex(pelvisMotionControlmex pelvisMotionControlmex.cpp)
  target_link_libraries(pelvisMotionControlmex drakeRBM drakeUtil drakeControlUtil)

  add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeControlUtil)

  add_mex(individualCentersOfPressuremex individualCentersOfPressuremex.cpp)
  include_directories (${CMAKE_SOURCE_DIR}/util )
  target_link_libraries(individualCentersOfPressuremex drakeRBM drakeUtil drakeControlUtil)


endif(MATLAB_FOUND)

add_subdirectory(test)
