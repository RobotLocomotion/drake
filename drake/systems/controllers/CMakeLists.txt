
add_library_with_exports(LIB_NAME drakeLinearControl SOURCE_FILES linear_quadratic_regulator.cc)
target_link_libraries(drakeLinearControl drakeMath drakeSystemFramework drakeSystemPrimitives)
drake_install_libraries(drakeLinearControl)
drake_install_headers(linear_quadratic_regulator.h)
drake_install_pkg_config_file(drake-linear-control
  TARGET drakeLinearControl
  LIBS -ldrakeLinearControl)

add_library_with_exports(LIB_NAME drakeZMPPlanner SOURCE_FILES zmp_planner.cc)
target_link_libraries(drakeZMPPlanner drakeLinearControl)
drake_install_libraries(drakeZMPPlanner)
drake_install_headers(zmp_planner.h)
drake_install_pkg_config_file(drake-zmp-planner
  TARGET drakeZMPPlanner
  LIBS -ldrakeZMPPlanner)

add_library_with_exports(LIB_NAME drakeControlUtil SOURCE_FILES controlUtil.cpp)
target_link_libraries(drakeControlUtil
  drakeRBM
  drakeTrajectories
  Eigen3::Eigen)
drake_install_libraries(drakeControlUtil)
drake_install_headers(controlUtil.h)
drake_install_pkg_config_file(drake-control-util
  TARGET drakeControlUtil
  LIBS -ldrakeControlUtil
  REQUIRES
    drake-rbm
    drake-trajectories
    eigen3)

if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeQPCommon SOURCE_FILES InstantaneousQPController.cpp)
  target_link_libraries(drakeQPCommon
      drakeControlUtil
      drakeLCMUtil
      drakeMultibodyParsers
      drakeQP
      drakeSide
      drakeTrajectories
      drakeYAMLUtil
      Eigen3::Eigen
      lcm)
  drake_install_libraries(drakeQPCommon)
  drake_install_headers(QPCommon.h InstantaneousQPController.h)
  drake_install_pkg_config_file(drake-qp-common
    TARGET drakeQPCommon
    LIBS -ldrakeQPCommon
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES
      drake-control-util
      drake-lcm-util
      drake-multibody-parsers
      drake-qp
      drake-side
      drake-trajectories
      drake-yaml-util
      eigen3
      lcm)

  if(bot2-lcmgl-client_FOUND)
    add_executable(zmpCoMObserverStateVisualizer zmpCoMObserverStateVisualizer.cpp)
    target_link_libraries(zmpCoMObserverStateVisualizer drake_lcmtypes bot2-lcmgl-client lcm)
  endif()
endif()

add_library_with_exports(LIB_NAME drakeZMPUtil SOURCE_FILES zmpUtil.cpp)
target_link_libraries(drakeZMPUtil
  drakeTrajectories
  Eigen3::Eigen)
drake_install_libraries(drakeZMPUtil)
drake_install_headers(zmpUtil.h)
drake_install_pkg_config_file(drake-zmp-util
  TARGET drakeZMPUtil
  LIBS -ldrakeZMPUtil
  REQUIRES
    drake-trajectories
    eigen3)

# System 2 controllers
set(system2_sources
  gravity_compensator.cc
  pid_controlled_system.cc
  pid_controller.cc)
set(system2_installed_headers
  gravity_compensator.h
  pid_controlled_system.h
  pid_controller.h)
add_library_with_exports(LIB_NAME drakeSystemControllers
    SOURCE_FILES ${system2_sources} ${system2_installed_headers})
add_dependencies(drakeSystemControllers drakeCommon drakeRBM drakeSystemFramework drakeSystemPrimitives)
target_link_libraries(drakeSystemControllers
    drakeCommon
    drakeRBM
    drakeSystemFramework
    drakeSystemPrimitives)

drake_install_headers(${system2_installed_headers})

drake_install_libraries(drakeSystemControllers)
drake_install_pkg_config_file(drake-system2-controllers
    TARGET drakeSystemControllers
    LIBS -ldrakeSystemControllers
    REQUIRES
      drake-rbm
      drake-system-framework
      drake-system-primitives)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
