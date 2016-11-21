
add_library_with_exports(LIB_NAME drakeLinearControl SOURCE_FILES linear_optimal_control.cc)
target_link_libraries(drakeLinearControl drakeSystemFramework)
drake_install_libraries(drakeLinearControl)
drake_install_headers(linear_optimal_control.h)
drake_install_pkg_config_file(drake-linear-control
  TARGET drakeLinearControl
  LIBS -ldrakeLinearControl)

add_library_with_exports(LIB_NAME drakeControlUtil SOURCE_FILES controlUtil.cpp)
target_link_libraries(drakeControlUtil drakeRBM drakeTrajectories)
drake_install_libraries(drakeControlUtil)
drake_install_headers(controlUtil.h)
drake_install_pkg_config_file(drake-control-util
  TARGET drakeControlUtil
  LIBS -ldrakeControlUtil
  REQUIRES eigen3)

if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeQPCommon SOURCE_FILES InstantaneousQPController.cpp)
  target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil drakeSide drakeYAMLUtil lcm)
  drake_install_libraries(drakeQPCommon)
  drake_install_headers(QPCommon.h InstantaneousQPController.h)
  drake_install_pkg_config_file(drake-qp-common
    TARGET drakeQPCommon
    LIBS -ldrakeQPCommon
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES
      drake-control-util
      drake-lcm-util
      drake-qp
      drake-side
      drake-trajectories
      drake-yaml-util
      eigen3
      lcm)

  pods_find_pkg_config(bot2-lcmgl-client)
  if(bot2-lcmgl-client_FOUND)
    add_executable(zmpCoMObserverStateVisualizer zmpCoMObserverStateVisualizer.cpp)
    pods_use_pkg_config_packages(zmpCoMObserverStateVisualizer bot2-lcmgl-client)
    target_link_libraries(zmpCoMObserverStateVisualizer drake_lcmtypes lcm)
  endif()
endif()

add_library_with_exports(LIB_NAME drakeZMPUtil SOURCE_FILES zmpUtil.cpp)
target_link_libraries(drakeZMPUtil drakeTrajectories)
drake_install_libraries(drakeZMPUtil)
drake_install_headers(zmpUtil.h)
drake_install_pkg_config_file(drake-zmp-util
  TARGET drakeZMPUtil
  LIBS -ldrakeZMPUtil
  REQUIRES eigen3)

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
add_dependencies(drakeSystemControllers drakeCommon drakeRBM drakeSystemFramework)
target_link_libraries(drakeSystemControllers
    drakeCommon
    drakeRBM
    drakeSystemFramework)

drake_install_headers(${system2_installed_headers})

drake_install_libraries(drakeSystemControllers)
drake_install_pkg_config_file(drake-system2-controllers
    TARGET drakeSystemControllers
    LIBS -ldrakeSystemControllers
    REQUIRES
      drake-rbm
      drake-system-framework)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
