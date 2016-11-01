drake_install_headers(
  LQR.h)

add_library_with_exports(LIB_NAME drakeControlUtil SOURCE_FILES controlUtil.cpp)
target_link_libraries(drakeControlUtil drakeRBM drakeTrajectories)
pods_install_libraries(drakeControlUtil)
drake_install_headers(controlUtil.h)
pods_install_pkg_config_file(drake-control-util
  LIBS -ldrakeControlUtil
  REQUIRES
  VERSION 0.0.1)

drake_find_package(gurobi PKG_CONFIG)
drake_find_package(yaml-cpp CONFIG)
if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeQPCommon SOURCE_FILES InstantaneousQPController.cpp)
  target_link_libraries(drakeQPCommon drakeControlUtil drakeQP drakeTrajectories drakeLCMUtil drakeSide drakeYAMLUtil lcm)
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

pods_install_libraries(drakeSystemControllers)
pods_install_pkg_config_file(drake-system2-controllers
    LIBS -ldrakeSystem2Controllers -ldrakeSystemFramework -ldrakeRBM
    REQUIRES
    VERSION 0.0.1)
add_subdirectory(test)
