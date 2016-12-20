add_library(drakeSystem1 INTERFACE)
target_include_directories (drakeSystem1 INTERFACE
  $<INSTALL_INTERFACE:${DRAKE_INCLUDE_DIR}>)
target_link_libraries(drakeSystem1 INTERFACE Eigen3::Eigen)
drake_install_pkg_config_file(drake-system1
  REQUIRES eigen3)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES
    LCMSystem.h
    LCMSystem.cpp
    )
  target_link_libraries(drakeLCMSystem
    drakeLCMTypes
    Eigen3::Eigen
    lcm)

  drake_install_libraries(drakeLCMSystem)
  drake_install_pkg_config_file(drake-lcm-system
    TARGET drakeLCMSystem
    LIBS -ldrakeLCMSystem
    REQUIRES
      eigen3
      drake-lcmtypes-cpp
      lcm)
endif()

add_library_with_exports(LIB_NAME drakeRBSystem SOURCE_FILES
  BotVisualizer.h
  RigidBodySystem.cpp
  RigidBodySystem.h
  Simulation.h
  System.h
  simulation_options.h
  vector.h
  )
target_link_libraries(drakeRBSystem
    drakeIK
    drakeMultibodyParsers
    drakeOptimization
    drakeRBM
    drakeRigidBodyConstraint
    Eigen3::Eigen
    spruce)

drake_install_libraries(drakeRBSystem)
drake_install_pkg_config_file(drake-rbsystem
  TARGET drakeRBSystem
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRBSystem
  REQUIRES
    drake-ik
    drake-multibody-parsers
    drake-optimization
    drake-rbm
    drake-rigid-body-constraint
    eigen3
    spruce)

if (lcm_FOUND)
  add_executable(rigidBodyLCMNode rigidBodyLCMNode.cpp)
  target_link_libraries(rigidBodyLCMNode
      Threads::Threads
      drakeLCMSystem
      drakeRBSystem
      gflags)
  drake_install_executables(TARGETS rigidBodyLCMNode)
endif ()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
