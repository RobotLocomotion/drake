add_library_with_exports(LIB_NAME drakeRBSystem SOURCE_FILES RigidBodySystem.cpp)
target_link_libraries(drakeRBSystem drakeRBM drakeOptimization drakeRigidBodyConstraint drakeIK)

drake_install_headers(RigidBodySystem.h)
drake_install_libraries(drakeRBSystem)
drake_install_pkg_config_file(drake-rbsystem
  TARGET drakeRBSystem
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRBSystem
  REQUIRES
    drake-ik
    drake-optimization
    drake-rbm
    drake-rigid-body-constraint
    eigen3)

if (lcm_FOUND)
  add_executable(rigidBodyLCMNode rigidBodyLCMNode.cpp)
  target_link_libraries(rigidBodyLCMNode gflags drakeRBSystem drakeLCMSystem Threads::Threads)
  drake_install_executables(TARGETS rigidBodyLCMNode)
  drake_install_headers(BotVisualizer.h)
endif ()

add_subdirectory(test)
