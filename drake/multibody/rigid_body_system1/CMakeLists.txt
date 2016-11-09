add_library_with_exports(LIB_NAME drakeRBSystem SOURCE_FILES RigidBodySystem.cpp)
target_link_libraries(drakeRBSystem drakeRBM drakeOptimization drakeRigidBodyConstraint drakeIK)

drake_install_headers(RigidBodySystem.h)
pods_install_libraries(drakeRBSystem)
pods_install_pkg_config_file(drake-rbsystem
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRBSystem -ldrakeRBM -ldrakeOptimization -ldrakeRigidBodyConstraint
  -ldrakeIK
  VERSION 0.0.1)

if (lcm_FOUND)
  add_executable(rigidBodyLCMNode rigidBodyLCMNode.cpp)
  target_link_libraries(rigidBodyLCMNode gflags drakeRBSystem drakeLCMSystem Threads::Threads)
  pods_install_executables(rigidBodyLCMNode)
  drake_install_headers(BotVisualizer.h)
endif ()

add_subdirectory(test)
