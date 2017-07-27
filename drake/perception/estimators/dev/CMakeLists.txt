add_library_with_exports(LIB_NAME drakeArticulatedIcp SOURCE_FILES
  scene.cc articulated_icp.cc)
target_link_libraries(drakeArticulatedIcp
  drakeRBM
  drakeOptimization
  Eigen3::Eigen)
drake_install_libraries(drakeArticulatedIcp)
drake_install_headers(
  scene.h articulated_icp.h)
drake_install_pkg_config_file(drake-articulated-icp
  TARGET drakeArticulatedIcp
  LIBS -ldrakeArticulatedIcp
  REQUIRES drake-rbm drake-optimization eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
