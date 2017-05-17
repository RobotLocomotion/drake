if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeRod2D SOURCE_FILES
  rod2d.cc
  )
target_link_libraries(drakeRod2D
  drakeCommon
  drakeRendering
  drakeSystemFramework
  drakeOptimization
  )
drake_install_libraries(drakeRod2D)
drake_install_headers(
  rod2d-inl.h
  rod2d.h
  )
drake_install_pkg_config_file(drake-rod2d
  TARGET drakeRod2D
  LIBS -ldrakeRod2D
  REQUIRES
    drake-common
    drake-system-framework)

