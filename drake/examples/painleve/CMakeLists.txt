if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakePainleve SOURCE_FILES
  painleve.cc
  )
target_link_libraries(drakePainleve
  drakeCommon
  drakeSystemFramework
  drakeOptimization
  )
drake_install_libraries(drakePainleve)
drake_install_headers(
  painleve-inl.h
  painleve.h
  )
drake_install_pkg_config_file(drake-painleve
  TARGET drakePainleve
  LIBS -ldrakePainleve
  REQUIRES
    drake-common
    drake-system-framework)

