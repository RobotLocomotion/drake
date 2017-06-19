add_library_with_exports(LIB_NAME drakeAtlasUtil SOURCE_FILES atlasUtil.cpp)
target_link_libraries(drakeAtlasUtil Eigen3::Eigen)
drake_install_libraries(drakeAtlasUtil)
drake_install_headers(
        atlasUtil.h
        )
drake_install_pkg_config_file(drake-atlas-util
  TARGET drakeAtlasUtil
  LIBS -ldrakeAtlasUtil
  REQUIRES eigen3)
