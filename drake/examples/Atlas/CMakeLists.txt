add_library_with_exports(LIB_NAME drakeAtlasUtil SOURCE_FILES atlasUtil.cpp)
pods_install_libraries(drakeAtlasUtil)
target_link_libraries(drakeAtlasUtil drakeYAMLUtil)
drake_install_headers(atlasUtil.h)
pods_install_pkg_config_file(drake-atlas-util
  LIBS -ldrakeAtlasUtil
  REQUIRES 
  VERSION 0.0.1)
