add_library(drakeAtlasUtil SHARED atlasUtil.cpp)
pods_install_libraries(drakeAtlasUtil)
pods_install_headers(atlasUtil.h DESTINATION drake)
pods_install_pkg_config_file(drake-atlas-util
  LIBS -ldrakeAtlasUtil
  REQUIRES 
  VERSION 0.0.1)
