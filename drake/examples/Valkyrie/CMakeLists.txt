add_library(drakeValkyrieUtil SHARED valkyrieUtil.cpp)
pods_install_libraries(drakeValkyrieUtil)
pods_install_headers(valkyrieUtil.h DESTINATION drake)
pods_install_pkg_config_file(drake-valkyrie-util
  LIBS -ldrakeValkyrieUtil
  REQUIRES 
  VERSION 0.0.1)
