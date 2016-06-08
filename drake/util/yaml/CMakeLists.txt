pods_find_pkg_config(yaml-cpp)
if(yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeYAMLUtil SOURCE_FILES yamlUtil.cpp)
  pods_use_pkg_config_packages(drakeYAMLUtil yaml-cpp)
  target_link_libraries(drakeYAMLUtil drakeRBM drakeSide)
  pods_install_libraries(drakeYAMLUtil)
  drake_install_headers(yamlUtil.h)
  pods_install_pkg_config_file(drake-yaml-util
    LIBS -ldrakeYAMLUtil
    REQUIRES yaml-cpp
    VERSION 0.0.1)
endif()
