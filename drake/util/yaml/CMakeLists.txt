if(yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeYAMLUtil SOURCE_FILES yamlUtil.cpp)
  target_link_libraries(drakeYAMLUtil drakeRBM drakeSide yaml-cpp)
  drake_install_libraries(drakeYAMLUtil)
  drake_install_headers(yamlUtil.h)
  drake_install_pkg_config_file(drake-yaml-util
    TARGET drakeYAMLUtil
    LIBS -ldrakeYAMLUtil
    REQUIRES
      drake-rbm
      drake-side
      yaml-cpp)
endif()
