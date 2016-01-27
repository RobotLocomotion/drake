add_library_with_exports(LIB_NAME drakeYAMLUtil SOURCE_FILES yamlUtil.cpp)
pods_use_pkg_config_packages(drakeYAMLUtil yaml-cpp)
target_link_libraries(drakeYAMLUtil drakeRBM)
pods_install_libraries(drakeYAMLUtil)
drake_install_headers(yamlUtil.h)

add_executable(loadParams loadParams.cpp)
target_link_libraries(loadParams drakeYAMLUtil)

