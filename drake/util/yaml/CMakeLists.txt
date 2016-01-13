include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../solvers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../systems)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../systems/robotInterfaces)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../systems/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../core)

add_library(drakeYAMLUtil SHARED yamlUtil.cpp)
pods_use_pkg_config_packages(drakeYAMLUtil yaml-cpp)
target_link_libraries(drakeYAMLUtil drakeRBM)
pods_install_libraries(drakeYAMLUtil)
pods_install_headers(yamlUtil.h DESTINATION drake)

add_executable(loadParams loadParams.cpp)
target_link_libraries(loadParams drakeYAMLUtil)

