add_library(drakeUtil drakeUtil.cpp drakeUtil.h)
target_link_libraries(drakeUtil Eigen3::Eigen)
drake_install_libraries(drakeUtil)
drake_install_headers(drakeUtil.h)

add_library(drakeGeometryUtil drakeGeometryUtil.cpp)
target_link_libraries(drakeGeometryUtil
  drakeCommon
  drakeMath
  Eigen3::Eigen)
drake_install_libraries(drakeGeometryUtil)
drake_install_headers(drakeGeometryUtil.h)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
