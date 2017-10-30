add_library(drakeUtil drakeUtil.h)
set_target_properties(drakeUtil PROPERTIES LINKER_LANGUAGE CXX)
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
