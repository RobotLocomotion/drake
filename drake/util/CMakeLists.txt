add_library(drakeUtil OBJECT drakeUtil.cpp)
drake_install_headers(drakeUtil.h)

drake_install_headers(drakeAppUtil.h)

if(lcm_FOUND)
  # note: requires drakeUtil header file, but not linking against it because
  # we only need methods that are templated and defined in the header

  add_library_with_exports(LIB_NAME drakeLCMUtil SOURCE_FILES lcmUtil.cpp)
  target_link_libraries(drakeLCMUtil
    drakeLCMTypes
    drakeTrajectories
    lcmtypes_bot2-core-cpp)
  drake_install_headers(lcmUtil.h)
  pods_install_libraries(drakeLCMUtil)
  pods_install_pkg_config_file(drake-lcm-util
    LIBS -ldrakeLCMUtil
    REQUIRES
    VERSION 0.0.1)
endif()

drake_install_headers(drakeGradientUtil.h)

add_library_with_exports(LIB_NAME drakeGeometryUtil SOURCE_FILES drakeGeometryUtil.cpp)
target_link_libraries(drakeGeometryUtil drakeCommon drakeMath)
pods_install_libraries(drakeGeometryUtil)
drake_install_headers(drakeGeometryUtil.h)
pods_install_pkg_config_file(drake-geometry-util
  LIBS -ldrakeGeometryUtil -ldrakeCommon
  REQUIRES
  VERSION 0.0.1)

add_library_with_exports(LIB_NAME drakeConvexHull SOURCE_FILES convexHull.cpp)
pods_install_libraries(drakeConvexHull)
drake_install_headers(convexHull.h)
pods_install_pkg_config_file(drake-convex-hull
  LIBS -ldrakeConvexHull
  REQUIRES
  VERSION 0.0.1)

add_subdirectory(yaml)
add_subdirectory(test)
