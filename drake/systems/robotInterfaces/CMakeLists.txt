add_library_with_exports(LIB_NAME drakeSide SOURCE_FILES Side.cpp)
drake_install_headers(Side.h)
drake_install_libraries(drakeSide)
drake_install_pkg_config_file(drake-side
  TARGET drakeSide
  LIBS -ldrakeSide)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeQPLocomotionPlan SOURCE_FILES QPLocomotionPlan.cpp BodyMotionData.cpp) # TODO: move BodyMotionData to a better place
  target_link_libraries(drakeQPLocomotionPlan
    drakeAtlasUtil
    drakeConvexHull
    drakeLCMUtil
    drakeRBM
    drakeSide
    drakeSplineGeneration
    drakeZMPUtil
    Eigen3::Eigen
    lcm)
  drake_install_libraries(drakeQPLocomotionPlan)
  drake_install_headers(QPLocomotionPlan.h BodyMotionData.h)
  drake_install_pkg_config_file(drake-qp-locomotion-plan
    TARGET drakeQPLocomotionPlan
    LIBS -ldrakeQPLocomotionPlan
    CFLAGS -I\${prefix}/include/lcmtypes
    REQUIRES
      drake-atlas-util
      drake-convex-hull
      drake-lcm-util
      drake-rbm
      drake-side
      drake-spline-generation
      drake-zmp-util
      eigen3
      lcm)
endif()
