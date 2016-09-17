if(MATLAB_FOUND)
  add_mex(PPTmex PPTmex.cpp)
  add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)
endif()

add_library_with_exports(LIB_NAME drakeTrajectories SOURCE_FILES PiecewiseFunction.cpp PiecewisePolynomialBase.cpp PiecewisePolynomial.cpp ExponentialPlusPiecewisePolynomial.cpp)
target_link_libraries(drakeTrajectories drakeCommon)
pods_install_libraries(drakeTrajectories)
drake_install_headers(PiecewiseFunction.h PiecewisePolynomial.h PiecewisePolynomialBase.h ExponentialPlusPiecewisePolynomial.h)
pods_install_pkg_config_file(drake-trajectories
  LIBS -ldrakeTrajectories
  REQUIRES
  VERSION 0.0.3)

pods_find_pkg_config(snopt_c)
find_package(Bullet)
find_library(BLAS_LIBRARY NAMES blas libblas)
if(BLAS_LIBRARY AND snopt_c_FOUND AND BULLET_FOUND AND MATLAB_FOUND)
  include_directories(${BULLET_INCLUDE_DIR})  
  add_mex(replanFunnels_mex replanFunnels_mex.cpp)
  add_mex(isCollisionFree_mex isCollisionFree_mex.cpp)
  add_mex(ptToPolyBullet_mex ptToPolyBullet_mex.cpp)
  add_mex(shiftFunnel_snopt_mex shiftFunnel_snopt_mex.cpp)
  pods_use_pkg_config_packages(replanFunnels_mex snopt_c)
  pods_use_pkg_config_packages(shiftFunnel_snopt_mex snopt_c)
  target_link_libraries(replanFunnels_mex -lblas drakeMexUtil ${BULLET_LIBRARIES})
  target_link_libraries(shiftFunnel_snopt_mex -lblas drakeMexUtil ${BULLET_LIBRARIES})
  target_link_libraries(isCollisionFree_mex drakeMexUtil ${BULLET_LIBRARIES})
  target_link_libraries(ptToPolyBullet_mex drakeMexUtil ${BULLET_LIBRARIES})

  # These warnings are not worth fixing.
  target_compile_options(replanFunnels_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
  target_compile_options(shiftFunnel_snopt_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
endif()

add_subdirectory(test)
