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

find_package(Bullet)

if(BULLET_FOUND AND MATLAB_FOUND)
  find_package(BLAS)
  pods_find_pkg_config(snopt_c)

  if(BLAS_FOUND AND snopt_c_FOUND)
    add_mex(replanFunnels_mex replanFunnels_mex.cpp)
    pods_use_pkg_config_packages(replanFunnels_mex snopt_c)
    target_include_directories(replanFunnels_mex PRIVATE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(replanFunnels_mex drakeMexUtil
      ${BLAS_LIBRARIES} ${BULLET_LIBRARIES})
    set_property(TARGET replanFunnels_mex APPEND_STRING PROPERTY
      LINK_FLAGS "${BLAS_LINKER_FLAGS}")

    add_mex(shiftFunnel_snopt_mex shiftFunnel_snopt_mex.cpp)
    pods_use_pkg_config_packages(shiftFunnel_snopt_mex snopt_c)
    target_include_directories(shiftFunnel_snopt_mex PRIVATE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(shiftFunnel_snopt_mex drakeMexUtil
      ${BLAS_LIBRARIES} ${BULLET_LIBRARIES})
    set_property(TARGET shiftFunnel_snopt_mex APPEND_STRING PROPERTY
      LINK_FLAGS "${BLAS_LINKER_FLAGS}")

    # These warnings are not worth fixing.
    target_compile_options(replanFunnels_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
    target_compile_options(shiftFunnel_snopt_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
  endif()

  add_mex(isCollisionFree_mex isCollisionFree_mex.cpp)
  target_include_directories(isCollisionFree_mex PRIVATE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(isCollisionFree_mex drakeMexUtil ${BULLET_LIBRARIES})

  add_mex(ptToPolyBullet_mex ptToPolyBullet_mex.cpp)
  target_include_directories(ptToPolyBullet_mex PRIVATE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(ptToPolyBullet_mex drakeMexUtil ${BULLET_LIBRARIES})
endif()

add_subdirectory(test)
