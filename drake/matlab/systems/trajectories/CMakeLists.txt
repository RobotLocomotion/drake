if(Matlab_FOUND)
  drake_add_mex(PPTmex PPTmex.cpp)
  target_link_libraries(PPTmex Eigen3::Eigen)
  drake_add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)
  target_link_libraries(ExpPlusPPTrajectoryEvalmex Eigen3::Eigen)
endif()

if(Bullet_FOUND AND Matlab_FOUND)
  if(BLAS_FOUND AND snopt_FOUND)
    drake_add_mex(replanFunnels_mex replanFunnels_mex.cpp)
    target_include_directories(replanFunnels_mex PRIVATE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(replanFunnels_mex drakeMexUtil snopt_c
      ${BLAS_LIBRARIES} ${BULLET_LIBRARIES})

    drake_add_mex(shiftFunnel_snopt_mex shiftFunnel_snopt_mex.cpp)
    target_include_directories(shiftFunnel_snopt_mex PRIVATE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(shiftFunnel_snopt_mex drakeMexUtil snopt_c
      ${BLAS_LIBRARIES} ${BULLET_LIBRARIES})

    # Some platforms require extra link flags. On those that do not, the
    # BLAS_LINKER_FLAGS variable is undefined.
    if(DEFINED BLAS_LINKER_FLAGS)
      set_property(TARGET replanFunnels_mex APPEND_STRING PROPERTY
        LINK_FLAGS "${BLAS_LINKER_FLAGS}")
      set_property(TARGET shiftFunnel_snopt_mex APPEND_STRING PROPERTY
        LINK_FLAGS "${BLAS_LINKER_FLAGS}")
    endif()

    # These warnings are not worth fixing.
    target_compile_options(replanFunnels_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
    target_compile_options(shiftFunnel_snopt_mex PRIVATE ${CXX_FLAGS_NO_ERROR_SHADOW})
  endif()

  drake_add_mex(isCollisionFree_mex isCollisionFree_mex.cpp)
  target_include_directories(isCollisionFree_mex PRIVATE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(isCollisionFree_mex drakeMexUtil ${BULLET_LIBRARIES})

  drake_add_mex(ptToPolyBullet_mex ptToPolyBullet_mex.cpp)
  target_include_directories(ptToPolyBullet_mex PRIVATE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(ptToPolyBullet_mex drakeMexUtil ${BULLET_LIBRARIES})
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
