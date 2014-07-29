if (eigen3_FOUND)
  pods_find_pkg_config(gurobi)

  add_mex(drakeControlUtil SHARED controlUtil.cpp)
  target_link_libraries(drakeControlUtil drakeRBM drakeUtil)

  if (gurobi_FOUND)
    add_mex(QPControllermex QPControllermex.cpp)
    target_link_libraries(QPControllermex drakeQP drakeControlUtil)
    pods_use_pkg_config_packages(QPControllermex gurobi)
  endif()

  add_mex(supportDetectmex supportDetectmex.cpp)
  target_link_libraries(supportDetectmex drakeRBM drakeUtil drakeControlUtil)

endif(eigen3_FOUND)
