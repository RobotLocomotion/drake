if (eigen3_FOUND)
  pods_find_pkg_config(gurobi)

  if (gurobi_FOUND)
    add_mex(QPControllermex QPControllermex.cpp)
    target_link_libraries(QPControllermex drakeRBM drakeUtil drakeQP)
    pods_use_pkg_config_packages(QPControllermex gurobi)
  endif()

endif(eigen3_FOUND)
  