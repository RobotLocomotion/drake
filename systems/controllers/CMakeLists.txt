
# IMPORTANT NOTE!! 
# matlab has it's own boost libraries.  DO NOT let any mex file depend
# on the system boost (directly nor indirectly), or you're asking for trouble.
find_package( Boost COMPONENTS filesystem system regex)

include_directories (${CMAKE_SOURCE_DIR}/util )

if (eigen3_FOUND)
  # todo: check eigen version.  3.1.0 didn't work for clang.  3.2.0 did.
  if (gurobi_FOUND)
    add_mex(QPControllermex QPControllermex.cpp)
    pods_use_pkg_config_packages(QPControllermex eigen3 gurobi drake-qp drake-rbm)
    target_link_libraries(QPControllermex drakeRBM drakeUtil)
  endif(gurobi_FOUND)

endif(eigen3_FOUND)
  