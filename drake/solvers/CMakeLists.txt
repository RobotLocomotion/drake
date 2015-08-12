
# todo: still build something useful if gurobi does not exist

pods_find_pkg_config(gurobi)

if (gurobi_FOUND)

  add_library(drakeQP QP.cpp)
  set_target_properties(drakeQP PROPERTIES COMPILE_FLAGS -fPIC)
  pods_use_pkg_config_packages(drakeQP gurobi)

  if (MATLAB_FOUND)
    add_mex(fastQPmex fastQPmex.cpp)
    target_link_libraries(fastQPmex drakeQP)
    add_mex(gurobiQPmex gurobiQPmex.cpp)
    target_link_libraries(gurobiQPmex drakeQP)
  endif()

  pods_install_libraries(drakeQP)
  pods_install_headers(fastQP.h gurobiQP.h DESTINATION drake)
  pods_install_pkg_config_file(drake-qp
    LIBS -ldrakeQP
    REQUIRES gurobi
    VERSION 0.0.1)

endif()

pods_find_pkg_config(snopt_c)

if(MATLAB_FOUND)
  add_mex(lqrmex lqrmex.cpp)
  target_link_libraries(lqrmex drakeUtil)
endif()

if(snopt_c_FOUND AND MATLAB_FOUND)
  add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  pods_use_pkg_config_packages(NonlinearProgramSnoptmex snopt_c)
  set_target_properties(NonlinearProgramSnoptmex PROPERTIES COMPILE_FLAGS -fPIC)

  # link against matlab's copy of the gfortran library, if possible
  find_library(GFORTRAN_LIBRARY NAMES gfortran libgfortran
    PATHS "${MATLAB_ROOT}/sys/os/"
    PATH_SUFFIXES glnxa64/ maci64/ NO_DEFAULT_PATH )
  if (NOT GFORTRAN_LIBRARY)
    # the above does not seem work on linux out of the box, because matlab
    # symlinks to libgfortran.so.3 but not to libgfortran.so, and cmake isn't
    # able to find it with the default CMAKE_FIND_LIBRARY_SUFFIXES.  Instead
    # of setting that variable, here is the annoying work-around:
    find_file(GFORTRAN_LIBRARY NAMES libgfortran.so.3
      PATHS "${MATLAB_ROOT}/sys/os/"
      PATH_SUFFIXES glnxa64/ maci64/ NO_DEFAULT_PATH)
  endif()

  if (GFORTRAN_LIBRARY)
    target_link_libraries(NonlinearProgramSnoptmex ${GFORTRAN_LIBRARY})
  endif()
endif()

add_subdirectory(qpSpline)
