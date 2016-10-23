drake_find_package(gurobi PKG_CONFIG)

if(gurobi_FOUND)
  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES qp.cc)
  target_include_directories(drakeQP PUBLIC ${gurobi_INCLUDE_DIRS})
  target_link_libraries(drakeQP ${gurobi_LIBRARIES})

  pods_install_libraries(drakeQP)
  drake_install_headers(fast_qp.h gurobi_qp.h)
  pods_install_pkg_config_file(drake-qp
    LIBS -ldrakeQP
    REQUIRES gurobi
    VERSION 0.0.1)
endif()

set(optimization_files)
list(APPEND optimization_files
  mathematical_program.cc moby_lcp_solver.cc
  system_identification.cpp linear_system_solver.cc
  equality_constrained_qp_solver.cc)
pods_find_pkg_config(ipopt)
if (ipopt_FOUND)
 list(APPEND optimization_files ipopt_solver.cc)
else()
  list(APPEND optimization_files no_ipopt.cc)
endif(ipopt_FOUND)
drake_find_package(NLopt CONFIG)
if(NLopt_FOUND)
  list(APPEND optimization_files nlopt_solver.cc)
else()
  list(APPEND optimization_files no_nlopt.cc)
endif()
pods_find_pkg_config(dreal)
if(dreal_FOUND)
  list(APPEND optimization_files dreal_solver.cc)
else()
  list(APPEND optimization_files no_dreal.cc)
endif()
drake_find_package(snopt_c CONFIG)
if(snopt_c_FOUND)
  list(APPEND optimization_files snopt_solver.cc)
else()
  list(APPEND optimization_files no_snopt.cc)
endif()
drake_find_package(mosek PKG_CONFIG)
if(mosek_FOUND)
  list(APPEND optimization_files mosek_solver.cc)
else()
  list(APPEND optimization_files no_mosek.cc)
endif()
if(gurobi_FOUND)
  list(APPEND optimization_files gurobi_solver.cc)
else()
  list(APPEND optimization_files no_gurobi.cc)
endif()


add_library_with_exports(LIB_NAME drakeOptimization SOURCE_FILES ${optimization_files})
target_link_libraries(drakeOptimization drakeCommon)
drake_install_headers(
  constraint.h
  Function.h
  mathematical_program.h
  decision_variable.h
  solution_result.h
  system_identification.h
  )
pods_install_libraries(drakeOptimization)
pods_install_pkg_config_file(drake-optimization
  LIBS -ldrakeOptimization
  REQUIRES
  VERSION 0.0.1)

if(ipopt_FOUND)
  pods_use_pkg_config_packages(drakeOptimization ipopt)
  # IPOPT builds its own version of BLAS during compilation, and links
  # it into the output library.  On (at least some) ELF systems, you
  # wind up with a symbol name collision between IPOPT's BLAS and
  # other BLAS implementations in other dynamic libraries (MATLAB, I'm
  # looking in your direction...)  Since the actual implementations
  # are incompatible, this does not end well.  Linking with the
  # -Bsymbolic flags causes the internal references to these symbols
  # inside libdrakeOptimization to be handled at compile time, and
  # prevents the dynamic linker from hijacking IPOPT's calls into
  # BLAS.
  #
  # This should probably be a test for the GNU linker instead of a platform.
  if (NOT (APPLE OR WIN32))
    set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic -Wl,-Bsymbolic-functions")
  endif()
endif()
if(NLopt_FOUND)
  target_link_libraries(drakeOptimization nlopt)
endif()
if(dreal_FOUND)
  pods_use_pkg_config_packages(drakeOptimization dreal)
endif()
if(snopt_c_FOUND)
  target_link_libraries(drakeOptimization snopt_c)
endif()
if(gurobi_FOUND)
  target_include_directories(drakeOptimization PUBLIC ${gurobi_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization ${gurobi_LIBRARIES})
endif()
if(mosek_FOUND)
  target_include_directories(drakeOptimization PUBLIC ${mosek_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization ${mosek_LIBRARIES})
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
add_subdirectory(trajectoryOptimization)
