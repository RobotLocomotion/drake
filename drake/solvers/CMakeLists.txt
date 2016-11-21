if(gurobi_FOUND)
  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES qp.cc)
  target_include_directories(drakeQP PUBLIC ${gurobi_INCLUDE_DIRS})
  target_link_libraries(drakeQP gurobi)

  drake_install_libraries(drakeQP)
  drake_install_headers(fast_qp.h gurobi_qp.h)
  drake_install_pkg_config_file(drake-qp
    TARGET drakeQP
    LIBS -ldrakeQP
    REQUIRES gurobi)
endif()

set(optimization_files)
set(optimization_requires)
list(APPEND optimization_files
  mathematical_program.cc moby_lcp_solver.cc
  system_identification.cc linear_system_solver.cc
  equality_constrained_qp_solver.cc)
pods_find_pkg_config(ipopt)
if (ipopt_FOUND)
 list(APPEND optimization_files ipopt_solver.cc)
 list(APPEND optimization_requires ipopt)
else()
  list(APPEND optimization_files no_ipopt.cc)
endif(ipopt_FOUND)
if(NLopt_FOUND)
  list(APPEND optimization_files nlopt_solver.cc)
  list(APPEND optimization_requires nlopt)
else()
  list(APPEND optimization_files no_nlopt.cc)
endif()
if(dreal_FOUND)
  list(APPEND optimization_files dreal_solver.cc)
  list(APPEND optimization_requires dreal)
else()
  list(APPEND optimization_files no_dreal.cc)
endif()
if(snopt_FOUND)
  list(APPEND optimization_files snopt_solver.cc)
  list(APPEND optimization_requires snopt_c)
else()
  list(APPEND optimization_files no_snopt.cc)
endif()
if(mosek_FOUND)
  list(APPEND optimization_files mosek_solver.cc)
  list(APPEND optimization_requires mosek)
else()
  list(APPEND optimization_files no_mosek.cc)
endif()
if(gurobi_FOUND)
  list(APPEND optimization_files gurobi_solver.cc)
  list(APPEND optimization_requires gurobi)
else()
  list(APPEND optimization_files no_gurobi.cc)
endif()


add_library_with_exports(LIB_NAME drakeOptimization SOURCE_FILES ${optimization_files})
target_link_libraries(drakeOptimization drakeCommon)
drake_install_headers(
  constraint.h
  function.h
  mathematical_program.h
  decision_variable.h
  solution_result.h
  system_identification.h
  )
drake_install_libraries(drakeOptimization)
drake_install_pkg_config_file(drake-optimization
  TARGET drakeOptimization
  LIBS -ldrakeOptimization
  REQUIRES
    eigen3
    ${optimization_requires})

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
  # TODO(jamiesnape): Fix imported target upstream.
  target_include_directories(drakeOptimization PUBLIC ${NLOPT_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization nlopt)
endif()
if(dreal_FOUND)
  pods_use_pkg_config_packages(drakeOptimization dreal)
endif()
if(snopt_FOUND)
  target_link_libraries(drakeOptimization snopt_c)
endif()
if(gurobi_FOUND)
  target_include_directories(drakeOptimization PUBLIC ${gurobi_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization gurobi)
endif()
if(mosek_FOUND)
  target_include_directories(drakeOptimization PUBLIC ${mosek_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization ${mosek_LIBRARIES})
endif()

add_subdirectory(qpSpline)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(trajectoryOptimization)
