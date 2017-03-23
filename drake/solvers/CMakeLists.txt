if(gurobi_FOUND)
  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES qp.cc)
  target_link_libraries(drakeQP
    Eigen3::Eigen
    gurobi)

  drake_install_libraries(drakeQP)
  drake_install_headers(fast_qp.h gurobi_qp.h)
  drake_install_pkg_config_file(drake-qp
    TARGET drakeQP
    LIBS -ldrakeQP
    REQUIRES
      eigen3
      gurobi)
endif()

set(solver_headers
  dreal_solver.h
  gurobi_solver.h
  ipopt_solver.h
  linear_system_solver.h
  moby_lcp_solver.h
  mosek_solver.h
  nlopt_solver.h
  snopt_solver.h
  )
set(optimization_files)
set(optimization_requires)
list(APPEND optimization_files
  ${solver_headers}
  constraint.cc
  decision_variable.cc
  equality_constrained_qp_solver.cc
  linear_system_solver.cc
  mathematical_program.cc
  mathematical_program_solver_interface.cc
  moby_lcp_solver.cc
  no_dreal.cc
  system_identification.cc
  )
if(ipopt_FOUND)
 list(APPEND optimization_files ipopt_solver.cc)
 list(APPEND optimization_requires ipopt)
else()
  list(APPEND optimization_files no_ipopt.cc)
endif()
if(NLopt_FOUND)
  list(APPEND optimization_files nlopt_solver.cc)
  list(APPEND optimization_requires nlopt)
else()
  list(APPEND optimization_files no_nlopt.cc)
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
target_link_libraries(drakeOptimization
  drakeCommon
  Eigen3::Eigen)
drake_install_headers(
  ${solver_headers}
  binding.h
  constraint.h
  function.h
  mathematical_program.h
  mathematical_program_solver_interface.h
  decision_variable.h
  system_identification.h
  )
drake_install_libraries(drakeOptimization)
drake_install_pkg_config_file(drake-optimization
  TARGET drakeOptimization
  LIBS -ldrakeOptimization
  REQUIRES
    drake-common
    eigen3
    ${optimization_requires})

if(ipopt_FOUND)
  target_link_libraries(drakeOptimization ipopt)
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
  if(NOT APPLE)
    set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic -Wl,-Bsymbolic-functions")
  endif()
endif()
if(NLopt_FOUND)
  # TODO(jamiesnape): Fix imported target upstream.
  target_include_directories(drakeOptimization PUBLIC ${NLOPT_INCLUDE_DIRS})
  target_link_libraries(drakeOptimization nlopt)
endif()
if(snopt_FOUND)
  target_link_libraries(drakeOptimization snopt_c)
endif()
if(gurobi_FOUND)
  target_link_libraries(drakeOptimization gurobi)
endif()
if(mosek_FOUND)
  target_link_libraries(drakeOptimization mosek)
endif()

add_library_with_exports(LIB_NAME drakeRotationConstraint SOURCE_FILES rotation_constraint.cc)
target_link_libraries(drakeRotationConstraint
        drakeOptimization)
drake_install_headers(
        rotation_constraint.h
)
drake_install_pkg_config_file(drake-rotation-constraint
        TARGET drakeRotationConstraint
        LIBS -ldrakeRotationConstraint
        REQUIRES
        drake-common
        drake-optimization)

drake_install_libraries(drakeRotationConstraint)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
