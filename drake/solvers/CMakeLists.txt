set(solver_headers
  dreal_solver.h
  gurobi_solver.h
  ipopt_solver.h
  linear_system_solver.h
  moby_lcp_solver.h
  mosek_solver.h
  nlopt_solver.h
  snopt_solver.h
  solver_id.h
  solver_type_converter.h
  )
set(optimization_files)
set(optimization_requires)
list(APPEND optimization_files
  ${solver_headers}
  constraint.cc
  cost.cc
  create_constraint.cc
  create_cost.cc
  decision_variable.cc
  dreal_solver_common.cc
  equality_constrained_qp_solver.cc
  gurobi_solver_common.cc
  indeterminate.cc
  ipopt_solver_common.cc
  linear_system_solver.cc
  mathematical_program.cc
  mathematical_program_api.cc
  mathematical_program_solver_interface.cc
  moby_lcp_solver.cc
  mosek_solver_common.cc
  nlopt_solver_common.cc
  no_dreal.cc
  no_gurobi.cc
  no_mosek.cc
  no_snopt.cc
  snopt_solver_common.cc
  solver_id.cc
  solver_type_converter.cc
  symbolic_extraction.cc
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

add_library_with_exports(LIB_NAME drakeOptimization SOURCE_FILES ${optimization_files})
target_link_libraries(drakeOptimization
  drakeCommon
  Eigen3::Eigen)
drake_install_headers(
  ${solver_headers}
  binding.h
  constraint.h
  cost.h
  create_constraint.h
  create_cost.h
  function.h
  mathematical_program.h
  mathematical_program_solver_interface.h
  decision_variable.h
  indeterminate.h
  symbolic_extraction.h
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

add_library_with_exports(LIB_NAME drakeMixedIntegerOptimizationUtil SOURCE_FILES mixed_integer_optimization_util.cc)
target_link_libraries(drakeMixedIntegerOptimizationUtil
        drakeMath
        drakeOptimization)
drake_install_headers(
        mixed_integer_optimization_util.h
)
drake_install_pkg_config_file(drake-mixed-integer-optimization-util
        TARGET drakeMixedIntegerOptimizationUtil
        LIBS -ldrakeMixedIntegerOptimizationUtil
        REQUIRES
        drake-math
        drake-optimization)

add_library_with_exports(LIB_NAME drakeBilinearProductUtil SOURCE_FILES bilinear_product_util.cc)
target_link_libraries(drakeBilinearProductUtil
        drakeCommon)
drake_install_headers(
        bilinear_product_util.h
)
drake_install_pkg_config_file(drake-bilinear-product-util
        TARGET drakeBilinearProductUtil
        LIBS -ldrakeBilinearProductUtil
        REQUIRES
        drake-common)

add_library_with_exports(LIB_NAME drakeRotationConstraint SOURCE_FILES rotation_constraint.cc)
target_link_libraries(drakeRotationConstraint
        drakeOptimization
        drakeMixedIntegerOptimizationUtil
        drakeBilinearProductUtil)
drake_install_headers(
        rotation_constraint.h
)
drake_install_pkg_config_file(drake-rotation-constraint
        TARGET drakeRotationConstraint
        LIBS -ldrakeRotationConstraint
        REQUIRES
        drake-bilinear-product-util
        drake-common
        drake-mixed-integer-optimization-util
        drake-optimization)

drake_install_libraries(drakeRotationConstraint)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
