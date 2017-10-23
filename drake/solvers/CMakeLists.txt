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
list(APPEND optimization_files
  ${solver_headers}
  constraint.cc
  cost.cc
  create_constraint.cc
  create_cost.cc
  decision_variable.cc
  dreal_solver_common.cc
  equality_constrained_qp_solver.cc
  evaluator_base.cc
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
  nlopt_solver.cc
  no_dreal.cc
  no_gurobi.cc
  no_ipopt.cc
  no_mosek.cc
  no_snopt.cc
  snopt_solver_common.cc
  solver_id.cc
  solver_type_converter.cc
  symbolic_extraction.cc
  system_identification.cc
  )

add_library(drakeOptimization ${optimization_files})
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
  evaluator_base.h
  function.h
  mathematical_program.h
  mathematical_program_solver_interface.h
  decision_variable.h
  indeterminate.h
  symbolic_extraction.h
  system_identification.h
  )
drake_install_libraries(drakeOptimization)

# TODO(jamiesnape): Fix imported target upstream.
target_include_directories(drakeOptimization PUBLIC ${NLOPT_INCLUDE_DIRS})
target_link_libraries(drakeOptimization nlopt)
