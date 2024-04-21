#pragma once

#include "drake/common/name_value.h"

// This file helps Drake's solvers/clarabel_solver.cc set the Clarabel options.
//
// It is committed to source control to simplify the build process, but can be
// mechanically regenerated by running the `gen_serialize` program. A linter
// checks that the committed code matches what would be regenerated.

namespace clarabel {

template <typename Archive>
// NOLINTNEXTLINE(runtime/references)
void Serialize(Archive* a, DefaultSettings<double>& settings) {
#define DRAKE_VISIT(x) a->Visit(drake::MakeNameValue(#x, &(settings.x)))
  DRAKE_VISIT(max_iter);
  DRAKE_VISIT(time_limit);
  DRAKE_VISIT(verbose);
  DRAKE_VISIT(max_step_fraction);
  DRAKE_VISIT(tol_gap_abs);
  DRAKE_VISIT(tol_gap_rel);
  DRAKE_VISIT(tol_feas);
  DRAKE_VISIT(tol_infeas_abs);
  DRAKE_VISIT(tol_infeas_rel);
  DRAKE_VISIT(tol_ktratio);
  DRAKE_VISIT(reduced_tol_gap_abs);
  DRAKE_VISIT(reduced_tol_gap_rel);
  DRAKE_VISIT(reduced_tol_feas);
  DRAKE_VISIT(reduced_tol_infeas_abs);
  DRAKE_VISIT(reduced_tol_infeas_rel);
  DRAKE_VISIT(reduced_tol_ktratio);
  DRAKE_VISIT(equilibrate_enable);
  DRAKE_VISIT(equilibrate_max_iter);
  DRAKE_VISIT(equilibrate_min_scaling);
  DRAKE_VISIT(equilibrate_max_scaling);
  DRAKE_VISIT(linesearch_backtrack_step);
  DRAKE_VISIT(min_switch_step_length);
  DRAKE_VISIT(min_terminate_step_length);
  DRAKE_VISIT(direct_kkt_solver);
  // skipped: direct_solve_method
  DRAKE_VISIT(static_regularization_enable);
  DRAKE_VISIT(static_regularization_constant);
  DRAKE_VISIT(static_regularization_proportional);
  DRAKE_VISIT(dynamic_regularization_enable);
  DRAKE_VISIT(dynamic_regularization_eps);
  DRAKE_VISIT(dynamic_regularization_delta);
  DRAKE_VISIT(iterative_refinement_enable);
  DRAKE_VISIT(iterative_refinement_reltol);
  DRAKE_VISIT(iterative_refinement_abstol);
  DRAKE_VISIT(iterative_refinement_max_iter);
  DRAKE_VISIT(iterative_refinement_stop_ratio);
  DRAKE_VISIT(presolve_enable);
#undef DRAKE_VISIT
}

}  // namespace clarabel
