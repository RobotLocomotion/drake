#include "drake/solvers/osqp_solver.h"

#include <vector>

#include <osqp.h>

#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
void ParseQuadraticCosts(const MathematicalProgram& prog,
                         Eigen::SparseMatrix<c_float>* P,
                         std::vector<c_float>* q, double* constant_cost_term) {
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop through each quadratic costs in prog, and compute the Hessian matrix
  // P, the linear cost q, and the constant cost term.
  std::vector<Eigen::Triplet<c_float>> P_triplets;
  for (const auto& quadratic_cost : prog.quadratic_costs()) {
    const VectorXDecisionVariable& x = quadratic_cost.variables();
    // x_indices are the indices of the variables x (the variables bound with
    // this quadratic cost) in the program decision variables.
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);

    // Add quadratic_cost.Q to the Hessian P.
    const std::vector<Eigen::Triplet<double>> Qi_triplets =
        math::SparseMatrixToTriplets(quadratic_cost.evaluator()->Q());
    P_triplets.reserve(P_triplets.size() + Qi_triplets.size());
    for (int i = 0; i < static_cast<int>(Qi_triplets.size()); ++i) {
      // Unpack the field of the triplet (for clarity below).
      const int row = x_indices[Qi_triplets[i].row()];
      const int col = x_indices[Qi_triplets[i].col()];
      const double value = Qi_triplets[i].value();
      // Since OSQP 0.6.0 the P matrix is required to be upper triangular, so
      // we only add upper triangular entries to P_triplets.
      if (row <= col) {
        P_triplets.emplace_back(row, col, static_cast<c_float>(value));
      }
    }

    // Add quadratic_cost.b to the linear cost term q.
    for (int i = 0; i < x.rows(); ++i) {
      q->at(x_indices[i]) += quadratic_cost.evaluator()->b()(i);
    }

    // Add quadratic_cost.c to constant term
    *constant_cost_term += quadratic_cost.evaluator()->c();
  }
  P->resize(prog.num_vars(), prog.num_vars());
  P->setFromTriplets(P_triplets.begin(), P_triplets.end());
}

void ParseLinearCosts(const MathematicalProgram& prog, std::vector<c_float>* q,
                      double* constant_cost_term) {
  // Add the linear costs to the osqp cost.
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop over the linear costs stored inside prog.
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < static_cast<int>(linear_cost.GetNumElements()); ++i) {
      // Append the linear cost term to q.
      if (linear_cost.evaluator()->a()(i) != 0) {
        const int x_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        q->at(x_index) += linear_cost.evaluator()->a()(i);
      }
    }
    // Add the constant cost term to constant_cost_term.
    *constant_cost_term += linear_cost.evaluator()->b();
  }
}

// OSQP defines its own infinity in osqp/include/glob_opts.h.
c_float ConvertInfinity(double val) {
  if (std::isinf(val)) {
    if (val > 0) {
      return OSQP_INFTY;
    }
    return -OSQP_INFTY;
  }
  return static_cast<c_float>(val);
}

// Will call this function to parse both LinearConstraint and
// LinearEqualityConstraint.
template <typename C>
void ParseLinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& linear_constraints,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : linear_constraints) {
    const std::vector<int> x_indices =
        prog.FindDecisionVariableIndices(constraint.variables());
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(constraint.evaluator()->A());
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to osqp A.
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(*num_A_rows + Ai_triplet.row(),
                               x_indices[Ai_triplet.col()],
                               static_cast<c_float>(Ai_triplet.value()));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

void ParseBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : prog.bounding_box_constraints()) {
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to osqp A.
    for (int i = 0; i < static_cast<int>(constraint.GetNumElements()); ++i) {
      A_triplets->emplace_back(
          *num_A_rows + i,
          prog.FindDecisionVariableIndex(constraint.variables()(i)),
          static_cast<c_float>(1));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

void ParseAllLinearConstraints(
    const MathematicalProgram& prog, Eigen::SparseMatrix<c_float>* A,
    std::vector<c_float>* l, std::vector<c_float>* u,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  std::vector<Eigen::Triplet<c_float>> A_triplets;
  l->clear();
  u->clear();
  int num_A_rows = 0;
  ParseLinearConstraints(prog, prog.linear_constraints(), &A_triplets, l, u,
                         &num_A_rows, constraint_start_row);
  ParseLinearConstraints(prog, prog.linear_equality_constraints(), &A_triplets,
                         l, u, &num_A_rows, constraint_start_row);
  ParseBoundingBoxConstraints(prog, &A_triplets, l, u, &num_A_rows,
                              constraint_start_row);
  A->resize(num_A_rows, prog.num_vars());
  A->setFromTriplets(A_triplets.begin(), A_triplets.end());
}

// Convert an Eigen::SparseMatrix to csc_matrix, to be used by osqp.
// Make sure the input Eigen sparse matrix is compressed, by calling
// makeCompressed() function.
// The caller of this function is responsible for freeing the memory allocated
// here.
csc* EigenSparseToCSC(const Eigen::SparseMatrix<c_float>& mat) {
  // A csc matrix is in the compressed column major.
  c_float* values =
      static_cast<c_float*>(c_malloc(sizeof(c_float) * mat.nonZeros()));
  c_int* inner_indices =
      static_cast<c_int*>(c_malloc(sizeof(c_int) * mat.nonZeros()));
  c_int* outer_indices =
      static_cast<c_int*>(c_malloc(sizeof(c_int) * (mat.cols() + 1)));
  for (int i = 0; i < mat.nonZeros(); ++i) {
    values[i] = *(mat.valuePtr() + i);
    inner_indices[i] = static_cast<c_int>(*(mat.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat.cols() + 1; ++i) {
    outer_indices[i] = static_cast<c_int>(*(mat.outerIndexPtr() + i));
  }
  return csc_matrix(mat.rows(), mat.cols(), mat.nonZeros(), values,
                    inner_indices, outer_indices);
}

template <typename T1, typename T2>
void SetOsqpSolverSetting(const std::unordered_map<std::string, T1>& options,
                          const std::string& option_name,
                          T2* osqp_setting_field) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  }
}

template <typename T1, typename T2>
void SetOsqpSolverSettingWithDefaultValue(
    const std::unordered_map<std::string, T1>& options,
    const std::string& option_name, T2* osqp_setting_field,
    const T1& default_field_value) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  } else {
    *osqp_setting_field = default_field_value;
  }
}

void SetOsqpSolverSettings(const SolverOptions& solver_options,
                           OSQPSettings* settings) {
  const std::unordered_map<std::string, double>& options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const std::unordered_map<std::string, int>& options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());
  SetOsqpSolverSetting(options_double, "rho", &(settings->rho));
  SetOsqpSolverSetting(options_double, "sigma", &(settings->sigma));
  SetOsqpSolverSetting(options_int, "max_iter", &(settings->max_iter));
  SetOsqpSolverSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetOsqpSolverSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetOsqpSolverSetting(options_double, "eps_prim_inf",
                       &(settings->eps_prim_inf));
  SetOsqpSolverSetting(options_double, "eps_dual_inf",
                       &(settings->eps_dual_inf));
  SetOsqpSolverSetting(options_double, "alpha", &(settings->alpha));
  SetOsqpSolverSetting(options_double, "delta", &(settings->delta));
  // Default polish to true, to get an accurate solution.
  SetOsqpSolverSettingWithDefaultValue(options_int, "polish",
                                       &(settings->polish), 1);
  SetOsqpSolverSetting(options_int, "polish_refine_iter",
                       &(settings->polish_refine_iter));
  // The fallback value for console verbosity is the value set by drake options.
  int verbose_console = solver_options.get_print_to_console() != 0;
  SetOsqpSolverSettingWithDefaultValue(options_int, "verbose",
                                       &(settings->verbose), verbose_console);
  SetOsqpSolverSetting(options_int, "scaled_termination",
                       &(settings->scaled_termination));
  SetOsqpSolverSetting(options_int, "check_termination",
                       &(settings->check_termination));
  SetOsqpSolverSetting(options_int, "warm_start", &(settings->warm_start));
  SetOsqpSolverSetting(options_int, "scaling", &(settings->scaling));
  SetOsqpSolverSetting(options_int, "adaptive_rho", &(settings->adaptive_rho));
  SetOsqpSolverSetting(options_double, "adaptive_rho_interval",
                       &(settings->adaptive_rho_interval));
  SetOsqpSolverSetting(options_double, "adaptive_rho_tolerance",
                       &(settings->adaptive_rho_tolerance));
  SetOsqpSolverSetting(options_double, "adaptive_rho_fraction",
                       &(settings->adaptive_rho_fraction));
  SetOsqpSolverSetting(options_double, "time_limit", &(settings->time_limit));
}

template <typename C>
void SetDualSolution(
    const std::vector<Binding<C>>& constraints,
    const Eigen::VectorXd& all_dual_solution,
    const std::unordered_map<Binding<Constraint>, int>& constraint_start_row,
    MathematicalProgramResult* result) {
  for (const auto& constraint : constraints) {
    // OSQP uses the dual variable `y` as the negation of the shadow price, so
    // we need to negate `all_dual_solution` as Drake interprets dual solution
    // as the shadow price.
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    result->set_dual_solution(
        constraint,
        -all_dual_solution.segment(constraint_start_row.at(constraint_cast),
                                   constraint.evaluator()->num_constraints()));
  }
}
}  // namespace

bool OsqpSolver::is_available() { return true; }

void OsqpSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "OsqpSolver doesn't support the feature of variable scaling.");
  }

  OsqpSolverDetails& solver_details =
      result->SetSolverDetailsType<OsqpSolverDetails>();

  // TODO(hongkai.dai): OSQP uses initial guess to warm start.
  unused(initial_guess);

  // OSQP solves a convex quadratic programming problem
  // min 0.5 xᵀPx + qᵀx
  // s.t l ≤ Ax ≤ u
  // OSQP is written in C, so this function will be in C style.

  // Get the cost for the QP.
  Eigen::SparseMatrix<c_float> P_sparse;
  std::vector<c_float> q(prog.num_vars(), 0);
  double constant_cost_term{0};

  ParseQuadraticCosts(prog, &P_sparse, &q, &constant_cost_term);
  ParseLinearCosts(prog, &q, &constant_cost_term);

  // linear_constraint_start_row[binding] stores the starting row index in A
  // corresponding to the linear constraint `binding`.
  std::unordered_map<Binding<Constraint>, int> constraint_start_row;

  // Parse the linear constraints.
  Eigen::SparseMatrix<c_float> A_sparse;
  std::vector<c_float> l, u;
  ParseAllLinearConstraints(prog, &A_sparse, &l, &u, &constraint_start_row);

  // Now pass the constraint and cost to osqp data.
  OSQPData* data = nullptr;

  // Populate data.
  data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  data->n = prog.num_vars();
  data->m = A_sparse.rows();
  data->P = EigenSparseToCSC(P_sparse);
  data->q = q.data();
  data->A = EigenSparseToCSC(A_sparse);
  data->l = l.data();
  data->u = u.data();

  // Define Solver settings as default.
  // Problem settings
  OSQPSettings* settings =
      static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);

  SetOsqpSolverSettings(merged_options, settings);

  // If any step fails, it will set the solution_result and skip other steps.
  std::optional<SolutionResult> solution_result;

  // Setup workspace.
  OSQPWorkspace* work = nullptr;
  if (!solution_result) {
    const c_int osqp_setup_err = osqp_setup(&work, data, settings);
    if (osqp_setup_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  // Solve problem.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(work != nullptr);
    const c_int osqp_solve_err = osqp_solve(work);
    if (osqp_solve_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  // Extract results.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(work->info != nullptr);

    solver_details.iter = work->info->iter;
    solver_details.status_val = work->info->status_val;
    solver_details.primal_res = work->info->pri_res;
    solver_details.dual_res = work->info->dua_res;
    solver_details.setup_time = work->info->setup_time;
    solver_details.solve_time = work->info->solve_time;
    solver_details.polish_time = work->info->polish_time;
    solver_details.run_time = work->info->run_time;

    switch (work->info->status_val) {
      case OSQP_SOLVED:
      case OSQP_SOLVED_INACCURATE: {
        const Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> osqp_sol(
            work->solution->x, prog.num_vars());
        result->set_x_val(osqp_sol.cast<double>());
        result->set_optimal_cost(work->info->obj_val + constant_cost_term);
        solver_details.y =
            Eigen::Map<Eigen::VectorXd>(work->solution->y, work->data->m);
        solution_result = SolutionResult::kSolutionFound;
        SetDualSolution(prog.linear_constraints(), solver_details.y,
                        constraint_start_row, result);
        SetDualSolution(prog.linear_equality_constraints(), solver_details.y,
                        constraint_start_row, result);
        SetDualSolution(prog.bounding_box_constraints(), solver_details.y,
                        constraint_start_row, result);

        break;
      }
      case OSQP_PRIMAL_INFEASIBLE:
      case OSQP_PRIMAL_INFEASIBLE_INACCURATE: {
        solution_result = SolutionResult::kInfeasibleConstraints;
        result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
        break;
      }
      case OSQP_DUAL_INFEASIBLE:
      case OSQP_DUAL_INFEASIBLE_INACCURATE: {
        solution_result = SolutionResult::kDualInfeasible;
        break;
      }
      case OSQP_MAX_ITER_REACHED: {
        solution_result = SolutionResult::kIterationLimit;
        break;
      }
      default: {
        solution_result = SolutionResult::kUnknownError;
        break;
      }
    }
  }
  result->set_solution_result(solution_result.value());

  // Clean workspace.
  osqp_cleanup(work);
  c_free(data->P->x);
  c_free(data->P->i);
  c_free(data->P->p);
  c_free(data->P);
  c_free(data->A->x);
  c_free(data->A->i);
  c_free(data->A->p);
  c_free(data->A);
  c_free(data);
  c_free(settings);
}

}  // namespace solvers
}  // namespace drake
