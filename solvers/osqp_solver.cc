#include "drake/solvers/osqp_solver.h"

#include <optional>
#include <unordered_map>
#include <vector>

#include <osqp.h>

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/specific_options.h"

// This function must appear in the global namespace -- the Serialize pattern
// uses ADL (argument-dependent lookup) and the namespace for the OSQPSettings
// struct is the global namespace. (We can't even use an anonymous namespace!)
static void Serialize(
    drake::solvers::internal::SpecificOptions* archive,
    // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
    OSQPSettings& settings) {
  using drake::MakeNameValue;
  archive->Visit(MakeNameValue("device", &settings.device));
  // TODO(jwnimmer-tri) Add me here:
  // enum osqp_linsys_solver_type linsys_solver
  archive->Visit(MakeNameValue("allocate_solution",  // BR
                               &settings.allocate_solution));
  archive->Visit(MakeNameValue("verbose", &settings.verbose));
  archive->Visit(MakeNameValue("profiler_level", &settings.profiler_level));
  archive->Visit(MakeNameValue("warm_starting", &settings.warm_starting));
  archive->Visit(MakeNameValue("scaling", &settings.scaling));
  archive->Visit(MakeNameValue("polishing", &settings.polishing));
  archive->Visit(MakeNameValue("rho", &settings.rho));
  archive->Visit(MakeNameValue("rho_is_vec", &settings.rho_is_vec));
  archive->Visit(MakeNameValue("sigma", &settings.sigma));
  archive->Visit(MakeNameValue("alpha", &settings.alpha));
  archive->Visit(MakeNameValue("cg_max_iter", &settings.cg_max_iter));
  archive->Visit(MakeNameValue("cg_tol_reduction", &settings.cg_tol_reduction));
  archive->Visit(MakeNameValue("cg_tol_fraction", &settings.cg_tol_fraction));
  // TODO(jwnimmer-tri) Add me here:
  // osqp_precond_type cg_precond
  archive->Visit(MakeNameValue("adaptive_rho", &settings.adaptive_rho));
  archive->Visit(MakeNameValue("adaptive_rho_interval",  // BR
                               &settings.adaptive_rho_interval));
  archive->Visit(MakeNameValue("adaptive_rho_fraction",  // BR
                               &settings.adaptive_rho_fraction));
  archive->Visit(MakeNameValue("adaptive_rho_tolerance",
                               &settings.adaptive_rho_tolerance));
  archive->Visit(MakeNameValue("max_iter", &settings.max_iter));
  archive->Visit(MakeNameValue("eps_abs", &settings.eps_abs));
  archive->Visit(MakeNameValue("eps_rel", &settings.eps_rel));
  archive->Visit(MakeNameValue("eps_prim_inf", &settings.eps_prim_inf));
  archive->Visit(MakeNameValue("eps_dual_inf", &settings.eps_dual_inf));
  archive->Visit(MakeNameValue("scaled_termination",  // BR
                               &settings.scaled_termination));
  archive->Visit(MakeNameValue("check_termination",  // BR
                               &settings.check_termination));
  archive->Visit(MakeNameValue("check_dualgap", &settings.check_dualgap));
  archive->Visit(MakeNameValue("time_limit", &settings.time_limit));
  archive->Visit(MakeNameValue("delta", &settings.delta));
  archive->Visit(MakeNameValue("polish_refine_iter",  // BR
                               &settings.polish_refine_iter));
}

namespace drake {
namespace solvers {
namespace {

void ParseQuadraticCosts(const MathematicalProgram& prog,
                         Eigen::SparseMatrix<OSQPFloat>* P_upper,
                         std::vector<OSQPFloat>* q,
                         double* constant_cost_term) {
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());
  std::vector<Eigen::Triplet<OSQPFloat>> P_upper_triplets;
  internal::ParseQuadraticCosts(prog, &P_upper_triplets, q, constant_cost_term);
  // Scale the matrix P in the cost.
  // Note that the linear term is scaled in ParseLinearCosts().
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : P_upper_triplets) {
      // Column
      const auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
      // Row
      const auto row = scale_map.find(triplet.row());
      if (row != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (row->second));
      }
    }
  }

  P_upper->resize(prog.num_vars(), prog.num_vars());
  P_upper->setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
}

void ParseLinearCosts(const MathematicalProgram& prog,
                      std::vector<OSQPFloat>* q, double* constant_cost_term) {
  // Add the linear costs to the osqp cost.
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());
  internal::ParseLinearCosts(prog, q, constant_cost_term);

  // Scale the vector q in the cost.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (const auto& [index, scale] : scale_map) {
      q->at(index) *= scale;
    }
  }
}

// OSQP defines its own infinity in osqp/include/glob_opts.h.
OSQPFloat ConvertInfinity(double val) {
  if (std::isinf(val)) {
    if (val > 0) {
      return OSQP_INFTY;
    }
    return -OSQP_INFTY;
  }
  return static_cast<OSQPFloat>(val);
}

// Will call this function to parse both LinearConstraint and
// LinearEqualityConstraint.
template <typename C>
void ParseLinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& linear_constraints,
    std::vector<Eigen::Triplet<OSQPFloat>>* A_triplets,
    std::vector<OSQPFloat>* l, std::vector<OSQPFloat>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : linear_constraints) {
    const std::vector<int> x_indices =
        prog.FindDecisionVariableIndices(constraint.variables());
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(constraint.evaluator()->get_sparse_A());
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to osqp A.
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(*num_A_rows + Ai_triplet.row(),
                               x_indices[Ai_triplet.col()],
                               static_cast<OSQPFloat>(Ai_triplet.value()));
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
    std::vector<Eigen::Triplet<OSQPFloat>>* A_triplets,
    std::vector<OSQPFloat>* l, std::vector<OSQPFloat>* u, int* num_A_rows,
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
          static_cast<OSQPFloat>(1));
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
    const MathematicalProgram& prog, Eigen::SparseMatrix<OSQPFloat>* A,
    std::vector<OSQPFloat>* l, std::vector<OSQPFloat>* u,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  std::vector<Eigen::Triplet<OSQPFloat>> A_triplets;
  l->clear();
  u->clear();
  int num_A_rows = 0;
  ParseLinearConstraints(prog, prog.linear_constraints(), &A_triplets, l, u,
                         &num_A_rows, constraint_start_row);
  ParseLinearConstraints(prog, prog.linear_equality_constraints(), &A_triplets,
                         l, u, &num_A_rows, constraint_start_row);
  ParseBoundingBoxConstraints(prog, &A_triplets, l, u, &num_A_rows,
                              constraint_start_row);

  // Scale the matrix A.
  // Note that we only scale the columns of A, because the constraint has the
  // form l <= Ax <= u where the scaling of x enters the columns of A instead of
  // rows of A.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : A_triplets) {
      auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
    }
  }

  A->resize(num_A_rows, prog.num_vars());
  A->setFromTriplets(A_triplets.begin(), A_triplets.end());
}

// Convert an Eigen::SparseMatrix to csc_matrix, to be used by osqp.
// Make sure the input Eigen sparse matrix is compressed, by calling
// makeCompressed() function.
// The caller of this function is responsible for freeing the memory allocated
// here.
OSQPCscMatrix* EigenSparseToCSC(const Eigen::SparseMatrix<OSQPFloat>& mat) {
  // A csc matrix is in the compressed column major.
  OSQPFloat* values =
      static_cast<OSQPFloat*>(malloc(sizeof(OSQPFloat) * mat.nonZeros()));
  OSQPInt* inner_indices =
      static_cast<OSQPInt*>(malloc(sizeof(OSQPInt) * mat.nonZeros()));
  OSQPInt* outer_indices =
      static_cast<OSQPInt*>(malloc(sizeof(OSQPInt) * (mat.cols() + 1)));
  for (int i = 0; i < mat.nonZeros(); ++i) {
    values[i] = *(mat.valuePtr() + i);
    inner_indices[i] = static_cast<OSQPInt>(*(mat.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat.cols() + 1; ++i) {
    outer_indices[i] = static_cast<OSQPInt>(*(mat.outerIndexPtr() + i));
  }
  OSQPCscMatrix* result =
      OSQPCscMatrix_new(mat.rows(), mat.cols(), mat.nonZeros(), values,
                        inner_indices, outer_indices);
  result->owned = 1;
  return result;
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

bool OsqpSolver::is_available() {
  return true;
}

void OsqpSolver::DoSolve2(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          internal::SpecificOptions* options,
                          MathematicalProgramResult* result) const {
  OsqpSolverDetails& solver_details =
      result->SetSolverDetailsType<OsqpSolverDetails>();

  // OSQP solves a convex quadratic programming problem
  // min 0.5 xᵀPx + qᵀx
  // s.t l ≤ Ax ≤ u
  // OSQP is written in C, so this function will be in C style.

  // Get the cost for the QP.
  // Since OSQP 0.6.0 the P matrix is required to be upper triangular.
  Eigen::SparseMatrix<OSQPFloat> P_upper_sparse;
  std::vector<OSQPFloat> q(prog.num_vars(), 0);
  double constant_cost_term{0};

  ParseQuadraticCosts(prog, &P_upper_sparse, &q, &constant_cost_term);
  ParseLinearCosts(prog, &q, &constant_cost_term);

  // linear_constraint_start_row[binding] stores the starting row index in A
  // corresponding to the linear constraint `binding`.
  std::unordered_map<Binding<Constraint>, int> constraint_start_row;

  // Parse the linear constraints.
  Eigen::SparseMatrix<OSQPFloat> A_sparse;
  std::vector<OSQPFloat> l, u;
  ParseAllLinearConstraints(prog, &A_sparse, &l, &u, &constraint_start_row);

  // Now populate the constraint and cost as OSQP data.
  const OSQPInt n = prog.num_vars();
  const OSQPInt m = A_sparse.rows();
  const OSQPCscMatrix* P = EigenSparseToCSC(P_upper_sparse);
  const OSQPCscMatrix* A = EigenSparseToCSC(A_sparse);
  ScopeExit csc_guard([P, A]() {
    OSQPCscMatrix_free(const_cast<OSQPCscMatrix*>(P));
    OSQPCscMatrix_free(const_cast<OSQPCscMatrix*>(A));
  });

  // Create the settings, initialized to the upstream defaults.
  OSQPSettings* settings = OSQPSettings_new();
  ScopeExit settings_guard([settings]() {
    OSQPSettings_free(settings);
  });
  osqp_set_default_settings(settings);
  // Customize the defaults for Drake.
  // - Default polishing to true, to get an accurate solution.
  // - Disable adaptive rho, for determinism.
  settings->polishing = 1;
  settings->adaptive_rho_interval = OSQP_ADAPTIVE_RHO_FIXED;
  // Apply the user's additional options (if any).
  options->Respell([](const auto& common, auto* respelled) {
    respelled->emplace("verbose", common.print_to_console ? 1 : 0);
    // OSQP does not support setting the number of threads so we ignore the
    // kMaxThreads option.
  });
  options->CopyToSerializableStruct(settings);

  // If any step fails, it will set the solution_result and skip other steps.
  std::optional<SolutionResult> solution_result;

  // Setup workspace.
  OSQPSolver* solver = nullptr;
  ScopeExit solver_guard([&solver]() {
    if (solver != nullptr) {
      osqp_cleanup(solver);
    }
  });
  if (!solution_result) {
    const OSQPInt osqp_setup_err =
        osqp_setup(&solver, P, q.data(), A, l.data(), u.data(), m, n, settings);
    if (osqp_setup_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  if (!solution_result && initial_guess.array().isFinite().all()) {
    const OSQPInt osqp_warm_err =
        osqp_warm_start(solver, initial_guess.data(), nullptr);
    if (osqp_warm_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  // Solve problem.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(solver != nullptr);
    const OSQPInt osqp_solve_err = osqp_solve(solver);
    if (osqp_solve_err != 0) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }

  // Extract results.
  if (!solution_result) {
    DRAKE_THROW_UNLESS(solver->info != nullptr);

    solver_details.iter = solver->info->iter;
    solver_details.status_val = solver->info->status_val;
    solver_details.primal_res = solver->info->prim_res;
    solver_details.dual_res = solver->info->dual_res;
    solver_details.setup_time = solver->info->setup_time;
    solver_details.solve_time = solver->info->solve_time;
    solver_details.polish_time = solver->info->polish_time;
    solver_details.run_time = solver->info->run_time;
    solver_details.rho_updates = solver->info->rho_updates;

    // We set the primal and dual variables as long as osqp_solve() is finished.
    const Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> osqp_sol(
        solver->solution->x, prog.num_vars());

    // Scale solution back if `scale_map` is not empty.
    const auto& scale_map = prog.GetVariableScaling();
    if (!scale_map.empty()) {
      drake::VectorX<double> scaled_sol = osqp_sol.cast<double>();
      for (const auto& [index, scale] : scale_map) {
        scaled_sol(index) *= scale;
      }
      result->set_x_val(scaled_sol);
    } else {
      result->set_x_val(osqp_sol.cast<double>());
    }
    solver_details.y = Eigen::Map<Eigen::VectorXd>(solver->solution->y, m);
    SetDualSolution(prog.linear_constraints(), solver_details.y,
                    constraint_start_row, result);
    SetDualSolution(prog.linear_equality_constraints(), solver_details.y,
                    constraint_start_row, result);
    SetDualSolution(prog.bounding_box_constraints(), solver_details.y,
                    constraint_start_row, result);

    switch (solver->info->status_val) {
      case OSQP_SOLVED:
      case OSQP_SOLVED_INACCURATE: {
        result->set_optimal_cost(solver->info->obj_val + constant_cost_term);
        solution_result = SolutionResult::kSolutionFound;
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
        solution_result = SolutionResult::kSolverSpecificError;
        break;
      }
    }
  }
  result->set_solution_result(solution_result.value());
}

}  // namespace solvers
}  // namespace drake
