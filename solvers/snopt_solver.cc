#include "drake/solvers/snopt_solver.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

#include "snopt_cwrap.h"

// TODO(jwnimmer-tri) Eventually resolve these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// todo(sammy-tri) :  return more information that just the solution (INFO,
// infeasible constraints, ...)
// todo(sammy-tri) :  avoid all dynamic allocation

namespace drake {
namespace solvers {
namespace {
// This struct is used for passing additional info to the snopt_userfun, which
// evaluates the value and gradient of the cost and constraints. Apart from the
// standard information such as decision variable values, snopt_userfun could
// rely on additional information such as the cost gradient sparsity pattern.
struct SnoptUserFunInfo {
  const MathematicalProgram* prog_;
  const std::unordered_set<int>* cost_gradient_indices_;
};

// Return the number of rows in the nonlinear constraint.
template <typename C>
int SingleNonlinearConstraintSize(const C& constraint) {
  return constraint.num_constraints();
}

template <>
int SingleNonlinearConstraintSize<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return 1;
}

// Evaluate a single nonlinear constraints. For generic Constraint,
// LorentzConeConstraint, RotatedLorentzConeConstraint, we call Eval function
// of the constraint directly. For some other constraint, such as
// LinearComplementaryConstraint, we will evaluate its nonlinear constraint
// differently, than its Eval function.
template <typename C>
void EvaluateSingleNonlinearConstraint(const C& constraint,
                                       const Eigen::VectorXd& this_x,
                                       AutoDiffVecXd* ty) {
  ty->resize(SingleNonlinearConstraintSize(constraint));
  constraint.Eval(math::initializeAutoDiff(this_x), ty);
}

template <>
void EvaluateSingleNonlinearConstraint<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint,
    const Eigen::VectorXd& this_x, AutoDiffVecXd* ty) {
  ty->resize(1);
  auto tx = math::initializeAutoDiff(this_x);
  (*ty)(0) = tx.dot(constraint.M().cast<AutoDiffXd>() * tx +
                    constraint.q().cast<AutoDiffXd>());
}

/*
 * Evaluate the value and gradients of nonlinear constraints.
 * The template type Binding is supposed to be a
 * MathematicalProgram::Binding<Constraint> type.
 * @param constraint_list A list of Binding<Constraint>
 * @param F The value of the constraints
 * @param G The value of the non-zero entries in the gradient
 * @param constraint_index The starting index of the constraint_list(0) in the
 * optimization problem.
 * @param grad_index The starting index of the gradient of constraint_list(0)
 * in the optimization problem.
 * @param tx the AutoDiffMatrixType that stores the value of the decision
 * variable.
 */
template <typename C>
void EvaluateNonlinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, double F[], double G[],
    size_t* constraint_index, size_t* grad_index, const Eigen::VectorXd& xvec) {
  Eigen::VectorXd this_x;
  for (const auto& binding : constraint_list) {
    const auto& c = binding.evaluator();
    int num_constraints = SingleNonlinearConstraintSize(*c);

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int i = 0; i < num_v_variables; ++i) {
      this_x(i) = xvec(prog.FindDecisionVariableIndex(binding.variables()(i)));
    }

    AutoDiffVecXd ty;
    ty.resize(num_constraints);
    EvaluateSingleNonlinearConstraint(*c, this_x, &ty);

    for (int i = 0; i < num_constraints; i++) {
      F[(*constraint_index)++] = ty(i).value();
    }

    for (int i = 0; i < num_constraints; i++) {
      for (int j = 0; j < num_v_variables; ++j) {
        G[(*grad_index)++] = ty(i).derivatives()(j);
      }
    }
  }
}

/*
 * Loops through each cost stored in MathematicalProgram, and obtain the indices
 * of the non-zero gradient, in the summed cost.
 */
std::unordered_set<int> GetCostNonzeroGradientIndices(
    const MathematicalProgram& prog) {
  std::unordered_set<int> cost_gradient_indices;
  cost_gradient_indices.reserve(prog.num_vars());
  for (const auto& cost : prog.GetAllCosts()) {
    for (int i = 0; i < static_cast<int>(cost.GetNumElements()); ++i) {
      cost_gradient_indices.insert(
          prog.FindDecisionVariableIndex(cost.variables()(i)));
    }
  }
  return cost_gradient_indices;
}

void EvaluateAllCosts(const MathematicalProgram& prog,
                      const std::unordered_set<int>& cost_gradient_indices,
                      double F[], double G[], size_t* grad_index,
                      const Eigen::VectorXd& xvec) {
  // evaluate cost
  Eigen::VectorXd this_x;
  AutoDiffVecXd ty(1);

  std::vector<double> cost_gradient(prog.num_vars(), 0);
  for (auto const& binding : prog.GetAllCosts()) {
    auto const& obj = binding.evaluator();

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int j = 0; j < num_v_variables; ++j) {
      this_x(j) = xvec(prog.FindDecisionVariableIndex(binding.variables()(j)));
    }

    obj->Eval(math::initializeAutoDiff(this_x), &ty);

    F[0] += ty(0).value();

    for (int j = 0; j < num_v_variables; ++j) {
      size_t vj_index = prog.FindDecisionVariableIndex(binding.variables()(j));
      cost_gradient[vj_index] += ty(0).derivatives()(j);
    }
  }
  for (const auto cost_gradient_index : cost_gradient_indices) {
    G[*grad_index] = cost_gradient[cost_gradient_index];
    ++(*grad_index);
  }
}

void snopt_userfun(int* Status, int* n, double x[], int* needF, int* neF,
                   double F[], int* needG, int* neG, double G[], char* cu,
                   int* lencu, int iu[], int* leniu, double ru[], int* lenru) {
  // Our snOptA call passes the snopt workspace as the user workspace and
  // reserves one 8-char of space to pass the problem pointer.
  SnoptUserFunInfo const* snopt_userfun_info = NULL;
  {
    int* const p_snopt_userfun_info =
        reinterpret_cast<int*>(&snopt_userfun_info);
    int const* const iu_snopt_userfun_info = iu;
    std::copy(iu_snopt_userfun_info,
              iu_snopt_userfun_info + sizeof(snopt_userfun_info),
              p_snopt_userfun_info);
  }
  MathematicalProgram const* current_problem = snopt_userfun_info->prog_;
  std::unordered_set<int> const* cost_gradient_indices =
      snopt_userfun_info->cost_gradient_indices_;

  Eigen::VectorXd xvec(*n);
  for (int i = 0; i < *n; i++) {
    xvec(i) = x[i];
  }

  F[0] = 0.0;
  memset(G, 0, (*n) * sizeof(double));

  size_t grad_index = 0;

  current_problem->EvalVisualizationCallbacks(xvec);

  EvaluateAllCosts(*current_problem, *cost_gradient_indices, F, G, &grad_index,
                   xvec);

  // The constraint index starts at 1 because the cost is the
  // first row.
  size_t constraint_index = 1;
  // The gradient_index also starts after the cost.
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->generic_constraints(), F, G,
                               &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->lorentz_cone_constraints(), F,
                               G, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      *current_problem, current_problem->rotated_lorentz_cone_constraints(), F,
      G, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      *current_problem, current_problem->linear_complementarity_constraints(),
      F, G, &constraint_index, &grad_index, xvec);
}

/*
 * Updates the number of nonlinear constraints and the number of gradients by
 * looping through the constraint list
 * @tparam C A Constraint type. Note that some derived classes of Constraint
 * is regarded as generic constraint by SNOPT solver, such as
 * LorentzConeConstraint and RotatedLorentzConeConstraint, so @tparam C can also
 * be these derived classes.
 */
template <typename C>
void UpdateNumNonlinearConstraintsAndGradients(
    const std::vector<Binding<C>>& constraint_list,
    int* num_nonlinear_constraints, int* max_num_gradients) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();
    *max_num_gradients += n * binding.GetNumElements();
    *num_nonlinear_constraints += n;
  }
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the nonlinear constraint xᵀ(Mx+q) = 0
// So we only add one row of nonlinear constraint, and update the gradient of
// this nonlinear constraint accordingly.
template <>
void UpdateNumNonlinearConstraintsAndGradients<LinearComplementarityConstraint>(
    const std::vector<Binding<LinearComplementarityConstraint>>&
        constraint_list,
    int* num_nonlinear_constraints, int* max_num_gradients) {
  *num_nonlinear_constraints += constraint_list.size();
  for (const auto& binding : constraint_list) {
    *max_num_gradients += binding.evaluator()->M().rows();
  }
}

template <typename C>
void UpdateConstraintBoundsAndGradients(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, double* Flow, double* Fupp,
    double* Fmul, int* iGfun, int* jGvar, size_t* constraint_index,
    size_t* grad_index) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (int i = 0; i < n; i++) {
      Flow[*constraint_index + i] = lb(i);
      Fupp[*constraint_index + i] = ub(i);
      Fmul[*constraint_index + i] = 0;
    }

    for (int i = 0; i < n; i++) {
      for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
        iGfun[*grad_index] = *constraint_index + i + 1;  // row order
        jGvar[*grad_index] =
            prog.FindDecisionVariableIndex(binding.variables()(j)) + 1;
        (*grad_index)++;
      }
    }

    (*constraint_index) += n;
  }
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the nonlinear constraint xᵀ(Mx + q) = 0
// The bound of this constraint is 0. The indices of the non-zero gradient
// of this constraint is updated accordingly.
template <>
void UpdateConstraintBoundsAndGradients<LinearComplementarityConstraint>(
    const MathematicalProgram& prog,
    const std::vector<Binding<LinearComplementarityConstraint>>&
        constraint_list,
    double* Flow, double* Fupp, double* Fmul, int* iGfun, int* jGvar,
    size_t* constraint_index, size_t* grad_index) {
  for (const auto& binding : constraint_list) {
    Flow[*constraint_index] = 0;
    Fupp[*constraint_index] = 0;
    Fmul[*constraint_index] = 0;
    for (int j = 0; j < binding.evaluator()->M().rows(); ++j) {
      iGfun[*grad_index] = *constraint_index + 1;
      jGvar[*grad_index] =
          prog.FindDecisionVariableIndex(binding.variables()(j)) + 1;
      (*grad_index)++;
    }
    ++(*constraint_index);
  }
}

template <typename C>
Eigen::SparseMatrix<double> LinearConstraintA(const C& constraint) {
  return constraint.GetSparseMatrix();
}

// Return the number of rows in the linear constraint
template <typename C>
int LinearConstraintSize(const C& constraint) {
  return constraint.num_constraints();
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// The linear constraint we add to the program is Mx >= -q
// This linear constraint has the same number of rows, as matrix M.
template <>
int LinearConstraintSize<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return constraint.M().rows();
}

template <>
Eigen::SparseMatrix<double> LinearConstraintA<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return constraint.M().sparseView();
}

template <typename C>
std::pair<Eigen::VectorXd, Eigen::VectorXd> LinearConstraintBounds(
    const C& constraint) {
  return std::make_pair(constraint.lower_bound(), constraint.upper_bound());
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the constraint Mx >= -q
template <>
std::pair<Eigen::VectorXd, Eigen::VectorXd>
LinearConstraintBounds<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return std::make_pair(
      -constraint.q(),
      Eigen::VectorXd::Constant(constraint.q().rows(),
                                std::numeric_limits<double>::infinity()));
}

template <typename C>
void UpdateLinearConstraint(const MathematicalProgram& prog,
                            const std::vector<Binding<C>>& linear_constraints,
                            std::vector<Eigen::Triplet<double>>* tripletList,
                            double* Flow, double* Fupp,
                            size_t* constraint_index,
                            size_t* linear_constraint_index) {
  for (auto const& binding : linear_constraints) {
    auto const& c = binding.evaluator();
    int n = LinearConstraintSize(*c);

    const Eigen::SparseMatrix<double> A_constraint = LinearConstraintA(*c);

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_constraint, k); it;
           ++it) {
        tripletList->emplace_back(
            *linear_constraint_index + it.row(),
            prog.FindDecisionVariableIndex(binding.variables()(k)), it.value());
      }
    }

    const auto bounds = LinearConstraintBounds(*c);
    for (int i = 0; i < n; i++) {
      Flow[*constraint_index + i] = bounds.first(i);
      Fupp[*constraint_index + i] = bounds.second(i);
    }
    *constraint_index += n;
    *linear_constraint_index += n;
  }
}
}  // anon namespace

bool SnoptSolver::available() const { return true; }

SolutionResult SnoptSolver::Solve(MathematicalProgram& prog) const {
  snProblem drake_problem;
  // No print file, no summary.
  char problem_name[14] = "drake_problem";
  snInit(&drake_problem, problem_name, "", 0);

  SnoptUserFunInfo snopt_userfun_info;
  snopt_userfun_info.prog_ = &prog;
  const std::unordered_set<int> cost_gradient_indices =
      GetCostNonzeroGradientIndices(prog);
  snopt_userfun_info.cost_gradient_indices_ = &cost_gradient_indices;

  const int nx = prog.num_vars();
  double* x = new double[nx];
  double* xlow = new double[nx];
  double* xupp = new double[nx];
  double* xmul = new double[nx];
  int* xstate = new int[nx];

  // Initialize the guess for x.
  const Eigen::VectorXd& x_initial_guess = prog.initial_guess();
  for (int i = 0; i < nx; ++i) {
    if (!std::isnan(x_initial_guess(i))) {
      x[i] = x_initial_guess(i);
    } else {
      x[i] = 0.0;
    }
    xlow[i] = -std::numeric_limits<double>::infinity();
    xupp[i] = std::numeric_limits<double>::infinity();
    xstate[i] = 0;
  }
  // Set up the lower and upper bounds.
  for (auto const& binding : prog.bounding_box_constraints()) {
    const auto& c = binding.evaluator();
    const auto& lb = c->lower_bound();
    const auto& ub = c->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[vk_index] = std::max(lb(k), xlow[vk_index]);
      xupp[vk_index] = std::min(ub(k), xupp[vk_index]);
    }
  }
  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // we add the bounding box constraint x >= 0
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[vk_index] = std::max(xlow[vk_index], 0.0);
    }
  }

  // Update nonlinear constraints.
  int num_nonlinear_constraints = 0;
  int max_num_gradients = cost_gradient_indices.size();
  UpdateNumNonlinearConstraintsAndGradients(prog.generic_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(prog.lorentz_cone_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.rotated_lorentz_cone_constraints(), &num_nonlinear_constraints,
      &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.linear_complementarity_constraints(), &num_nonlinear_constraints,
      &max_num_gradients);

  // Update linear constraints.
  int num_linear_constraints = 0;
  const auto linear_constraints = prog.GetAllLinearConstraints();
  for (auto const& binding : linear_constraints) {
    num_linear_constraints += binding.evaluator()->num_constraints();
  }

  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // The linear constraint we add is Mx + q >= 0, so we will append
  // M.rows() rows to the linear constraints.
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    num_linear_constraints += binding.evaluator()->M().rows();
  }

  // Update the bound of the constraint.
  const int nF = 1 + num_nonlinear_constraints + num_linear_constraints;
  double* F = new double[nF];
  double* Flow = new double[nF];
  double* Fupp = new double[nF];
  double* Fmul = new double[nF];
  int* Fstate = new int[nF];
  // F[0] is the cost, so Flow[0] = -∞, Fupp[0] = ∞.
  Flow[0] = -std::numeric_limits<double>::infinity();
  Fupp[0] = std::numeric_limits<double>::infinity();
  Fmul[0] = 0;

  // Set up the gradient sparsity pattern.
  const int lenG = max_num_gradients;
  int* iGfun = new int[lenG];
  int* jGvar = new int[lenG];
  size_t grad_index = 0;
  for (const auto cost_gradient_index : cost_gradient_indices) {
    iGfun[grad_index] = 1;
    jGvar[grad_index] = cost_gradient_index + 1;
    ++grad_index;
  }
  // constraint_index starts at 1 because row 0 is the cost.
  size_t constraint_index = 1;
  UpdateConstraintBoundsAndGradients(prog, prog.generic_constraints(), Flow,
                                     Fupp, Fmul, iGfun, jGvar,
                                     &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(prog, prog.lorentz_cone_constraints(),
                                     Flow, Fupp, Fmul, iGfun, jGvar,
                                     &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.rotated_lorentz_cone_constraints(), Flow, Fupp, Fmul, iGfun,
      jGvar, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.linear_complementarity_constraints(), Flow, Fupp, Fmul, iGfun,
      jGvar, &constraint_index, &grad_index);

  // Now find the sparsity pattern of the linear constraints, and also update
  // Flow and Fupp corresponding to the linear constraints.
  // We use Eigen::Triplet to store the non-zero entries in the linear
  // constraint
  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(num_linear_constraints * prog.num_vars());

  size_t linear_constraint_index = 0;
  UpdateLinearConstraint(prog, linear_constraints, &tripletList, Flow, Fupp,
                         &constraint_index, &linear_constraint_index);
  UpdateLinearConstraint(prog, prog.linear_complementarity_constraints(),
                         &tripletList, Flow, Fupp, &constraint_index,
                         &linear_constraint_index);

  const int lenA = tripletList.size();
  double* A = new double[lenA];
  int* iAfun = new int[lenA];
  int* jAvar = new int[lenA];
  size_t A_index = 0;
  for (auto const& it : tripletList) {
    A[A_index] = it.value();
    iAfun[A_index] = 1 + num_nonlinear_constraints + it.row() + 1;
    jAvar[A_index] = it.col() + 1;
    A_index++;
  }

  const std::map<std::string, std::string>& snopt_option_str =
      prog.GetSolverOptionsStr(id());
  // Determines if we should print out snopt debugging info.
  const auto print_file_it = snopt_option_str.find("Print file");
  if (print_file_it != snopt_option_str.end()) {
    setPrintfile(&drake_problem,
                 const_cast<char*>(print_file_it->second.c_str()));
    char major_print_level[18] = "Major print level";
    setIntParameter(&drake_problem, major_print_level, 11);
    char print_file[11] = "Print file";
    setIntParameter(&drake_problem, print_file, 9);
  }

  for (const auto it : prog.GetSolverOptionsDouble(id())) {
    setRealParameter(&drake_problem, const_cast<char*>(it.first.c_str()),
                     it.second);
  }

  for (const auto it : prog.GetSolverOptionsInt(id())) {
    setIntParameter(&drake_problem, const_cast<char*>(it.first.c_str()),
                    it.second);
  }

  // Set the workspace.
  // integer user workspace
  // This is a tricky part, we use the integer workspace iu[snopt_miniu] to
  // store snopt_user_info. In snopt_userfun, we will need to read
  // snopt_user_info from the integer workspace.
  drake_problem.leniu = 100;
  drake_problem.iu =
      static_cast<int*>(malloc(sizeof(int) * drake_problem.leniu));
  {
    int const* const p_snopt_userfun_info =
        reinterpret_cast<int*>(&snopt_userfun_info);
    int* const iu_snopt_userfun_info = &drake_problem.iu[0];
    std::copy(p_snopt_userfun_info,
              p_snopt_userfun_info + sizeof(snopt_userfun_info),
              iu_snopt_userfun_info);
  }

  const int Cold = 0;
  const double ObjAdd = 0.0;
  const int ObjRow = 0;
  int nS = 0;
  // Should I left nInf and sInf uninitialized? The SNOPT example code leaves
  // them uninitialized.
  int nInf;
  double sInf;

  const int info =
      solveA(&drake_problem, Cold, nF, nx, ObjAdd, ObjRow, snopt_userfun, lenA,
             iAfun, jAvar, A, lenG, iGfun, jGvar, xlow, xupp, Flow, Fupp, x,
             xstate, xmul, F, Fstate, Fmul, &nS, &nInf, &sInf);

  SolverResult solver_result(id());
  solver_result.set_decision_variable_values(
      Eigen::Map<VectorX<double>>(x, nx));
  solver_result.set_optimal_cost(*F);

  // todo: extract the other useful quantities, too.

  SolutionResult solution_result{SolutionResult::kUnknownError};
  if (info >= 1 && info <= 6) {
    solution_result = SolutionResult::kSolutionFound;
  } else {
    drake::log()->debug("Snopt returns code {}\n", info);
    if (info >= 11 && info <= 16) {
      solution_result = SolutionResult::kInfeasibleConstraints;
    } else if (info >= 20 && info <= 22) {
      solver_result.set_optimal_cost(MathematicalProgram::kUnboundedCost);
      solution_result = SolutionResult::kUnbounded;
    } else if (info >= 30 && info <= 32) {
      solution_result = SolutionResult::kIterationLimit;
    } else if (info == 91) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }
  prog.SetSolverResult(solver_result);

  delete[] iAfun;
  delete[] jAvar;
  delete[] A;
  delete[] iGfun;
  delete[] jGvar;
  delete[] x;
  delete[] xlow;
  delete[] xupp;
  delete[] xmul;
  delete[] xstate;
  delete[] F;
  delete[] Flow;
  delete[] Fupp;
  delete[] Fmul;
  delete[] Fstate;

  free(&drake_problem.iu);
  deleteSNOPT(&drake_problem);
  return solution_result;
}

}  // namespace solvers
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wunused-parameter"
