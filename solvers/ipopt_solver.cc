#include "drake/solvers/ipopt_solver.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

using Ipopt::Index;
using Ipopt::IpoptCalculatedQuantities;
using Ipopt::IpoptData;
using Ipopt::Number;
using Ipopt::SolverReturn;

namespace drake {
namespace solvers {
namespace {

/// @param[out] lb Array of constraint lower bounds, parallel to @p ub
/// @param[out] ub Array of constraint upper bounds, parallel to @p lb
int GetConstraintBounds(const Constraint& c, Number* lb, Number* ub) {
  const Eigen::VectorXd& lower_bound = c.lower_bound();
  const Eigen::VectorXd& upper_bound = c.upper_bound();
  for (int i = 0; i < c.num_constraints(); i++) {
    lb[i] = lower_bound(i);
    ub[i] = upper_bound(i);
  }

  return c.num_constraints();
}

template <typename C>
void SetConstraintDualVariableIndex(
    const Binding<C>& binding, int constraint_index,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  const Binding<Constraint> binding_cast =
      internal::BindingDynamicCast<Constraint>(binding);
  constraint_dual_start_index->emplace(binding_cast, constraint_index);
}

void SetBoundingBoxConstraintDualSolution(
    const MathematicalProgram& prog, const Number* const z_L,
    const Number* const z_U,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             std::pair<std::vector<int>, std::vector<int>>>&
        bb_con_dual_variable_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : prog.bounding_box_constraints()) {
    std::vector<int> lower_dual_indices, upper_dual_indices;
    std::tie(lower_dual_indices, upper_dual_indices) =
        bb_con_dual_variable_indices.at(binding);
    Eigen::VectorXd dual_solution =
        Eigen::VectorXd::Zero(binding.GetNumElements());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      if (lower_dual_indices[i] != -1) {
        // Ipopt always returns a non-negative z_L. The definition of shadow
        // price means the dual variable for the lower bound is also
        // non-negative.
        dual_solution(i) = z_L[lower_dual_indices[i]];
      }
      // Ipopt always returns a non-negative z_U. But the definition of shadow
      // price means that the dual variable for the upper bound is negative, so
      // we need to negate z_U to get the dual solution.
      if (upper_dual_indices[i] != -1 &&
          z_U[upper_dual_indices[i]] >= dual_solution(i)) {
        // At most one side of the bounds can be active, so theoretically at
        // most one of z_U[upper_dual_indices[i]] or z_L[lower_dual_indices[i]]
        // can be non-zero. In practice, due to small numerical errors, both
        // z_U[upper_dual_indices[i]] and z_L[lower_dual_indices[i]] can be
        // non-zero, with one of them being very small number. We choose the
        // side with larger dual variable value as the active side (by comparing
        // z_U[upper_dual_indices[i] >= dual_solution(i)).
        dual_solution(i) = -z_U[upper_dual_indices[i]];
      }
    }
    result->set_dual_solution(binding, dual_solution);
  }
}

template <typename C>
void SetConstraintDualSolution(
    const Binding<C>& binding, const Eigen::VectorXd& lambda,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    MathematicalProgramResult* result) {
  const Binding<Constraint> binding_cast =
      internal::BindingDynamicCast<Constraint>(binding);
  // Ipopt defines multiplier as the negative of the shadow price, hence we have
  // to negate lambda.
  result->set_dual_solution(
      binding_cast,
      -lambda.segment(constraint_dual_start_index.at(binding_cast),
                      binding.evaluator()->num_constraints()));
}

void SetAllConstraintDualSolution(
    const MathematicalProgram& prog, const Eigen::VectorXd& lambda,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    MathematicalProgramResult* result) {
  for (const auto& binding : prog.generic_constraints()) {
    SetConstraintDualSolution(binding, lambda, constraint_dual_start_index,
                              result);
  }
  for (const auto& binding : prog.lorentz_cone_constraints()) {
    SetConstraintDualSolution(binding, lambda, constraint_dual_start_index,
                              result);
  }
  for (const auto& binding : prog.rotated_lorentz_cone_constraints()) {
    SetConstraintDualSolution(binding, lambda, constraint_dual_start_index,
                              result);
  }
  for (const auto& binding : prog.linear_constraints()) {
    SetConstraintDualSolution(binding, lambda, constraint_dual_start_index,
                              result);
  }
  for (const auto& binding : prog.linear_equality_constraints()) {
    SetConstraintDualSolution(binding, lambda, constraint_dual_start_index,
                              result);
  }
}

/// @param[out] num_grad number of gradients
/// @return number of constraints
int GetNumGradients(const Constraint& c, int var_count, Index* num_grad) {
  const int num_constraints = c.num_constraints();
  *num_grad = num_constraints * var_count;
  return num_constraints;
}

/// @param constraint_idx The starting row number for the constraint
/// being described.
///
/// Parameters @p iRow and @p jCol are used in the same manner as
/// described in
/// http://www.coin-or.org/Ipopt/documentation/node23.html for the
/// eval_jac_g() function (in the mode where it's requesting the
/// sparsity structure of the Jacobian).  The triplet format is also
/// described in
/// http://www.coin-or.org/Ipopt/documentation/node38.html#app.triplet
///
/// @return the number of row/column pairs filled in.
size_t GetGradientMatrix(
    const MathematicalProgram& prog, const Constraint& c,
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    Index constraint_idx, Index* iRow, Index* jCol) {
  const int m = c.num_constraints();
  size_t grad_index = 0;

  for (int i = 0; i < static_cast<int>(m); ++i) {
    for (int j = 0; j < variables.rows(); ++j) {
      iRow[grad_index] = constraint_idx + i;
      jCol[grad_index] = prog.FindDecisionVariableIndex(variables(j));
      grad_index++;
    }
  }

  return grad_index;
}

Eigen::VectorXd MakeEigenVector(Index n, const Number* x) {
  Eigen::VectorXd xvec(n);
  for (Index i = 0; i < n; i++) {
    xvec[i] = x[i];
  }
  return xvec;
}

/// Evaluate a constraint, storing the result of the evaluation into
/// @p result and gradients into @p grad.  @p grad is the sparse
/// matrix data for which the structure was defined in
/// GetGradientMatrix.
///
/// @return number of gradient entries populated.
size_t EvaluateConstraint(const MathematicalProgram& prog,
                          const Eigen::VectorXd& xvec, const Constraint& c,
                          const VectorXDecisionVariable& variables,
                          Number* result, Number* grad) {
  // For constraints which don't use all of the variables in the X
  // input, extract a subset into the AutoDiffVecXd this_x to evaluate
  // the constraint (we actually do this for all constraints.  One
  // potential optimization might be to detect if the initial "tx" has
  // the correct geometry (e.g. the constraint uses all decision
  // variables in the same order they appear in xvec), but this is not
  // currently done).
  int num_v_variables = variables.rows();
  Eigen::VectorXd this_x(num_v_variables);
  for (int i = 0; i < num_v_variables; ++i) {
    this_x(i) = xvec(prog.FindDecisionVariableIndex(variables(i)));
  }

  if (!grad) {
    // We don't want the gradient info, so just call the VectorXd version of
    // Eval.
    Eigen::VectorXd ty(c.num_constraints());

    c.Eval(this_x, &ty);

    // Store the results.
    for (int i = 0; i < c.num_constraints(); i++) {
      result[i] = ty(i);
    }
    return 0;
  }

  // Run the version which calculates gradients.

  AutoDiffVecXd ty(c.num_constraints());
  c.Eval(math::InitializeAutoDiff(this_x), &ty);

  // Store the results.  Since IPOPT directly knows the bounds of the
  // constraint, we don't need to apply any bounding information here.
  for (int i = 0; i < c.num_constraints(); i++) {
    result[i] = ty(i).value();
  }

  // Extract the appropriate derivatives from our result into the
  // gradient array.
  size_t grad_idx = 0;

  DRAKE_ASSERT(ty.rows() == c.num_constraints());
  for (int i = 0; i < ty.rows(); i++) {
    if (ty(i).derivatives().size() > 0) {
      for (int j = 0; j < variables.rows(); j++) {
        grad[grad_idx++] = ty(i).derivatives()(j);
      }
    } else {
      for (int j = 0; j < variables.rows(); j++) {
        grad[grad_idx++] = 0;
      }
    }
  }

  return grad_idx;
}

// IPOPT uses separate callbacks to get the result and the gradients.  When
// this code was initially written, the gradient values were populated in the
// cache during the result calculation for constraints (this is still true for
// costs).  However, it was later discovered that because IPOPT does not
// always ask for the gradients to be calculated, it's actually faster to
// calculate the constraint values only and then recalculate later with
// gradients only if necessary.  This likely makes the cache ineffective for
// constraints.
//
// See #13841 and #13891 for more discussion.
struct ResultCache {
  ResultCache(size_t x_size, size_t result_size, size_t grad_size) {
    // The choice of infinity as the default value below is arbitrary.
    x.resize(x_size, std::numeric_limits<double>::infinity());
    result.resize(result_size, std::numeric_limits<double>::infinity());
    grad.resize(grad_size, std::numeric_limits<double>::infinity());
  }

  /// @param n The size of the array located at @p x_in.
  bool is_x_equal(Index n, const Number* x_in) {
    DRAKE_ASSERT(n == static_cast<Index>(x.size()));
    return !std::memcmp(x.data(), x_in, x.size() * sizeof(Number));
  }

  // Sugar to copy an IPOPT bare array into `x`.
  void SetX(const Index n, const Number* x_arg) {
    DRAKE_ASSERT(static_cast<Index>(x.size()) == n);
    grad_valid = false;
    if (n == 0) { return; }
    DRAKE_ASSERT(x_arg != nullptr);
    std::memcpy(x.data(), x_arg, n * sizeof(Number));
  }

  // Sugar to copy one of our member fields into an IPOPT bare array.
  static void Extract(
      const std::vector<Number>& cache_data,
      const Index dest_size, Number* dest) {
    DRAKE_ASSERT(static_cast<Index>(cache_data.size()) == dest_size);
    if (dest_size == 0) { return; }
    DRAKE_ASSERT(dest != nullptr);
    std::memcpy(dest, cache_data.data(), dest_size * sizeof(Number));
  }

  std::vector<Number> x;
  std::vector<Number> result;
  std::vector<Number> grad;
  bool grad_valid{false};
};

// The C++ interface for IPOPT is described here:
// https://coin-or.github.io/Ipopt/INTERFACES.html#INTERFACE_CPP
//
// IPOPT provides a pure(-ish) virtual base class which you have to
// implement a concrete version of as the solver interface.
// IpoptSolver creates an instance of IpoptSolver_NLP which lives for
// the duration of the Solve() call.
class IpoptSolver_NLP : public Ipopt::TNLP {
 public:
  explicit IpoptSolver_NLP(const MathematicalProgram& problem,
                           const Eigen::VectorXd& x_init,
                           MathematicalProgramResult* result)
      : problem_(&problem), x_init_{x_init}, result_(result) {}

  virtual ~IpoptSolver_NLP() {}

  virtual bool get_nlp_info(
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Index& n, Index& m, Index& nnz_jac_g,
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = problem_->num_vars();

    // The IPOPT interface defines eval_f() and eval_grad_f() as
    // outputting a single number for the result, and the size of the
    // output gradient array at the same order as the x variables.
    // Initialize the cost cache with those dimensions.
    cost_cache_.reset(new ResultCache(n, 1, n));

    m = 0;
    nnz_jac_g = 0;
    Index num_grad = 0;
    for (const auto& c : problem_->generic_constraints()) {
      m += GetNumGradients(*(c.evaluator()), c.variables().rows(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      m += GetNumGradients(*(c.evaluator()), c.variables().rows(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      m += GetNumGradients(*(c.evaluator()), c.variables().rows(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->linear_constraints()) {
      m += GetNumGradients(*(c.evaluator()), c.variables().rows(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      m += GetNumGradients(*(c.evaluator()), c.variables().rows(), &num_grad);
      nnz_jac_g += num_grad;
    }

    constraint_cache_.reset(new ResultCache(n, m, nnz_jac_g));

    nnz_h_lag = 0;
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                               Number* g_l, Number* g_u) {
    unused(m);

    DRAKE_ASSERT(n == static_cast<Index>(problem_->num_vars()));
    for (Index i = 0; i < n; i++) {
      x_l[i] = -std::numeric_limits<double>::infinity();
      x_u[i] = std::numeric_limits<double>::infinity();
    }

    for (auto const& binding : problem_->bounding_box_constraints()) {
      const auto& c = binding.evaluator();
      const auto& lower_bound = c->lower_bound();
      const auto& upper_bound = c->upper_bound();
      for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
        const int idx =
            problem_->FindDecisionVariableIndex(binding.variables()(k));
        x_l[idx] = std::max(lower_bound(k), x_l[idx]);
        x_u[idx] = std::min(upper_bound(k), x_u[idx]);
      }
    }
    // Set the indices of the dual variables corresponding to each bounding box
    // constraint. Ipopt stores the dual variables in z_L and z_U.
    for (const auto& binding : problem_->bounding_box_constraints()) {
      std::vector<int> lower_dual_indices(
          binding.evaluator()->num_constraints(), -1);
      std::vector<int> upper_dual_indices(
          binding.evaluator()->num_constraints(), -1);
      for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
        const int idx =
            problem_->FindDecisionVariableIndex(binding.variables()(k));
        if (x_l[idx] == binding.evaluator()->lower_bound()(k)) {
          lower_dual_indices[k] = idx;
        }
        if (x_u[idx] == binding.evaluator()->upper_bound()(k)) {
          upper_dual_indices[k] = idx;
        }
      }
      bb_con_dual_variable_indices_.emplace(
          binding, std::make_pair(lower_dual_indices, upper_dual_indices));
    }

    size_t constraint_idx = 0;  // offset into g_l and g_u output arrays
    for (const auto& c : problem_->generic_constraints()) {
      SetConstraintDualVariableIndex(c, constraint_idx,
                                     &constraint_dual_start_index_);
      constraint_idx += GetConstraintBounds(
          *(c.evaluator()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      SetConstraintDualVariableIndex(c, constraint_idx,
                                     &constraint_dual_start_index_);
      constraint_idx += GetConstraintBounds(
          *(c.evaluator()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      SetConstraintDualVariableIndex(c, constraint_idx,
                                     &constraint_dual_start_index_);
      constraint_idx += GetConstraintBounds(
          *(c.evaluator()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->linear_constraints()) {
      SetConstraintDualVariableIndex(c, constraint_idx,
                                     &constraint_dual_start_index_);
      constraint_idx += GetConstraintBounds(
          *(c.evaluator()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      SetConstraintDualVariableIndex(c, constraint_idx,
                                     &constraint_dual_start_index_);
      constraint_idx += GetConstraintBounds(
          *(c.evaluator()), g_l + constraint_idx, g_u + constraint_idx);
    }
    return true;
  }

  virtual bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                                  Number* z_L, Number* z_U, Index m,
                                  bool init_lambda, Number* lambda) {
    unused(z_L, z_U, m, lambda);

    if (init_x) {
      DRAKE_ASSERT(x_init_.size() == n);
      for (Index i = 0; i < n; i++) {
        if (!std::isnan(x_init_[i])) {
          x[i] = x_init_[i];
        } else {
          x[i] = 0.0;
        }
      }
    }

    // We don't currently use any solver options which require
    // populating z_L, z_U or lambda.  Assert that IPOPT doesn't
    // expect us to in case any such options get turned on.
    DRAKE_ASSERT(!init_z);
    DRAKE_ASSERT(!init_lambda);
    return true;
  }

  // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
    if (new_x || !cost_cache_->is_x_equal(n, x)) {
      EvaluateCosts(n, x);
    }

    DRAKE_ASSERT(cost_cache_->result.size() == 1);
    obj_value = cost_cache_->result[0];
    return true;
  }

  virtual bool eval_grad_f(Index n, const Number* x, bool new_x,
                           Number* grad_f) {
    if (new_x || !cost_cache_->is_x_equal(n, x)) {
      EvaluateCosts(n, x);
    }

    ResultCache::Extract(cost_cache_->grad, n, grad_f);
    return true;
  }

  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m,
                      Number* g) {
    if (new_x || !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x, false);
    }

    ResultCache::Extract(constraint_cache_->result, m, g);
    return true;
  }

  virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                          Index nele_jac, Index* iRow, Index* jCol,
                          Number* values) {
    unused(m);

    if (values == nullptr) {
      DRAKE_ASSERT(iRow != nullptr);
      DRAKE_ASSERT(jCol != nullptr);

      int constraint_idx = 0;  // Passed into GetGradientMatrix as
                                  // the starting row number for the
                                  // constraint being described.
      int grad_idx = 0;        // Offset into iRow, jCol output variables.
                                  // Incremented by the number of triplets
                                  // populated by each call to
                                  // GetGradientMatrix.
      for (const auto& c : problem_->generic_constraints()) {
        grad_idx +=
            GetGradientMatrix(*problem_, *(c.evaluator()), c.variables(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.evaluator()->num_constraints();
      }
      for (const auto& c : problem_->lorentz_cone_constraints()) {
        grad_idx +=
            GetGradientMatrix(*problem_, *(c.evaluator()), c.variables(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.evaluator()->num_constraints();
      }
      for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
        grad_idx +=
            GetGradientMatrix(*problem_, *(c.evaluator()), c.variables(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.evaluator()->num_constraints();
      }
      for (const auto& c : problem_->linear_constraints()) {
        grad_idx +=
            GetGradientMatrix(*problem_, *(c.evaluator()), c.variables(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.evaluator()->num_constraints();
      }
      for (const auto& c : problem_->linear_equality_constraints()) {
        grad_idx +=
            GetGradientMatrix(*problem_, *(c.evaluator()), c.variables(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.evaluator()->num_constraints();
      }
      DRAKE_ASSERT(static_cast<Index>(grad_idx) == nele_jac);
      return true;
    }

    DRAKE_ASSERT(iRow == nullptr);
    DRAKE_ASSERT(jCol == nullptr);

    // We're being asked for the actual values.
    if (new_x || !constraint_cache_->grad_valid ||
        !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x, true);
    }

    ResultCache::Extract(constraint_cache_->grad, nele_jac, values);
    return true;
  }

  virtual void finalize_solution(SolverReturn status, Index n, const Number* x,
                                 const Number* z_L, const Number* z_U, Index m,
                                 const Number* g, const Number* lambda,
                                 Number obj_value, const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq) {
    unused(ip_data, ip_cq);

    IpoptSolverDetails& solver_details =
        result_->SetSolverDetailsType<IpoptSolverDetails>();
    solver_details.status = status;
    solver_details.z_L = Eigen::Map<const Eigen::VectorXd>(z_L, n);
    solver_details.z_U = Eigen::Map<const Eigen::VectorXd>(z_U, n);
    solver_details.g = Eigen::Map<const Eigen::VectorXd>(g, m);
    solver_details.lambda = Eigen::Map<const Eigen::VectorXd>(lambda, m);

    SetBoundingBoxConstraintDualSolution(
        *problem_, z_L, z_U, bb_con_dual_variable_indices_, result_);
    SetAllConstraintDualSolution(*problem_, solver_details.lambda,
                                 constraint_dual_start_index_, result_);

    result_->set_solution_result(SolutionResult::kUnknownError);
    switch (status) {
      case Ipopt::SUCCESS: {
        result_->set_solution_result(SolutionResult::kSolutionFound);
        break;
      }
      case Ipopt::STOP_AT_ACCEPTABLE_POINT: {
        // This case happens because the user requested more lenient solution
        // acceptability criteria so it is counted as solved.
        result_->set_solution_result(SolutionResult::kSolutionFound);
        break;
      }
      case Ipopt::LOCAL_INFEASIBILITY: {
        result_->set_solution_result(SolutionResult::kInfeasibleConstraints);
        break;
      }
      case Ipopt::DIVERGING_ITERATES: {
        result_->set_solution_result(SolutionResult::kUnbounded);
        result_->set_optimal_cost(MathematicalProgram::kUnboundedCost);
        break;
      }
      case Ipopt::MAXITER_EXCEEDED: {
        result_->set_solution_result(SolutionResult::kIterationLimit);
        drake::log()->warn(
            "IPOPT terminated after exceeding the maximum iteration limit.  "
            "Hint: Remember that IPOPT is an interior-point method "
            "and performs badly if any variables are unbounded.");
        break;
      }
      default: {
        result_->set_solution_result(SolutionResult::kUnknownError);
        break;
      }
    }

    Eigen::VectorXd solution(n);
    for (Index i = 0; i < n; i++) {
      solution(i) = x[i];
    }
    result_->set_x_val(solution.cast<double>());
    if (result_->get_solution_result() != SolutionResult::kUnbounded) {
      result_->set_optimal_cost(obj_value);
    }
  }

 private:
  void EvaluateCosts(Index n, const Number* x) {
    const Eigen::VectorXd xvec = MakeEigenVector(n, x);

    problem_->EvalVisualizationCallbacks(xvec);

    AutoDiffVecXd ty(1);
    Eigen::VectorXd this_x;

    cost_cache_->SetX(n, x);
    cost_cache_->result[0] = 0;
    cost_cache_->grad.assign(n, 0);

    for (auto const& binding : problem_->GetAllCosts()) {
      int num_v_variables = binding.GetNumElements();
      this_x.resize(num_v_variables);
      for (int i = 0; i < num_v_variables; ++i) {
        this_x(i) =
            xvec(problem_->FindDecisionVariableIndex(binding.variables()(i)));
      }

      binding.evaluator()->Eval(math::InitializeAutoDiff(this_x), &ty);

      cost_cache_->result[0] += ty(0).value();

      if (ty(0).derivatives().size() > 0) {
        for (int j = 0; j < num_v_variables; ++j) {
          const size_t vj_index =
              problem_->FindDecisionVariableIndex(binding.variables()(j));
          cost_cache_->grad[vj_index] += ty(0).derivatives()(j);
        }
      }
      cost_cache_->grad_valid = true;

      // We do not need to add code for ty(0).derivatives().size() == 0, since
      // cost_cache_->grad would be unchanged if the derivative has zero size.
    }
  }

  void EvaluateConstraints(Index n, const Number* x, bool eval_gradient) {
    const Eigen::VectorXd xvec = MakeEigenVector(n, x);

    constraint_cache_->SetX(n, x);
    Number* result = constraint_cache_->result.data();
    Number* grad = eval_gradient ? constraint_cache_->grad.data() : nullptr;

    for (const auto& c : problem_->generic_constraints()) {
      grad += EvaluateConstraint(*problem_, xvec, (*c.evaluator()),
                                 c.variables(), result, grad);
      result += c.evaluator()->num_constraints();
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      grad += EvaluateConstraint(*problem_, xvec, (*c.evaluator()),
                                 c.variables(), result, grad);
      result += c.evaluator()->num_constraints();
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      grad += EvaluateConstraint(*problem_, xvec, (*c.evaluator()),
                                 c.variables(), result, grad);
      result += c.evaluator()->num_constraints();
    }
    for (const auto& c : problem_->linear_constraints()) {
      grad += EvaluateConstraint(*problem_, xvec, (*c.evaluator()),
                                 c.variables(), result, grad);
      result += c.evaluator()->num_constraints();
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      grad += EvaluateConstraint(*problem_, xvec, (*c.evaluator()),
                                 c.variables(), result, grad);
      result += c.evaluator()->num_constraints();
    }

    if (eval_gradient) {
      constraint_cache_->grad_valid = true;
    }
  }

  const MathematicalProgram* const problem_;
  std::unique_ptr<ResultCache> cost_cache_;
  std::unique_ptr<ResultCache> constraint_cache_;
  Eigen::VectorXd x_init_;
  MathematicalProgramResult* const result_;
  // bb_con_dual_variable_indices_[constraint] maps the bounding box constraint
  // to the indices of its dual variables (one for lower bound and one for upper
  // bound). If this constraint doesn't have a dual variable (because the bound
  // is looser than some other bounding box constraint, hence this constraint
  // can never be active), then the index is set to -1.
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     std::pair<std::vector<int>, std::vector<int>>>
      bb_con_dual_variable_indices_;
  // constraint_dual_start_index_[constraint] stores the starting index of the
  // corresponding dual variables.
  std::unordered_map<Binding<Constraint>, int> constraint_dual_start_index_;
};

template <typename T>
bool HasOptionWithKey(const SolverOptions& solver_options,
                      const std::string& key) {
  return solver_options.GetOptions<T>(IpoptSolver::id()).count(key) > 0;
}

/**
 * If the key has been set in ipopt_options, then this function is an no-op.
 * Otherwise set this key with default_val.
 */
template <typename T>
void SetIpoptOptionsHelper(const std::string& key, const T& default_val,
                           SolverOptions* ipopt_options) {
  if (!HasOptionWithKey<T>(*ipopt_options, key)) {
    ipopt_options->SetOption(IpoptSolver::id(), key, default_val);
  }
}

void SetIpoptOptions(const MathematicalProgram& prog,
                     const std::optional<SolverOptions>& user_solver_options,
                     Ipopt::IpoptApplication* app) {
  SolverOptions merged_solver_options = user_solver_options.has_value()
                                            ? user_solver_options.value()
                                            : SolverOptions();
  merged_solver_options.Merge(prog.solver_options());

  // The default tolerance.
  const double tol = 1.05e-10;  // Note: SNOPT is only 1e-6, but in #3712 we
  // diagnosed that the CompareMatrices tolerance needed to be the sqrt of the
  // constr_viol_tol
  SetIpoptOptionsHelper<double>("tol", tol, &merged_solver_options);
  SetIpoptOptionsHelper<double>("constr_viol_tol", tol, &merged_solver_options);
  SetIpoptOptionsHelper<double>("acceptable_tol", tol, &merged_solver_options);
  SetIpoptOptionsHelper<double>("acceptable_constr_viol_tol", tol,
                                &merged_solver_options);
  SetIpoptOptionsHelper<std::string>("hessian_approximation", "limited-memory",
                                     &merged_solver_options);
  // Note: 0<= print_level <= 12, with higher numbers more verbose.  4 is very
  // useful for debugging. Otherwise, we default to printing nothing. The user
  // can always select an arbitrary print level, by setting the ipopt value
  // directly in the solver options.
  int common_print_level = merged_solver_options.get_print_to_console() ? 4 : 0;
  SetIpoptOptionsHelper<int>("print_level", common_print_level,
                             &merged_solver_options);

  const auto& ipopt_options_double =
      merged_solver_options.GetOptionsDouble(IpoptSolver::id());
  const auto& ipopt_options_str =
      merged_solver_options.GetOptionsStr(IpoptSolver::id());
  const auto& ipopt_options_int =
      merged_solver_options.GetOptionsInt(IpoptSolver::id());
  for (const auto& it : ipopt_options_double) {
    app->Options()->SetNumericValue(it.first, it.second);
  }

  for (const auto& it : ipopt_options_int) {
    app->Options()->SetIntegerValue(it.first, it.second);
  }

  // The default linear solver is MA27, but it is not freely redistributable so
  // we cannot use it. MUMPS is the only compatible linear solver guaranteed to
  // be available on both macOS and Ubuntu. In versions of IPOPT prior to 3.13,
  // it would correctly determine that MUMPS was the only available solver, but
  // its behavior changed to instead error having unsuccessfully tried to dlopen
  // a nonexistent hsl library that would contain MA27.
  app->Options()->SetStringValue("linear_solver", "mumps");

  app->Options()->SetStringValue("sb", "yes");  // Turn off the banner.
  for (const auto& it : ipopt_options_str) {
    app->Options()->SetStringValue(it.first, it.second);
  }
}

}  // namespace

const char* IpoptSolverDetails::ConvertStatusToString() const {
  switch (status) {
    case Ipopt::SolverReturn::SUCCESS: {
      return "Success";
    }
    case Ipopt::SolverReturn::MAXITER_EXCEEDED: {
      return "Max iteration exceeded";
    }
    case Ipopt::SolverReturn::CPUTIME_EXCEEDED: {
      return "CPU time exceeded";
    }
    case Ipopt::SolverReturn::STOP_AT_TINY_STEP: {
      return "Stop at tiny step";
    }
    case Ipopt::SolverReturn::STOP_AT_ACCEPTABLE_POINT: {
      return "Stop at acceptable point";
    }
    case Ipopt::SolverReturn::LOCAL_INFEASIBILITY: {
      return "Local infeasibility";
    }
    case Ipopt::SolverReturn::USER_REQUESTED_STOP: {
      return "User requested stop";
    }
    case Ipopt::SolverReturn::FEASIBLE_POINT_FOUND: {
      return "Feasible point found";
    }
    case Ipopt::SolverReturn::DIVERGING_ITERATES: {
      return "Divergent iterates";
    }
    case Ipopt::SolverReturn::RESTORATION_FAILURE: {
      return "Restoration failure";
    }
    case Ipopt::SolverReturn::ERROR_IN_STEP_COMPUTATION: {
      return "Error in step computation";
    }
    case Ipopt::SolverReturn::INVALID_NUMBER_DETECTED: {
      return "Invalid number detected";
    }
    case Ipopt::SolverReturn::TOO_FEW_DEGREES_OF_FREEDOM: {
      return "Too few degrees of freedom";
    }
    case Ipopt::SolverReturn::INVALID_OPTION: {
      return "Invalid option";
    }
    case Ipopt::SolverReturn::OUT_OF_MEMORY: {
      return "Out of memory";
    }
    case Ipopt::SolverReturn::INTERNAL_ERROR: {
      return "Internal error";
    }
    case Ipopt::SolverReturn::UNASSIGNED: {
      return "Unassigned";
    }
  }
  return "Unknown enumerated SolverReturn value.";
}

bool IpoptSolver::is_available() { return true; }

void IpoptSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "IpoptSolver doesn't support the feature of variable scaling.");
  }

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  SetIpoptOptions(prog, merged_options, &(*app));

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    result->set_solution_result(SolutionResult::kInvalidInput);
    return;
  }

  Ipopt::SmartPtr<IpoptSolver_NLP> nlp =
      new IpoptSolver_NLP(prog, initial_guess, result);
  status = app->OptimizeTNLP(nlp);
}

}  // namespace solvers
}  // namespace drake
