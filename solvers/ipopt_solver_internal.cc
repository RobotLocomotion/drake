#include "drake/solvers/ipopt_solver_internal.h"

#include <algorithm>
#include <limits>
#include <optional>

#include <IpLinearSolvers.h>

#include "drake/common/text_logging.h"

using Ipopt::Index;
using Ipopt::IpoptCalculatedQuantities;
using Ipopt::IpoptData;
using Ipopt::Number;
using Ipopt::SolverReturn;

namespace drake {
namespace solvers {
namespace internal {
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

/// @param[out] num_grad number of gradients
/// @return number of constraints
int GetNumGradients(const Constraint& c, int var_count, Index* num_grad) {
  const int num_constraints = c.num_constraints();
  if (c.gradient_sparsity_pattern().has_value()) {
    *num_grad = c.gradient_sparsity_pattern()->size();
  } else {
    *num_grad = num_constraints * var_count;
  }
  return num_constraints;
}

/// @param[out] num_grad number of gradients
/// @return number of constraints
int GetNumGradients(const LinearConstraint& c, Index* num_grad) {
  const int num_constraints = c.num_constraints();
  *num_grad = c.get_sparse_A().nonZeros();
  return num_constraints;
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
  for (const auto& binding : prog.quadratic_constraints()) {
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

/// @param constraint_idx The starting row number for the constraint
/// being described.
///
/// Parameters @p iRow and @p jCol are used in the same manner as
/// described in
/// https://coin-or.github.io/Ipopt/classIpopt_1_1TNLP.html#aa4162d052f69d4f9946a42feec012853
/// for the eval_jac_g() function (in the mode where it's requesting the
/// sparsity structure of the Jacobian).  The triplet format is also
/// described in
/// https://coin-or.github.io/Ipopt/IMPL.html#TRIPLET
/// @return the number of row/column pairs filled in.
size_t GetGradientMatrix(
    const MathematicalProgram& prog, const Constraint& c,
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    Index constraint_idx, Index* iRow, Index* jCol) {
  const int m = c.num_constraints();
  size_t grad_index = 0;

  const std::optional<std::vector<std::pair<int, int>>>& sparsity_pattern =
      c.gradient_sparsity_pattern();
  if (sparsity_pattern.has_value()) {
    for (const auto& [row, col] : sparsity_pattern.value()) {
      iRow[grad_index] = constraint_idx + row;
      jCol[grad_index] = prog.FindDecisionVariableIndex(variables(col));
      grad_index++;
    }
  } else {
    for (int i = 0; i < static_cast<int>(m); ++i) {
      for (int j = 0; j < variables.rows(); ++j) {
        iRow[grad_index] = constraint_idx + i;
        jCol[grad_index] = prog.FindDecisionVariableIndex(variables(j));
        grad_index++;
      }
    }
  }

  return grad_index;
}

/// Overloads GetGradientMatrix for linear constraints.
size_t GetGradientMatrix(
    const MathematicalProgram& prog, const LinearConstraint& c,
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    Index constraint_idx, Index* iRow, Index* jCol) {
  size_t grad_index = 0;
  const Eigen::SparseMatrix<double>& A = c.get_sparse_A();
  for (int i = 0; i < A.outerSize(); ++i) {
    // A is in column major, so we iterate through each column by looping over
    // i.
    const int var_index = prog.FindDecisionVariableIndex(variables(i));
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
      iRow[grad_index] = constraint_idx + it.row();
      jCol[grad_index] = var_index;
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
template <typename ConstraintType>
size_t EvaluateConstraint(const MathematicalProgram& prog,
                          const Eigen::VectorXd& xvec,
                          const Binding<ConstraintType>& binding,
                          Number* result, Number* grad) {
  Constraint* c = binding.evaluator().get();

  // For constraints which don't use all of the variables in the X
  // input, extract a subset into the AutoDiffVecXd this_x to evaluate
  // the constraint (we actually do this for all constraints.  One
  // potential optimization might be to detect if the initial "tx" has
  // the correct geometry (e.g. the constraint uses all decision
  // variables in the same order they appear in xvec), but this is not
  // currently done).
  int num_v_variables = binding.variables().rows();
  Eigen::VectorXd this_x(num_v_variables);
  for (int i = 0; i < num_v_variables; ++i) {
    this_x(i) = xvec(prog.FindDecisionVariableIndex(binding.variables()(i)));
  }

  if (!grad) {
    // We don't want the gradient info, so just call the VectorXd version of
    // Eval.
    Eigen::VectorXd ty(c->num_constraints());

    c->Eval(this_x, &ty);

    // Store the results.
    for (int i = 0; i < c->num_constraints(); i++) {
      result[i] = ty(i);
    }
    return 0;
  }

  // Run the version which calculates gradients.

  // Leverage the fact that the gradient is equal to the A matrix for linear
  // constraints.
  if constexpr (std::is_same_v<ConstraintType, LinearEqualityConstraint> ||
                std::is_same_v<ConstraintType, LinearConstraint>) {
    auto A = static_cast<ConstraintType*>(c)->get_sparse_A();
    // Verify that A has the proper size.
    DRAKE_ASSERT(A.rows() == c->num_constraints());
    DRAKE_ASSERT(A.cols() == binding.variables().rows());
    // Evaluate the constraint.
    Eigen::VectorXd ty(c->num_constraints());
    c->Eval(this_x, &ty);
    // Set the result and the gradient.
    size_t grad_idx = 0;
    for (int i = 0; i < ty.rows(); i++) {
      result[i] = ty(i);
    }
    for (int i = 0; i < A.outerSize(); ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
        grad[grad_idx++] = it.value();
      }
    }
    return grad_idx;
  } else {
    // Otherwise, use auto-diff.
    AutoDiffVecXd ty(c->num_constraints());
    c->Eval(math::InitializeAutoDiff(this_x), &ty);

    // Store the results.  Since IPOPT directly knows the bounds of the
    // constraint, we don't need to apply any bounding information here.
    for (int i = 0; i < c->num_constraints(); i++) {
      result[i] = ty(i).value();
    }

    // Extract the appropriate derivatives from our result into the
    // gradient array.
    size_t grad_idx = 0;

    DRAKE_ASSERT(ty.rows() == c->num_constraints());
    const std::optional<std::vector<std::pair<int, int>>>& sparsity_pattern =
        binding.evaluator()->gradient_sparsity_pattern();
    if (sparsity_pattern.has_value()) {
      for (const auto& [row, col] : sparsity_pattern.value()) {
        if (ty(row).derivatives().size() > 0) {
          grad[grad_idx++] = ty(row).derivatives()(col);
        } else {
          grad[grad_idx++] = 0;
        }
      }
    } else {
      for (int i = 0; i < ty.rows(); i++) {
        if (ty(i).derivatives().size() > 0) {
          for (int j = 0; j < binding.variables().rows(); j++) {
            grad[grad_idx++] = ty(i).derivatives()(j);
          }
        } else {
          for (int j = 0; j < binding.variables().rows(); j++) {
            grad[grad_idx++] = 0;
          }
        }
      }
    }

    return grad_idx;
  }
}

}  // namespace

ResultCache::ResultCache(size_t x_size, size_t result_size, size_t grad_size) {
  // The choice of infinity as the default value below is arbitrary.
  x.resize(x_size, std::numeric_limits<double>::infinity());
  result.resize(result_size, std::numeric_limits<double>::infinity());
  grad.resize(grad_size, std::numeric_limits<double>::infinity());
}

/// @param n The size of the array located at @p x_in.
bool ResultCache::is_x_equal(Ipopt::Index n, const Ipopt::Number* x_in) {
  DRAKE_ASSERT(n == static_cast<Index>(x.size()));
  return !std::memcmp(x.data(), x_in, x.size() * sizeof(Number));
}

// Sugar to copy an IPOPT bare array into `x`.
void ResultCache::SetX(const Ipopt::Index n, const Ipopt::Number* x_arg) {
  DRAKE_ASSERT(static_cast<Index>(x.size()) == n);
  grad_valid = false;
  if (n == 0) {
    return;
  }
  DRAKE_ASSERT(x_arg != nullptr);
  std::memcpy(x.data(), x_arg, n * sizeof(Number));
}

// Sugar to copy one of our member fields into an IPOPT bare array.
void ResultCache::Extract(const std::vector<Ipopt::Number>& cache_data,
                          const Ipopt::Index dest_size, Ipopt::Number* dest) {
  DRAKE_ASSERT(static_cast<Index>(cache_data.size()) == dest_size);
  if (dest_size == 0) {
    return;
  }
  DRAKE_ASSERT(dest != nullptr);
  std::memcpy(dest, cache_data.data(), dest_size * sizeof(Number));
}

IpoptSolver_NLP::IpoptSolver_NLP(const MathematicalProgram& problem,
                                 const Eigen::VectorXd& x_init,
                                 MathematicalProgramResult* result)
    : problem_(&problem), x_init_{x_init}, result_(result) {}

IpoptSolver_NLP::~IpoptSolver_NLP() = default;

bool IpoptSolver_NLP::get_nlp_info(
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
  for (const auto& c : problem_->quadratic_constraints()) {
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
    m += GetNumGradients(*(c.evaluator()), &num_grad);
    nnz_jac_g += num_grad;
  }
  for (const auto& c : problem_->linear_equality_constraints()) {
    m += GetNumGradients(*(c.evaluator()), &num_grad);
    nnz_jac_g += num_grad;
  }

  constraint_cache_.reset(new ResultCache(n, m, nnz_jac_g));

  nnz_h_lag = 0;
  index_style = C_STYLE;
  return true;
}

bool IpoptSolver_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                      Index m, Number* g_l, Number* g_u) {
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
    std::vector<int> lower_dual_indices(binding.evaluator()->num_constraints(),
                                        -1);
    std::vector<int> upper_dual_indices(binding.evaluator()->num_constraints(),
                                        -1);
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
  for (const auto& c : problem_->quadratic_constraints()) {
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

bool IpoptSolver_NLP::get_starting_point(Index n, bool init_x, Number* x,
                                         bool init_z, Number* z_L, Number* z_U,
                                         Index m, bool init_lambda,
                                         Number* lambda) {
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
bool IpoptSolver_NLP::eval_f(Index n, const Number* x, bool new_x,
                             Number& obj_value) {
  if (new_x || !cost_cache_->is_x_equal(n, x)) {
    EvaluateCosts(n, x);
  }

  DRAKE_ASSERT(cost_cache_->result.size() == 1);
  obj_value = cost_cache_->result[0];
  return true;
}

bool IpoptSolver_NLP::eval_grad_f(Index n, const Number* x, bool new_x,
                                  Number* grad_f) {
  if (new_x || !cost_cache_->is_x_equal(n, x)) {
    EvaluateCosts(n, x);
  }

  ResultCache::Extract(cost_cache_->grad, n, grad_f);
  return true;
}

bool IpoptSolver_NLP::eval_g(Index n, const Number* x, bool new_x, Index m,
                             Number* g) {
  if (new_x || !constraint_cache_->is_x_equal(n, x)) {
    EvaluateConstraints(n, x, false);
  }

  ResultCache::Extract(constraint_cache_->result, m, g);
  return true;
}

bool IpoptSolver_NLP::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
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
    for (const auto& c : problem_->quadratic_constraints()) {
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

void IpoptSolver_NLP::finalize_solution(SolverReturn status, Index n,
                                        const Number* x, const Number* z_L,
                                        const Number* z_U, Index m,
                                        const Number* g, const Number* lambda,
                                        Number obj_value,
                                        const IpoptData* ip_data,
                                        IpoptCalculatedQuantities* ip_cq) {
  unused(ip_data, ip_cq);

  status_ = status;
  z_L_ = Eigen::Map<const Eigen::VectorXd>(z_L, n);
  z_U_ = Eigen::Map<const Eigen::VectorXd>(z_U, n);
  g_ = Eigen::Map<const Eigen::VectorXd>(g, m);
  lambda_ = Eigen::Map<const Eigen::VectorXd>(lambda, m);

  SetBoundingBoxConstraintDualSolution(*problem_, z_L, z_U,
                                       bb_con_dual_variable_indices_, result_);
  SetAllConstraintDualSolution(*problem_, lambda_, constraint_dual_start_index_,
                               result_);

  result_->set_solution_result(SolutionResult::kSolverSpecificError);
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
      result_->set_solution_result(SolutionResult::kSolverSpecificError);
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

void IpoptSolver_NLP::EvaluateCosts(Index n, const Number* x) {
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

void IpoptSolver_NLP::EvaluateConstraints(Index n, const Number* x,
                                          bool eval_gradient) {
  const Eigen::VectorXd xvec = MakeEigenVector(n, x);

  constraint_cache_->SetX(n, x);
  Number* result = constraint_cache_->result.data();
  Number* grad = eval_gradient ? constraint_cache_->grad.data() : nullptr;

  for (const auto& c : problem_->generic_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }
  for (const auto& c : problem_->quadratic_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }
  for (const auto& c : problem_->lorentz_cone_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }
  for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }
  for (const auto& c : problem_->linear_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }
  for (const auto& c : problem_->linear_equality_constraints()) {
    grad += EvaluateConstraint(*problem_, xvec, c, result, grad);
    result += c.evaluator()->num_constraints();
  }

  if (eval_gradient) {
    constraint_cache_->grad_valid = true;
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
