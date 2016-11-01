#include "drake/solvers/ipopt_solver.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

// TODO(sam.creasey) figure out how to get the real pkgconfig cflags
#define HAVE_CSTDDEF
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#undef HAVE_CSTDDEF

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff.h"

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
size_t GetConstraintBounds(const Constraint& c, Number* lb, Number* ub) {
  const Eigen::VectorXd& lower_bound = c.lower_bound();
  const Eigen::VectorXd& upper_bound = c.upper_bound();
  for (size_t i = 0; i < c.num_constraints(); i++) {
    lb[i] = lower_bound(i);
    ub[i] = upper_bound(i);
  }

  return c.num_constraints();
}

/// @param[out] num_grad number of gradients
/// @return number of constraints
size_t GetNumGradients(const Constraint& c, const VariableList& variable_list,
                       Index* num_grad) {
  size_t var_count = 0;
  for (const DecisionVariableView& v : variable_list) {
    var_count += v.size();
  }

  const size_t num_constraints = c.num_constraints();
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
size_t GetGradientMatrix(const Constraint& c, const VariableList& variable_list,
                         Index constraint_idx, Index* iRow, Index* jCol) {
  const size_t m = c.num_constraints();
  size_t grad_index = 0;

  for (const DecisionVariableView& v : variable_list) {
    for (size_t i = 0; i < m; i++) {
      for (size_t j = 0; j < v.size(); j++) {
        iRow[grad_index] = constraint_idx + i;
        jCol[grad_index] = v.index() + j;
        grad_index++;
      }
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
/// @return number of gradient entries populated
size_t EvaluateConstraint(const Eigen::VectorXd& xvec, const Constraint& c,
                          const VariableList& variable_list, Number* result,
                          Number* grad) {
  // For constraints which don't use all of the variables in the X
  // input, extract a subset into the TaylorVecXd this_x to evaluate
  // the constraint (we actually do this for all constraints.  One
  // potential optimization might be to detect if the initial "tx" has
  // the correct geometry (e.g. the constraint uses all decision
  // variables in the same order they appear in xvec), but this is not
  // currently done).
  size_t var_count = 0;
  for (const DecisionVariableView& v : variable_list) {
    var_count += v.size();
  }

  auto tx = math::initializeAutoDiff(xvec);
  TaylorVecXd this_x(var_count);
  size_t index = 0;
  for (const DecisionVariableView& v : variable_list) {
    this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
    index += v.size();
  }

  TaylorVecXd ty(c.num_constraints());
  c.Eval(this_x, ty);

  // Store the results.  Since IPOPT directly knows the bounds of the
  // constraint, we don't need to apply any bounding information here.
  for (size_t i = 0; i < c.num_constraints(); i++) {
    result[i] = ty(i).value();
  }

  // Extract the appropriate derivatives from our result into the
  // gradient array.  Like above, we need to use variable_list to
  // figure out where the derivatives we actually care about are
  // located.
  size_t grad_idx = 0;
  for (const DecisionVariableView& v : variable_list) {
    for (size_t i = 0; i < c.num_constraints(); i++) {
      for (size_t j = v.index(); j < v.index() + v.size(); j++) {
        grad[grad_idx++] = ty(i).derivatives()(j);
      }
    }
  }
  return grad_idx;
}

// IPOPT uses separate callbacks to get the result and the gradients.
// Since Drake's eval() functions emit both of these at once, cache
// the result for IPOPT.
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

  std::vector<Number> x;
  std::vector<Number> result;
  std::vector<Number> grad;
};

// The C++ interface for IPOPT is described here:
// http://www.coin-or.org/Ipopt/documentation/node23.html
//
// IPOPT provides a pure(-ish) virtual base class which you have to
// implement a concrete version of as the solver interface.
// IpoptSolver creates an instance of IpoptSolver_NLP which lives for
// the duration of the Solve() call.
class IpoptSolver_NLP : public Ipopt::TNLP {
 public:
  explicit IpoptSolver_NLP(MathematicalProgram* problem)
      : problem_(problem), result_(SolutionResult::kUnknownError) {}

  virtual ~IpoptSolver_NLP() {}

  virtual bool get_nlp_info(
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Index& n, Index& m, Index& nnz_jac_g,
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = problem_->num_vars();

    // The IPOPT interface defines eval_f() and eval_grad_f() as
    // ouputting a single Number for the result, and the size of the
    // output gradient array at the same order as the x variables.
    // Initialize the cost cache with those dimensions.
    cost_cache_.reset(new ResultCache(n, 1, n));

    m = 0;
    nnz_jac_g = 0;
    Index num_grad = 0;
    for (const auto& c : problem_->generic_constraints()) {
      m += GetNumGradients(*(c.constraint()), c.variable_list(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      m += GetNumGradients(*(c.constraint()), c.variable_list(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      m += GetNumGradients(*(c.constraint()), c.variable_list(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->linear_constraints()) {
      m += GetNumGradients(*(c.constraint()), c.variable_list(), &num_grad);
      nnz_jac_g += num_grad;
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      m += GetNumGradients(*(c.constraint()), c.variable_list(), &num_grad);
      nnz_jac_g += num_grad;
    }

    constraint_cache_.reset(new ResultCache(n, m, nnz_jac_g));

    nnz_h_lag = 0;
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                               Number* g_l, Number* g_u) {
    DRAKE_ASSERT(n == static_cast<Index>(problem_->num_vars()));
    for (Index i = 0; i < n; i++) {
      x_l[i] = -std::numeric_limits<double>::infinity();
      x_u[i] = std::numeric_limits<double>::infinity();
    }

    for (auto const& binding : problem_->bounding_box_constraints()) {
      const auto& c = binding.constraint();
      const auto& lower_bound = c->lower_bound();
      const auto& upper_bound = c->upper_bound();
      int var_count = 0;
      for (const DecisionVariableView& v : binding.variable_list()) {
        for (size_t k = 0; k < v.size(); k++) {
          const int idx = v.index() + k;
          x_l[idx] = std::max(lower_bound(var_count), x_l[idx]);
          x_u[idx] = std::min(upper_bound(var_count), x_u[idx]);
          ++var_count;
        }
      }
    }

    size_t constraint_idx = 0;  // offset into g_l and g_u output arrays
    for (const auto& c : problem_->generic_constraints()) {
      constraint_idx += GetConstraintBounds(
          *(c.constraint()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      constraint_idx += GetConstraintBounds(
          *(c.constraint()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      constraint_idx += GetConstraintBounds(
          *(c.constraint()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->linear_constraints()) {
      constraint_idx += GetConstraintBounds(
          *(c.constraint()), g_l + constraint_idx, g_u + constraint_idx);
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      constraint_idx += GetConstraintBounds(
          *(c.constraint()), g_l + constraint_idx, g_u + constraint_idx);
    }
    return true;
  }

  virtual bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                                  Number* z_L, Number* z_U, Index m,
                                  bool init_lambda, Number* lambda) {
    if (init_x) {
      const Eigen::VectorXd& initial_guess = problem_->initial_guess();
      DRAKE_ASSERT(initial_guess.size() == n);
      for (Index i = 0; i < n; i++) {
        x[i] = initial_guess[i];
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

    DRAKE_ASSERT(static_cast<Index>(cost_cache_->grad.size()) == n);
    std::memcpy(grad_f, cost_cache_->grad.data(), n * sizeof(Number));
    return true;
  }

  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m,
                      Number* g) {
    if (new_x || !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x);
    }

    DRAKE_ASSERT(static_cast<Index>(constraint_cache_->result.size()) == m);
    std::memcpy(g, constraint_cache_->result.data(), m * sizeof(Number));
    return true;
  }

  virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                          Index nele_jac, Index* iRow, Index* jCol,
                          Number* values) {
    if (values == nullptr) {
      DRAKE_ASSERT(iRow != nullptr);
      DRAKE_ASSERT(jCol != nullptr);

      size_t constraint_idx = 0;  // Passed into GetGradientMatrix as
                                  // the starting row number for the
                                  // constraint being described.
      size_t grad_idx = 0;        // Offset into iRow, jCol output variables.
                                  // Incremented by the number of triplets
                                  // populated by each call to
                                  // GetGradientMatrix.
      for (const auto& c : problem_->generic_constraints()) {
        grad_idx +=
            GetGradientMatrix(*(c.constraint()), c.variable_list(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->lorentz_cone_constraints()) {
        grad_idx +=
            GetGradientMatrix(*(c.constraint()), c.variable_list(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
        grad_idx +=
            GetGradientMatrix(*(c.constraint()), c.variable_list(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->linear_constraints()) {
        grad_idx +=
            GetGradientMatrix(*(c.constraint()), c.variable_list(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->linear_equality_constraints()) {
        grad_idx +=
            GetGradientMatrix(*(c.constraint()), c.variable_list(),
                              constraint_idx, iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      DRAKE_ASSERT(static_cast<Index>(grad_idx) == nele_jac);
      return true;
    }

    DRAKE_ASSERT(iRow == nullptr);
    DRAKE_ASSERT(jCol == nullptr);

    // We're being asked for the actual values.
    if (new_x || !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x);
    }

    DRAKE_ASSERT(static_cast<Index>(constraint_cache_->grad.size()) ==
                 nele_jac);
    std::memcpy(values, constraint_cache_->grad.data(),
                nele_jac * sizeof(Number));
    return true;
  }

  virtual void finalize_solution(SolverReturn status, Index n, const Number* x,
                                 const Number* z_L, const Number* z_U, Index m,
                                 const Number* g, const Number* lambda,
                                 Number obj_value, const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq) {
    problem_->SetSolverResult("IPOPT", status);

    switch (status) {
      case Ipopt::SUCCESS: {
        result_ = SolutionResult::kSolutionFound;
        break;
      }
      case Ipopt::LOCAL_INFEASIBILITY: {
        result_ = SolutionResult::kInfeasibleConstraints;
        break;
      }
      default: {
        result_ = SolutionResult::kUnknownError;
        break;
      }
    }

    Eigen::VectorXd solution(n);
    for (Index i = 0; i < n; i++) {
      solution(i) = x[i];
    }
    problem_->SetDecisionVariableValues(solution);
  }

  SolutionResult result() const { return result_; }

 private:
  void EvaluateCosts(Index n, const Number* x) {
    const Eigen::VectorXd xvec = MakeEigenVector(n, x);

    auto tx = math::initializeAutoDiff(xvec);
    TaylorVecXd ty(1);
    TaylorVecXd this_x;

    memcpy(cost_cache_->x.data(), x, n * sizeof(Number));
    cost_cache_->result[0] = 0;
    cost_cache_->grad.assign(n, 0);

    for (auto const& binding : problem_->GetAllCosts()) {
      size_t index = 0;
      for (const DecisionVariableView& v : binding.variable_list()) {
        this_x.conservativeResize(index + v.size());
        this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
        index += v.size();
      }

      binding.constraint()->Eval(this_x, ty);

      cost_cache_->result[0] += ty(0).value();
      for (const DecisionVariableView& v : binding.variable_list()) {
        for (size_t j = v.index(); j < v.index() + v.size(); j++) {
          cost_cache_->grad[j] += ty(0).derivatives()(j);
        }
      }
    }
  }

  void EvaluateConstraints(Index n, const Number* x) {
    const Eigen::VectorXd xvec = MakeEigenVector(n, x);

    memcpy(constraint_cache_->x.data(), x, n * sizeof(Number));
    Number* result = constraint_cache_->result.data();
    Number* grad = constraint_cache_->grad.data();

    for (const auto& c : problem_->generic_constraints()) {
      grad += EvaluateConstraint(xvec, (*c.constraint()), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->lorentz_cone_constraints()) {
      grad += EvaluateConstraint(xvec, (*c.constraint()), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->rotated_lorentz_cone_constraints()) {
      grad += EvaluateConstraint(xvec, (*c.constraint()), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->linear_constraints()) {
      grad += EvaluateConstraint(xvec, (*c.constraint()), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      grad += EvaluateConstraint(xvec, (*c.constraint()), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
  }

  MathematicalProgram* const problem_;
  std::unique_ptr<ResultCache> cost_cache_;
  std::unique_ptr<ResultCache> constraint_cache_;
  SolutionResult result_;
};

}  // namespace

bool IpoptSolver::available() const { return true; }

SolutionResult IpoptSolver::Solve(MathematicalProgram& prog) const {
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  const double tol = 1e-10;  // Note: SNOPT is only 1e-6, but in #3712 we
  // diagnosed that the CompareMatrices tolerance needed to be the sqrt of the
  // constr_viol_tol
  app->Options()->SetNumericValue("tol", tol);
  app->Options()->SetNumericValue("constr_viol_tol", tol);
  app->Options()->SetNumericValue("acceptable_tol", tol);
  app->Options()->SetNumericValue("acceptable_constr_viol_tol", tol);
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("print_level", 2);

  for (const auto& it : prog.GetSolverOptionsDouble("IPOPT")) {
    app->Options()->SetNumericValue(it.first, it.second);
  }

  for (const auto& it : prog.GetSolverOptionsInt("IPOPT")) {
    app->Options()->SetIntegerValue(it.first, it.second);
  }

  for (const auto& it : prog.GetSolverOptionsStr("IPOPT")) {
    app->Options()->SetStringValue(it.first, it.second);
  }

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    return SolutionResult::kInvalidInput;
  }

  Ipopt::SmartPtr<IpoptSolver_NLP> nlp = new IpoptSolver_NLP(&prog);
  status = app->OptimizeTNLP(nlp);

  return nlp->result();
}

}  // namespace solvers
}  // namespace drake
