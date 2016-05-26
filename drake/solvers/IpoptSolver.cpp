
#include "drake/solvers/IpoptSolver.h"

#include <cstring>
#include <memory>
#include <vector>

// TODO(sam.creasey) figure out how to get the real pkgconfig cflags
#define HAVE_CSTDDEF
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#undef HAVE_CSTDDEF

#include "drake/core/Gradient.h"
#include "drake/solvers/Optimization.h"

using Ipopt::Index;
using Ipopt::IpoptCalculatedQuantities;
using Ipopt::IpoptData;
using Ipopt::Number;
using Ipopt::SolverReturn;

using Drake::DecisionVariableView;
using Drake::OptimizationProblem;
using Drake::TaylorVecXd;

namespace drake {
namespace solvers {
namespace {

size_t GetConstraintBounds(
    const Drake::Constraint& c, Number* lb, Number* ub) {
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
size_t GetNumGradients(
    const Drake::Constraint& c, const Drake::VariableList& variable_list,
    Index* num_grad) {
  const size_t m = c.num_constraints();

  size_t var_count = 0;
  for (const DecisionVariableView& v : variable_list) {
    var_count += v.size();
  }
  *num_grad = m * var_count;
  return m;
}

size_t GetGradientMatrix(
    const Drake::Constraint& c, const Drake::VariableList& variable_list,
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
  for (size_t i = 0; i < n; i++) {
    xvec[i] = x[i];
  }
  return xvec;
}

/// @return number of gradient entries populated
size_t EvaluateConstraint(
    const Eigen::VectorXd& xvec,
    const Drake::Constraint& c, const Drake::VariableList& variable_list,
    Number* result, Number* grad) {

  size_t var_count = 0;
  for (const DecisionVariableView& v : variable_list) {
    var_count += v.size();
  }

  auto tx = Drake::initializeAutoDiff(xvec);
  TaylorVecXd this_x(var_count);
  size_t index = 0;
  for (const DecisionVariableView& v : variable_list) {
    this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
    index += v.size();
  }

  TaylorVecXd ty(c.num_constraints());
  c.eval(this_x, ty);
  for (size_t i = 0; i < c.num_constraints(); i++) {
    result[i] = ty(i).value();
  }

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

struct ResultCache {
  ResultCache(size_t x_size, size_t result_size, size_t grad_size) {
    // The choice of infinity as the default value below is arbitrary.
    x.resize(x_size, std::numeric_limits<double>::infinity());
    result.resize(result_size, std::numeric_limits<double>::infinity());
    grad.resize(grad_size, std::numeric_limits<double>::infinity());
  }

  bool is_x_equal(Index n, const Number* x_in) {
    assert(n == x.size());
    return !std::memcmp(x.data(), x_in, x.size() * sizeof(Number));
  }

  std::vector<Number> x;
  std::vector<Number> result;
  std::vector<Number> grad;
};

class IpoptSolver_NLP : public Ipopt::TNLP {
 public:
  explicit IpoptSolver_NLP(OptimizationProblem* problem)
      : problem_(problem),
        result_(SolutionResult::kUnknownError) {}

  virtual ~IpoptSolver_NLP() {}

  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = problem_->num_vars();

    cost_cache_.reset(new ResultCache(n, 1, n));

    m = 0;
    nnz_jac_g = 0;
    Index num_grad = 0;
    for (const auto& c : problem_->generic_constraints()) {
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

  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u) {
    assert(n == problem_->num_vars());
    for (Index i = 0; i < n; i++) {
      x_l[i] = -std::numeric_limits<double>::infinity();
      x_u[i] = std::numeric_limits<double>::infinity();
    }

    for (auto const& binding : problem_->bounding_box_constraints()) {
      auto const& c = binding.constraint();
      const Eigen::VectorXd& lower_bound = c->lower_bound();
      const Eigen::VectorXd& upper_bound = c->upper_bound();
      for (const DecisionVariableView& v : binding.variable_list()) {
        for (int k = 0; k < v.size(); k++) {
          const int idx = v.index() + k;
          x_l[idx] = std::max(lower_bound(k), x_l[idx]);
          x_u[idx] = std::min(upper_bound(k), x_u[idx]);
        }
      }
    }

    size_t constraint_idx = 0;
    for (const auto& c : problem_->generic_constraints()) {
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

  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda) {
    if (init_x) {
      const Eigen::VectorXd& initial_guess = problem_->initial_guess();
      assert(initial_guess.size() == n);
      for (size_t i = 0; i < n; i++) {
        x[i] = initial_guess[i];
      }
    }

    assert(!init_z);
    assert(!init_lambda);
    return true;
  }

  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
    if (new_x || !cost_cache_->is_x_equal(n, x)) {
      EvaluateCosts(n, x);
    }

    obj_value = cost_cache_->result[0];
    return true;
  }

  virtual bool eval_grad_f(
      Index n, const Number* x, bool new_x, Number* grad_f) {
    if (new_x || !cost_cache_->is_x_equal(n, x)) {
      EvaluateCosts(n, x);
    }

    assert(cost_cache_->grad.size() == n);
    std::memcpy(grad_f, cost_cache_->grad.data(), n * sizeof(Number));
    return true;
  }

  virtual bool eval_g(
      Index n, const Number* x, bool new_x, Index m, Number* g) {
    if (new_x || !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x);
    }

    assert(constraint_cache_->result.size() == m);
    std::memcpy(g, constraint_cache_->result.data(), m * sizeof(Number));
    return true;
  }

  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values) {
    if (values == nullptr) {
      assert(iRow != nullptr);
      assert(jCol != nullptr);

      size_t constraint_idx = 0;
      size_t grad_idx = 0;
      for (const auto& c : problem_->generic_constraints()) {
        grad_idx += GetGradientMatrix(
            *(c.constraint()), c.variable_list(), constraint_idx,
            iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->linear_constraints()) {
        grad_idx += GetGradientMatrix(
            *(c.constraint()), c.variable_list(), constraint_idx,
            iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      for (const auto& c : problem_->linear_equality_constraints()) {
        grad_idx += GetGradientMatrix(
            *(c.constraint()), c.variable_list(), constraint_idx,
            iRow + grad_idx, jCol + grad_idx);
        constraint_idx += c.constraint()->num_constraints();
      }
      assert(grad_idx == nele_jac);
      return true;
    }

    assert(iRow == nullptr);
    assert(jCol == nullptr);

    // We're being asked for the actual values.
    if (new_x || !constraint_cache_->is_x_equal(n, x)) {
      EvaluateConstraints(n, x);
    }

    assert(constraint_cache_->grad.size() == nele_jac);
    std::memcpy(
        values, constraint_cache_->grad.data(), nele_jac * sizeof(Number));
    return true;
  }

  virtual void finalize_solution(
      SolverReturn status,
      Index n, const Number* x, const Number* z_L, const Number* z_U,
      Index m, const Number* g, const Number* lambda,
      Number obj_value,
      const IpoptData* ip_data,
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

    Eigen::VectorXd sol(n);
    for (size_t i = 0; i < n; i++) {
      sol(i) = x[i];
    }
    problem_->SetDecisionVariableValues(sol);
  }

  SolutionResult result() const { return result_; }

 private:
  void EvaluateCosts(Index n, const Number* x) {
    const Eigen::VectorXd xvec = MakeEigenVector(n, x);

    auto tx = Drake::initializeAutoDiff(xvec);
    TaylorVecXd ty(1);
    TaylorVecXd this_x;

    memcpy(cost_cache_->x.data(), x, n * sizeof(Number));
    cost_cache_->result[0] = 0;
    cost_cache_->grad.assign(n, 0);

    for (auto const& binding : problem_->generic_objectives()) {
      size_t index = 0;
      for (const DecisionVariableView& v : binding.variable_list()) {
        this_x.conservativeResize(index + v.size());
        this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
        index += v.size();
      }

      binding.constraint()->eval(this_x, ty);

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
      grad += EvaluateConstraint(xvec, *c.constraint(), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->linear_constraints()) {
      grad += EvaluateConstraint(xvec, *c.constraint(), c.variable_list(),
                         result, grad);
      result += c.constraint()->num_constraints();
    }
    for (const auto& c : problem_->linear_equality_constraints()) {
      grad += EvaluateConstraint(xvec, *c.constraint(), c.variable_list(),
                                 result, grad);
      result += c.constraint()->num_constraints();
    }
  }

  OptimizationProblem* const problem_;
  std::unique_ptr<ResultCache> cost_cache_;
  std::unique_ptr<ResultCache> constraint_cache_;
  SolutionResult result_;
};

}  // end anonymous namespace


bool IpoptSolver::available() const {
  return true;
}

SolutionResult IpoptSolver::Solve(OptimizationProblem &prog) const {
  assert(prog.linear_complementarity_constraints().empty());

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  const double tol = 1e-8;
  app->Options()->SetNumericValue("tol", tol);
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("print_level", 2);

  for (const auto it : prog.GetSolverOptionsDouble("IPOPT")) {
    app->Options()->SetNumericValue(it.first, it.second);
  }

  for (const auto it : prog.GetSolverOptionsInt("IPOPT")) {
    app->Options()->SetIntegerValue(it.first, it.second);
  }

  for (const auto it : prog.GetSolverOptionsStr("IPOPT")) {
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

}  // namespace drake
}  // namespace solvers
