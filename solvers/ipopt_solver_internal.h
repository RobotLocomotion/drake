#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to ipopt_solver_internal_test.

#include <memory>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>

#include "drake/common/drake_export.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
namespace internal {

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
  ResultCache(size_t x_size, size_t result_size, size_t grad_size);

  /// @param n The size of the array located at @p x_in.
  bool is_x_equal(Ipopt::Index n, const Ipopt::Number* x_in);

  // Sugar to copy an IPOPT bare array into `x`.
  void SetX(const Ipopt::Index n, const Ipopt::Number* x_arg);

  // Sugar to copy one of our member fields into an IPOPT bare array.
  static void Extract(const std::vector<Ipopt::Number>& cache_data,
                      const Ipopt::Index dest_size, Ipopt::Number* dest);

  std::vector<Ipopt::Number> x;
  std::vector<Ipopt::Number> result;
  std::vector<Ipopt::Number> grad;
  bool grad_valid{false};
};

// The C++ interface for IPOPT is described here:
// https://coin-or.github.io/Ipopt/INTERFACES.html#INTERFACE_CPP
//
// IPOPT provides a pure(-ish) virtual base class which you have to
// implement a concrete version of as the solver interface.
// IpoptSolver creates an instance of IpoptSolver_NLP which lives for
// the duration of the Solve() call.
// Use DRAKE_NO_EXPORT to hide the visibility of this class to the linker.
class DRAKE_NO_EXPORT IpoptSolver_NLP : public Ipopt::TNLP {
 public:
  IpoptSolver_NLP(const MathematicalProgram& problem,
                  const Eigen::VectorXd& x_init,
                  MathematicalProgramResult* result);

  virtual ~IpoptSolver_NLP();

  virtual bool get_nlp_info(
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l,
                               Ipopt::Number* x_u, Ipopt::Index m,
                               Ipopt::Number* g_l, Ipopt::Number* g_u);

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L,
                                  Ipopt::Number* z_U, Ipopt::Index m,
                                  bool init_lambda, Ipopt::Number* lambda);

  // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
  virtual bool eval_f(
      Ipopt::Index n, const Ipopt::Number* x, bool new_x,
      // NOLINTNEXTLINE(runtime/references); this is built into ipopt's API.
      Ipopt::Number& obj_value);

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                           Ipopt::Number* grad_f);

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Index m, Ipopt::Number* g);

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac,
                          Ipopt::Index* iRow, Ipopt::Index* jCol,
                          Ipopt::Number* values);

  virtual void finalize_solution(
      Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
      const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
      const Ipopt::Number* g, const Ipopt::Number* lambda,
      Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
      Ipopt::IpoptCalculatedQuantities* ip_cq);

  const Ipopt::SolverReturn& status() const { return status_; }
  const Eigen::VectorXd& z_L() const { return z_L_; }
  const Eigen::VectorXd& z_U() const { return z_U_; }
  const Eigen::VectorXd& g() const { return g_; }
  const Eigen::VectorXd& lambda() const { return lambda_; }

 private:
  void EvaluateCosts(Ipopt::Index n, const Ipopt::Number* x);

  void EvaluateConstraints(Ipopt::Index n, const Ipopt::Number* x,
                           bool eval_gradient);

  const MathematicalProgram* const problem_;
  std::unique_ptr<ResultCache> cost_cache_;
  std::unique_ptr<ResultCache> constraint_cache_;
  Eigen::VectorXd x_init_;
  MathematicalProgramResult* const result_;
  // status, z_L, z_U, g, lambda will be stored in IpoptSolverDetails, which is
  // declared in ipopt_solver.h. But ipopt_solver_internal doesn't depend on
  // ipopt_solver.h (in fact, ipopt_solver depends on ipopt_solver_internal). So
  // we store z_L, z_U, g, lambda here, and set IpoptSolverDetails in
  // ipopt_solver.cc
  Ipopt::SolverReturn status_{Ipopt::SolverReturn::UNASSIGNED};
  Eigen::VectorXd z_L_;
  Eigen::VectorXd z_U_;
  Eigen::VectorXd g_;
  Eigen::VectorXd lambda_;

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

// Returns Drake's supported values for the "linear_solver" IpoptSolver option.
// This will be affected by which options the IPOPT library was compiled with.
// The first item in the result is Drake's default value.
std::vector<std::string_view> GetSupportedIpoptLinearSolvers();

}  // namespace internal
}  // namespace solvers
}  // namespace drake
