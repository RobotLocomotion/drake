#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

using internal::BlockSparseCholeskySolver;
using internal::BlockSparseSymmetricMatrixT;
using internal::BlockSparsityPattern;

// Parameters to configure the ICF convex solver.
struct IcfSolverParameters {
  // Outer solver iteration limit
  int max_iterations{100};

  // Tolerance ε for the convergence conditions
  //   ‖D ∇ℓ‖ ≤ ε max(1, ‖D r‖),
  //   η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖).
  double tolerance{1e-8};

  // Whether hessian reuse between iterations and time steps is enabled.
  bool enable_hessian_reuse{false};

  // Target maximum number of iterations for Hessian reuse. The solver
  // effectively estimates the number of iterations to convergence. If the
  // estimate is larger than this target, the Hessian will be recomputed to
  // regain faster Newton-style convergence. See [Hairer, 1996], Ch. IV.8.
  int hessian_reuse_target_iterations{10};

  // Dense algebra (LDLT) for solving for the search direction H⁻¹ g.
  // This is primarily useful for debugging and testing: sparse algebra is
  // generally much faster.
  bool use_dense_algebra{false};

  // Parameters for exact linesearch
  int max_ls_iterations{100};
  double ls_tolerance{1e-8};
  double alpha_max{1.0};  // maximum step length

  // Whether to print stats to console.
  bool print_solver_stats{false};
};

// Statistics to track during the optimization process.
struct IcfSolverStats {
  // The number of solver iterations.
  // Iterations are counted starting from k = 0 as the first iteration. All
  // std::vectors below will have size() == num_iterations.
  int num_iterations;

  // The total number of Hessian factorizations.
  int num_factorizations;

  // The cost ℓ(v) at each iteration.
  std::vector<double> cost;

  // The gradient norm ||∇ℓ(v)|| at each iteration.
  std::vector<double> gradient_norm;

  // The number of linesearch iterations at each solver iteration.
  std::vector<int> ls_iterations;

  // The linesearch parameter α at each iteration.
  std::vector<double> alpha;

  // The step size at this iteration, ||Δvₖ||
  std::vector<double> step_norm;

  // Reset the stats to start a new solve.
  void Clear() {
    num_iterations = 0;
    num_factorizations = 0;
    cost.clear();
    gradient_norm.clear();
    ls_iterations.clear();
    alpha.clear();
    step_norm.clear();
  }

  // Reserve space for the vectors to avoid reallocations.
  void Reserve(int size) {
    cost.reserve(size);
    gradient_norm.reserve(size);
    ls_iterations.reserve(size);
    alpha.reserve(size);
    step_norm.reserve(size);
  }
};

// A solver for convex Irrotational Contact Fields (ICF) problems,
//
//     min_v ℓ(v; q₀, v₀, h)
//
// where (q₀, v₀) is the initial state, h is the time step, and ℓ(v) is the
// convex cost.
template <typename T>
class IcfSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfSolver);

  IcfSolver() = default;

  // Re-allocate stored data structures based on the provided model.
  void Resize(const IcfModel<T>& model) {
    model.ResizeData(&data_);
    stats_.Reserve(parameters_.max_iterations);
    search_direction_.resize(model.num_velocities());
  }

  // Solve the convex problem to compute next-step velocities v = min ℓ(v).
  //
  // @param model The ICF model defining the optimization problem.
  // @param v_guess Initial guess for the solution. On output, contains the
  //                computed solution.
  //
  // @return true if and only if the optimizer converged.
  bool SolveWithGuess(const IcfModel<T>& model, VectorX<T>* v_guess);

  // Access solver statistics from the most recent solve.
  const IcfSolverStats& stats() const { return stats_; }

  void set_parameters(const IcfSolverParameters& parameters) {
    parameters_ = parameters;
  }

  const IcfSolverParameters& get_parameters() const { return parameters_; }

  IcfSolverParameters& get_mutable_parameters() { return parameters_; }

 private:
  // Solve min_α ℓ(v + α Δ v) using a 1D Newton method with bisection
  // fallback. Returns the linesearch parameter α and the number of iterations
  // taken.
  std::pair<T, int> PerformExactLineSearch(const IcfModel<T>& model,
                                           const IcfData<T>& data,
                                           const VectorX<T>& dv);

  // Returns the root of the quadratic equation ax² + bx + c = 0, x ∈ [0, 1].
  // Used for cubic linesearch initialization.
  T SolveQuadraticInUnitInterval(const T& a, const T& b, const T& c) const;

  // Solve for the Newton search direction Δv = −H⁻¹g, with flags for several
  // levels of Hessian reuse:
  //  - reuse_factorization: reuse the exact same factorization of H as in the
  //                         previous iteration. Do not compute the new
  //                         Hessian at all. This is the fastest option, but
  //                         gives a lower-quality search direction.
  //  - reuse_sparsity_pattern: recompute H and its factorization, but reuse
  //  the
  //                            stored sparsity pattern. This gives an exact
  //                            Newton step, but avoids some allocations.
  void ComputeSearchDirection(const IcfModel<T>& model, const IcfData<T>& data,
                              VectorX<T>* dv, bool reuse_factorization = false,
                              bool reuse_sparsity_pattern = false);

  // Indicate whether a change in problem structure requires a Hessian with a
  // new sparsity pattern.
  bool SparsityPatternChanged(const IcfModel<T>& model) const;

  // Stored Hessian and factorization objects. Allows for Hessian reuse
  // between iterations and between subsequent soicf.lves (which is a valid
  // strategy since the problem is convex).
  std::unique_ptr<BlockSparseSymmetricMatrixT<T>> hessian_;
  BlockSparseCholeskySolver<Eigen::MatrixXd> hessian_factorization_;
  Eigen::LDLT<Eigen::MatrixXd> dense_hessian_factorization_;
  SearchDirectionData<T> search_direction_data_;

  // Track the sparsity pattern for Hessian reuse
  std::unique_ptr<BlockSparsityPattern> previous_sparsity_pattern_;

  // Flag for Hessian factorization re-use (changes between iterations)
  bool reuse_hessian_factorization_{true};

  // Store data is a function of v, and changes between solver iterations
  IcfData<T> data_;

  // Iteration limits, tolerances, and other parameters
  IcfSolverParameters parameters_;

  // Logging utilities
  IcfSolverStats stats_;

  // Pre-allocated search direction Δv
  VectorX<T> search_direction_;
};

// Forward-declare specializations to double, prior to DRAKE_DECLARE... below.
template <>
bool IcfSolver<double>::SolveWithGuess(const IcfModel<double>&,
                                       VectorX<double>*);
template <>
std::pair<double, int> IcfSolver<double>::PerformExactLineSearch(
    const IcfModel<double>&, const IcfData<double>&, const VectorX<double>&);

template <>
void IcfSolver<double>::ComputeSearchDirection(const IcfModel<double>&,
                                               const IcfData<double>&,
                                               VectorX<double>*, bool, bool);

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::IcfSolver);
