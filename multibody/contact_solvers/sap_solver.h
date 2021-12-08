#pragma once

#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// SAP solver parameters such as tolerances, maximum number of iterations and
// regularization parameters.
struct SapSolverParameters {
  // Stopping criteria tolerances. We monitor the optimality condition (for
  // SAP, balance of momentum), i.e. ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖jc‖),
  // where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum balance residual, p = A⋅v and jc =
  // Jᵀ⋅γ. The norms above are defined as ‖x‖ = ‖D⋅x‖₂, where D = diag(A)^(1/2).
  // If p is a generalized momentum, then D⋅p is a scaled generalized momentum
  // where each component has the same units, square root of Joules. Therefore
  // the norms above are used to weigh all components of the generalized
  // momentum equally.
  double abs_tolerance{1.0e-6};  // Absolute tolerance εₐ, square root of Joule.
  double rel_tolerance{1.0e-6};  // Relative tolerance εᵣ.
  int max_iterations{100};       // Maximum number of Newton iterations.

  // Line-search parameters.
  double ls_alpha_max{1.5};   // Maximum line search parameter allowed.
  int ls_max_iterations{40};  // Maximum number of line search iterations.
  double ls_c{1.0e-4};        // Armijo's criterion parameter.
  double ls_rho{0.8};         // Backtracking search parameter.

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
  // corresponds to a diagonal approximation of the Delassuss operator for each
  // contact. See [Castro et al., 2021. §IX.A] for details.
  double beta{1.0};

  // Dimensionless parameterization of the regularization of friction. An
  // approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
  double sigma{1.0e-3};

  // Tolerance used in impulse soft norms. In Ns.
  double soft_tolerance{1.0e-7};
};

// This class implements the Semi-Analytic Primal (SAP) solver described in
// [Castro et al., 2021].
//
// SAP uses the convex approximation of contact constraints by [Anitescu, 2006],
// with constraint regularization and analytical inverse dynamics introduced by
// [Todorov, 2014]. However SAP introduces a primal formulation in velocities
// instead of in impulses as done in previous work. This leads to a numerical
// scheme that warm-starts very effectively using velocities from the previous
// time step. In addition, SAP uses regularization to model physical compliance
// rather than to introduce constraint stabilization as previously done by
// [Todorov, 2014]. Please refer to [Castro et al., 2021] for details.
//
// - [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
//   Unconstrained Convex Formulation of Compliant Contact. Available online at
//   https://arxiv.org/abs/2110.10107.
// - [Anitescu, 2006] Anitescu M., 2006. Optimization-based simulation of
//   nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
//   pp.113-143.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   MuJoCo. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
//
// TODO(amcastro-tri): enable AutoDiffXd support, if only for dense matrices so
// that we can test the long term performant solution.
// @tparam_double_only
template <typename T>
class SapSolver final : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapSolver);

  SapSolver() = default;
  ~SapSolver() final = default;

  // Solve the contact problem specified by the input data. See
  // ContactSolver::SolveWithGuess() for details. Currently, only `T = double`
  // is supported. An exception is thrown if `T != double`.
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  // New parameters will affect the next call to SolveWithGuess().
  void set_parameters(const SapSolverParameters& parameters) {
    parameters_ = parameters;
  }

 private:
  friend class SapSolverTester;

  // Structure used to store input data pre-processed for computation. For
  // mathematical quantities, we attempt to follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII and Unicode
  // symbols.
  struct PreProcessedData {
    // Constructs an empty data.
    PreProcessedData() = default;

    // @param nv Number of generalized velocities.
    // @param nc Number of contact constraints.
    // @param dt The discrete time step used for simulation.
    PreProcessedData(double dt, int nv_in, int nc_in) : time_step(dt) {
      Resize(nv_in, nc_in);
    }

    // Resizes this PreProcessedData to store data for a problem with nv_in
    // generalized velocities and nc_in contact constraints. A call to this
    // method causes loss of all previously existing data.
    // @param nv_in Number of generalized velocities.
    // @param nc_in Number of contact constraints.
    void Resize(int nv_in, int nc_in) {
      nv = nv_in;
      nc = nc_in;
      const int nc3 = 3 * nc;
      R.resize(nc3);
      Rinv.resize(nc3);
      vhat.resize(nc3);
      mu.resize(nc);
      inv_sqrt_A.resize(nv);
      v_star.resize(nv);
      p_star.resize(nv);
      delassus_diagonal.resize(nc);
    }

    T time_step{NAN};        // Discrete time step used by the solver.
    int nv{0};               // Number of generalized velocities.
    int nc{0};               // Number of contacts.
    VectorX<T> R;            // (Diagonal) Regularization matrix, of size 3nc.
    VectorX<T> Rinv;         // Inverse of regularization matrix, of size 3nc.
    VectorX<T> vhat;         // Constraints stabilization velocity, of size 3nc.
    VectorX<T> mu;           // Friction coefficients, of size nc.
    BlockSparseMatrix<T> J;  // Jacobian as block-sparse matrix.
    BlockSparseMatrix<T> A;  // Momentum matrix as block-sparse matrix.
    std::vector<MatrixX<T>> At;   // Per-tree blocks of the momentum matrix.

    // Inverse of the diagonal matrix formed with the square root of the
    // diagonal entries of the momentum matrix, i.e. inv_sqrt_A =
    // diag(A)^{-1/2}. This matrix is used to compute a scaled momentum
    // residual, see discussion on tolerances in SapSolverParameters for
    // details.
    VectorX<T> inv_sqrt_A;

    VectorX<T> v_star;  // Free-motion generalized velocities.
    VectorX<T> p_star;  // Free motion generalized impulse, i.e. p* = M⋅v*.
    VectorX<T> delassus_diagonal;  // Delassus operator diagonal approximation.
  };

  // Computes a diagonal approximation of the Delassus operator used to compute
  // a per constraint diagonal scaling into delassus_diagonal. Given an
  // approximation Wₖₖ of the block diagonal element corresponding to the k-th
  // constraint, the scaling is computed as delassus_diagonal[k] = ‖Wₖₖ‖ᵣₘₛ =
  // ‖Wₖₖ‖/3. See [Castro et al. 2021] for details.
  // @pre Matrix entries stored in `At` are SPD.
  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& At,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* delassus_diagonal) const;

  // This method extracts and pre-processes input data into a format that is
  // more convenient for computation. In particular, it computes quantities
  // directly appearing in the optimization problem such as R, v̂, W, among
  // others.
  PreProcessedData PreProcessData(
      const T& time_step, const SystemDynamicsData<T>& dynamics_data,
      const PointContactData<T>& contact_data) const;

  // Pack solution into ContactSolverResults. Where v is the vector of
  // generalized velocities, vc is the vector of contact velocities and gamma is
  // the vector of generalized contact impulses.
  static void PackContactResults(const PreProcessedData& data,
                                 const VectorX<T>& v, const VectorX<T>& vc,
                                 const VectorX<T>& gamma,
                                 ContactSolverResults<T>* result);

  const SapSolverParameters& parameters() const { return parameters_; }

  SapSolverParameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::contact_solvers::internal::SapSolver<
    double>;
