#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
#include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* A struct to hold the key parameters that define a convex ICF problem.

These parameters are owned by the IcfModel, and are set externally by
an IcfBuilder. */
template <typename T>
struct IcfParameters {
  // Discrete time step δt.
  T time_step{0.0};

  // Current generalized velocities v₀, size nv.
  VectorX<T> v0;

  // Current mass matrix M₀, size nv x nv.
  MatrixX<T> M0;

  // Current diagonal joint-space damping matrix D₀, size nv.
  VectorX<T> D0;

  // Current bias terms k₀, size nv.
  VectorX<T> k0;

  // Body spatial velocity Jacobians, indexed by body (size nb). Each J_WB[body]
  // is 6 x clique_nv, where clique_nv is the number of velocities in the clique
  // containing `body`.
  EigenPool<Matrix6X<T>> J_WB;

  // (Composite) mass of each body, size nb.
  std::vector<T> body_mass;

  // Clique index for each body, size nb.
  std::vector<int> body_to_clique;

  // 1 if body is free floating, 0 otherwise, size nb.
  std::vector<int> body_is_floating;

  // Number of velocities in each clique, indexed by clique, size nc.
  std::vector<int> clique_sizes;

  // Starting index in the velocity vector for each clique, size nc + 1.
  std::vector<int> clique_start;
};

/* This class defines a convex ICF problem,

   minᵥ ℓ(v;q₀,v₀,δt) = 1/2 v'Av - r'v + ℓ_c(v).

The optimality conditions ∇ℓ = 0 are
   Av - r + ∇ℓ_c = 0,
   Av - r - Jᵀγ = 0,
   Av = r + Jᵀγ,
   Mv = Mv₀ - δt k₀ + Jᵀγ,
   M(v - v₀) + δt k₀ = Jᵀγ,

since A = M, r = Av₀ - δt k₀, and ∇ℓ_c = -Jᵀγ. These are the discrete momentum
balance conditions for a multibody system with contact (and other constraints).
(Note that we really use A = M + δtD to handle joint damping implicitly, but the
above notation is easier to read.)

This class is designed to be independent of the MultibodyPlant used to
construct the problem: the job of constructing the problem given a
MultibodyPlant and its state (Context) falls to IcfBuilder.

Similarly, this model does not change with different values of the decision
variable v: quantities that change with v are stored in IcfData.

@tparam_nonsymbolic_scalar */
template <typename T>
class IcfModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfModel);

  using ConstJacobianView = typename EigenPool<Matrix6X<T>>::ConstMatrixView;
  using ConstVector6View = typename EigenPool<Vector6<T>>::ConstMatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstMatrixView;
  using MatrixXView = typename EigenPool<MatrixX<T>>::MatrixView;
  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;

  /* Constructs an empty model. */
  IcfModel();

  /* Releases ownership of parameters so that we can re-use memory.
  The typical usage is something like:

    auto params = model.ReleaseParameters();
    ... modify params ...
    model.ResetParameters(std::move(params));

  @warning Between ReleaseParameters() and ResetParameters(), the model is in an
  invalid state: no other member functions should be called. */
  std::unique_ptr<IcfParameters<T>> ReleaseParameters() {
    return std::move(params_);
  }

  /* Sets problem parameters. Verifies that the parameters are valid and
  computes some auxiliary data that will be useful during the solve. */
  void ResetParameters(std::unique_ptr<IcfParameters<T>> params);

  /* Provides access to problem parameters. */
  const IcfParameters<T>& params() const {
    DRAKE_ASSERT(params_ != nullptr);
    return *params_;
  }

  /* Returns the number of bodies involved in the problem.*/
  int num_bodies() const { return num_bodies_; }

  /* Returns the number of generalized velocities in the problem. */
  int num_velocities() const { return num_velocities_; }

  /* Returns the number of cliques in the problem. Each clique corresponds to a
  diagonal block in the mass matrix. */
  int num_cliques() const { return num_cliques_; }

  /* Returns the maximum number of velocities in any clique. */
  int max_clique_size() const { return max_clique_size_; }

  /* Returns the total number of constraints of any type in the problem. */
  int num_constraints() const {
    return num_coupler_constraints() + num_gain_constraints() +
           num_limit_constraints() + num_patch_constraints();
  }

  /* Provides mutable access to the pool of all coupler constraints. */
  CouplerConstraintsPool<T>& coupler_constraints_pool() {
    return coupler_constraints_pool_;
  }

  /* Provides mutable access to the pool of all gain (e.g., actuation)
  constraints. */
  GainConstraintsPool<T>& gain_constraints_pool() {
    return gain_constraints_pool_;
  }

  /* Provides mutable access to the pool of all joint limit constraints. */
  LimitConstraintsPool<T>& limit_constraints_pool() {
    return limit_constraints_pool_;
  }

  /* Provides mutable access to the pool of all patch (contact) constraints. */
  PatchConstraintsPool<T>& patch_constraints_pool() {
    return patch_constraints_pool_;
  }

  int num_coupler_constraints() const {
    return coupler_constraints_pool_.num_constraints();
  }

  int num_gain_constraints() const {
    return gain_constraints_pool_.num_constraints();
  }

  int num_limit_constraints() const {
    return limit_constraints_pool_.num_constraints();
  }

  int num_patch_constraints() const {
    return patch_constraints_pool_.num_patches();
  }

  /* Returns the time step δt. */
  const T& time_step() const { return params().time_step; }

  /* Returns the initial generalized velocities v₀. */
  const VectorX<T>& v0() const { return params().v0; }

  /* Returns the initial mass matrix M₀. */
  const MatrixX<T>& M0() const { return params().M0; }

  /* Returns the initial joint damping vector. */
  const VectorX<T>& D0() const { return params().D0; }

  /* Returns the initial coriolis, centrifugal, and gravitational terms, k₀. */
  const VectorX<T>& k0() const { return params().k0; }

  /* Returns a reference to the spatial velocity Jacobian for the given body. */
  ConstJacobianView J_WB(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().J_WB[body];
  }

  /* Returns the (composite) mass of the given body. */
  const T& body_mass(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_mass[body];
  }

  /* Maps a body index to a clique index. */
  int body_to_clique(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_to_clique[body];
  }

  /* Checks whether the given body is anchored to the world. Anchored bodies are
  not included in the problem, and thus have a negative clique index. */
  bool is_anchored(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return body_to_clique(body) < 0;
  }

  /* Checks whether the given body is free floating, for optimizing computations
  for identity Jacobians. */
  bool is_floating(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_is_floating[body] == 1;
  }

  /* Returns the number of generalized velocities in the given clique. */
  int clique_size(int clique) const {
    return clique < 0 ? 0 : params().clique_sizes[clique];
  }

  /* Accesses the subset of elements of `full_vector` that go with a given
  clique. This is useful when `full_vector` contains quantities for all
  velocities, e.g., generalized velocities or forces. */
  Eigen::VectorBlock<const VectorX<T>> clique_segment(
      int clique, const VectorX<T>& full_vector) const;
  Eigen::VectorBlock<VectorX<T>> mutable_clique_segment(
      int clique, VectorX<T>* full_vector) const;

  /* Returns the initial spatial velocity of the given body. */
  ConstVector6View V_WB0(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return V_WB0_[body];
  }

  /* Returns the scaling factor diag(M)⁻¹/² for convergence checks. */
  const VectorX<T>& scale_factor() const { return scale_factor_; }

  /* Returns the linearized dynamics matrix block for the given clique. */
  ConstMatrixXView A(int clique) const {
    DRAKE_ASSERT(0 <= clique && clique < num_cliques());
    return A_[clique];
  }

  /* Returns the linear cost term r = A v₀ - δt k₀ */
  const VectorX<T>& r() const { return r_; }

  /* Returns diag(M)⁻¹. This is often used to approximate the Delassus operator
  W = J⋅M⁻¹⋅Jᵀ ≈ J⋅diag(M)⁻¹⋅Jᵀ. */
  ConstVectorXView clique_diagonal_mass_inverse(int clique) const {
    DRAKE_ASSERT(0 <= clique && clique < num_cliques());
    return clique_diagonal_mass_inverse_[clique];
  }

  /* Returns the Hessian sparsity pattern. This is useful for detecting when the
  sparsity pattern is the same (in which case we use UpdateHessian()), and when
  it has changed (in which case we use MakeHessian()). */
  const contact_solvers::internal::BlockSparsityPattern& sparsity_pattern()
      const {
    DRAKE_ASSERT(sparsity_pattern_ != nullptr);
    return *sparsity_pattern_;
  }

  /* Resizes `data` to fit this model.
  No allocations are required if `data`'s capacity is already enough. */
  void ResizeData(IcfData<T>* data) const;

  /* Updates `data` as a function of v. Performs no heap allocations.
  @pre `data` has been resized to fit this model via ResizeData(). */
  void CalcData(const VectorX<T>& v, IcfData<T>* data) const;

  /* Makes a new Hessian matrix. If only `data` changes for the same ICF model,
  calling UpdateHessian() to reuse the sparsity pattern of the Hessian is
  cheaper, and incurs no memory allocations.

  The workflow to use the Hessian should be:

  // The integrator will own the factorization.
  internal::BlockSparseCholeskySolver<Eigen::MatrixXd> factorization;

  // Make an entirely new Hessian when Newton's convergence is slow and the
  // sparsity changed (i.e. across time steps).
  auto hessian = model.MakeHessian(data);

  // This performs sparsity analysis, so only call when Hessian's sparsity
  // changed (i.e. MakeHessian() was called).
  factorization.SetMatrix(*hessian);
  factorization.Factor(); // Actual numerical factorization.

  // Start Newton iteration.
  VectorXd search_direction = factorization.Solve(-residual);

  if (slow convergence within Newton) {
     // The model did not change, we can reuse the sparsity within the Newton
     // iterations.
     factorization.UpdateMatrix(*hessian);  // Update values, not the sparsity.
     factorization.Factor();  // Perform actual factorization.
  }

  Note: For "dense" hessians, we can get one with
  BlockSparseSymmetricMatrix::MakeDenseMatrix(). The additional bookkeeping in
  this class is indeed necessary to build the matrix even if dense.

  See documentation in internal::BlockSparseCholeskySolver for further details.
  */
  std::unique_ptr<
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>>
  MakeHessian(const IcfData<T>& data) const;

  /* Updates the values of the Hessian for the input `data`.
  @pre The sparsity of the `hessian` matches the structure of `this` model. */
  void UpdateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Pre-computes some quantities used to speed up CalcCostAlongLine() below. */
  void CalcSearchDirectionData(
      const IcfData<T>& data, const VectorX<T>& w,
      IcfSearchDirectionData<T>* search_direction_data) const;

  /* Computes ℓ̃ (α) = ℓ(v + α⋅w) along w at α, along with first and second
  derivatives of ℓ̃ (α).

  @param alpha The value of α.
  @param data Stores velocity v along with cached quantities. See CalcData().
  @param search_direction Stores w along with cached quantities. See
                          CalcSearchDirectionData().
  @param[out] dcost_dalpha dℓ̃ /dα on output.
  @param[out] dcost_dalpha d²ℓ̃ /dα² on output.
  @returns The cost ℓ̃ (α). */
  T CalcCostAlongLine(const T& alpha, const IcfData<T>& data,
                      const IcfSearchDirectionData<T>& search_direction,
                      T* dcost_dalpha, T* d2cost_dalpha2) const;

  /* Computes and stores the Hessian sparsity pattern.
  Note that this incurs heap allocations. */
  void SetSparsityPattern();

  /* Changes only the time step δt, updating all dependent quantities. This
  allows us to reuse pre-computed quantities, like geometry queries, between
  ICF solves that share a common initial state (q₀,v₀). */
  void UpdateTimeStep(const T& time_step);

 private:
  /* Checks that this model's parameters define a valid ICF problem. */
  void VerifyInvariants() const;

  /* Computes result = A⋅v, where A is the (sparse) linearized dynamics matrix.
   */
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* result) const;

  /* Computes the cost (1/2 v'Av - r'v) and gradient (Av - r) for the terms that
  relate to momentum only (no constraints). */
  void CalcMomentumTerms(const VectorX<T>& v, IcfData<T>* data) const;

  /* Computes spatial velocities V_WB for all bodies, given generalized
  velocities v. */
  void CalcBodySpatialVelocities(const VectorX<T>& v,
                                 EigenPool<Vector6<T>>* V_WB) const;

  // Core parameters that define the optimization problem.
  std::unique_ptr<IcfParameters<T>> params_;

  // Secondary parameters that are derived from the core parameters.
  EigenPool<Vector6<T>> V_WB0_;  // Body spatial velocities at v₀, V = J_WB v₀.
  VectorX<T> scale_factor_;      // Scale diag(M)^{-1/2} for convergence check.
  EigenPool<MatrixX<T>> A_;  // Sparse linearized dynamics matrix A = M₀ + δtD₀.
  EigenPool<VectorX<T>> clique_diagonal_mass_inverse_;  // diag(M₀)⁻¹.
  VectorX<T> Av0_;  // Av₀ storage, used to recompute r with a new δt.
  VectorX<T> r_;    // Linear cost term r = Av₀ - δt k₀.

  // Bookkeeping parameters. Like the core parameters, these do not change if
  // only the time step δt is updated.
  int num_bodies_{0};
  int num_velocities_{0};
  int num_cliques_{0};
  int max_clique_size_{0};

  // Sparsity pattern of the Hessian matrix. Defined on a per-clique basis.
  std::unique_ptr<contact_solvers::internal::BlockSparsityPattern>
      sparsity_pattern_;

  // Fixed set of constraints.
  CouplerConstraintsPool<T> coupler_constraints_pool_;
  GainConstraintsPool<T> gain_constraints_pool_;
  LimitConstraintsPool<T> limit_constraints_pool_;
  PatchConstraintsPool<T> patch_constraints_pool_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);
