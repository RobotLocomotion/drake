#pragma once

#ifndef DRAKE_POOLED_SAP_INCLUDED
#error Do not include this file. Use "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"  // NOLINT
#endif

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

using internal::BlockSparsityPattern;

/**
 * A struct to hold the key parameters that define a convex SAP problem,
 *
 *    minᵥ ℓ(v;q₀,v₀,δt) = 1/2 v'Av - r'v + ℓ(v)
 *
 * These parameters are owned by the PooledSapModel, and are set externally by
 * a PooledSapBuilder.
 *
 * TODO(CENIC): Consider some restructuring here. It's not obvious to me (Vince)
 * that this struct is better than having these parameters as members of
 * PooledSapModel directly.
 */
template <typename T>
struct PooledSapParameters {
  // Check some conditions that are required for a valid SAP problem.
  void VerifyInvariants() const {
    DRAKE_DEMAND(time_step > 0);
    const int num_bodies = ssize(body_cliques);
    DRAKE_DEMAND(ssize(body_is_floating) == num_bodies);
    DRAKE_DEMAND(ssize(body_mass) == num_bodies);
    DRAKE_DEMAND(num_bodies > 0);
    DRAKE_DEMAND(body_cliques[0] < 0);  // Always for the world.

    int num_velocities = 0;
    const int num_cliques = A.size();
    DRAKE_DEMAND(ssize(clique_nu) == num_cliques);
    for (int c = 0; c < num_cliques; ++c) {
      DRAKE_DEMAND(A[c].rows() == A[c].cols());
      const int clique_nv = A[c].rows();
      num_velocities += clique_nv;
      DRAKE_DEMAND(clique_nu[c] <= clique_nv);
    }
    DRAKE_DEMAND(r.size() == num_velocities);
    DRAKE_DEMAND(v0.size() == num_velocities);
    DRAKE_DEMAND(effort_limits.size() == num_velocities);

    DRAKE_DEMAND(J_WB.size() == num_bodies);
    for (int b = 0; b < num_bodies; ++b) {
      const int c = body_cliques[b];
      DRAKE_DEMAND(c < num_cliques);
      if (c >= 0) {
        DRAKE_DEMAND(J_WB[b].cols() == A[c].rows());
      }
    }
  }

  // The discrete time step δt.
  T time_step{0.0};

  // The current generalized velocities.
  VectorX<T> v0;

  // The (sparse) linear dynamics matrix (mass matrix + damping). Of size
  // num_cliques.
  EigenPool<MatrixX<T>> A;

  // The linear term in the cost, r = M₀ v₀ - δt k₀  + δt τ₀.
  VectorX<T> r;

  // Maps tree index to clique index, or -1 if the tree is anchored.
  std::vector<int> tree_to_clique;

  // Scaling factor D = diag(M)^{-1/2} for convergence check. Scales all
  // components of the gradient to the same units, see [Castro 2021, IV.E].
  VectorX<T> D;

  // Clique for the b-th rigid body. Negative if anchored.
  // body_cliques[0]  < 0 must correspond to the world.
  std::vector<int> body_cliques;

  // Indicator flag for floating bodies (1 if floating, 0 otherwise).
  std::vector<int> body_is_floating;

  // Mass of each body. Uses composite mass if the body is massless.
  std::vector<T> body_mass;

  // Spatial velocity Jacobian for each body.
  EigenPool<Matrix6X<T>> J_WB;

  // Number of actuators in each clique.
  std::vector<int> clique_nu;

  // Effort limits for the entire model, of size model.num_velocities(). Zero if
  // unused.
  VectorX<T> effort_limits;
};

/**
 * This class define a convex SAP problem,
 *
 *    minᵥ ℓ(v;q₀,v₀,δt) = 1/2 v'Av - r'v + ℓ(v).
 *
 * The gradient of this cost is
 *
 *    Av = r + Jᵀγ,
 *    Mv = Mv₀ + δt(τ₀ - k₀) + Jᵀγ,
 *    M(v - v₀) + δt k₀ = δt τ₀ + Jᵀγ,
 *
 * which are the discrete momentum balance conditions for a multibody system
 * with contact (and other constraints).
 *
 * This class is designed to be independent of the MultibodyPlant used to
 * construct the problem: the job of constructing the problem given a
 * MultibodyPlant and its state (Context) falls to PooledSapBuilder.
 *
 * Similarly, this model does not change with different values of the decision
 * variable v: quantities that change with v are stored in PooledSapData.
 *
 * TODO(CENIC): Consider renaming everything from "PooledSap" to "ICF"
 * (Irrotational Contact Fields) or something like that. "SAP" is already an
 * overloaded term, and CENIC builds on ICF more so than SAP.
 */
template <typename T>
class PooledSapModel {
 public:
  // TODO(amcastro-tri): We'd only need to fix the model (this) pointer in the
  // constraint classes within their copy/ctor to enable value semantics.
  // For now, I'll just disable it.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PooledSapModel);

  // Defined in separate headers.
  class CouplerConstraintsPool;
  class LimitConstraintsPool;
  class PatchConstraintsPool;
  class GainConstraintsPool;

  using ConstSpatialVelocityJacobianView =
      typename EigenPool<Matrix6X<T>>::ConstElementView;

  // Constructor for an empty model.
  PooledSapModel()
      : params_(std::make_unique<PooledSapParameters<T>>()),
        coupler_constraints_pool_(this),
        gain_constraints_pool_(this),
        limit_constraints_pool_(this),
        patch_constraints_pool_(this) {}

  // Reset problem parameters. Verifies that the parameters are valid, then
  // computes some auxiliary data that will be useful during the solve.
  void ResetParameters(std::unique_ptr<PooledSapParameters<T>> params) {
    DRAKE_DEMAND(params != nullptr);
    params->VerifyInvariants();

    params_ = std::move(params);
    num_bodies_ = ssize(params_->body_cliques);

    // Pre-compute the size of each clique and its starting index in the
    // full velocity vector.
    const int num_cliques = params_->A.size();
    clique_sizes_.clear();
    clique_start_.clear();
    clique_sizes_.reserve(num_cliques);
    clique_start_.reserve(num_cliques + 1);
    clique_start_.push_back(0);
    num_velocities_ = 0;
    auto& A = params_->A;
    // TODO(CENIC): this loop feels redundant with the loop in VerifyInvariants.
    for (int c = 0; c < A.size(); ++c) {
      const int clique_nv = A[c].rows();
      num_velocities_ += clique_nv;
      clique_sizes_.push_back(clique_nv);
      // Here we are pushing the start for the next clique, c+1.
      clique_start_.push_back(clique_start_.back() + clique_nv);
    }
    DRAKE_DEMAND(params_->r.size() == num_velocities_);

    // Reset all constraint pools.
    coupler_constraints_pool_.Clear();
    gain_constraints_pool_.Clear();
    limit_constraints_pool_.Clear();
    patch_constraints_pool_.Clear();
    patch_constraints_pool_.Reset(params_->time_step, clique_start_,
                                  clique_sizes_);

    // Pre-compute spatial velocity for each body at the current state.
    V_WB0_.Resize(num_bodies_);
    CalcBodySpatialVelocities(params_->v0, &V_WB0_);

    // Set the diagonal approximation of the Delassus operator for constraints
    // for which the Jacobian J is the identity matrix.
    clique_delassus_.Resize(clique_sizes_, clique_sizes_);
    for (int c = 0; c < A.size(); ++c) {
      clique_delassus_[c] = A[c].diagonal().cwiseInverse();
    }
  }

  // Release ownership of parameters so that we can re-use memory.
  // The typical usage is something like:
  //
  //   auto params = model.ReleaseParameters();
  //   ... modify params ...
  //   model.ResetParameters(std::move(params));
  //
  std::unique_ptr<PooledSapParameters<T>> ReleaseParameters() {
    return std::move(params_);
  }

  // Read-only access to parameters.
  const PooledSapParameters<T>& params() const {
    DRAKE_ASSERT(params_ != nullptr);
    return *params_;
  }

  // Mutable access to parameters.
  // TODO(CENIC): this seems redundant with the ResetParameters/ResetParameters
  // workflow. We should choose one (or figure out a better way).
  PooledSapParameters<T>& params() {
    DRAKE_ASSERT(params_ != nullptr);
    return *params_;
  }

  // The time step δt.
  const T& time_step() const {
    DRAKE_ASSERT(params_ != nullptr);
    return params().time_step;
  }

  // Get the clique index associated with the given body.
  int body_clique(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params_->body_cliques[body];
  }

  // Returns true iff the given body is anchored to the world. Anchored bodies
  // are not included in the problem, and thus have a negative clique index.
  bool is_anchored(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return body_clique(body) < 0;
  }

  // Returns `true` for a free floating body. This can be used to optimize out
  // Jacobian multiplications when these are the identity matrix.
  // TODO(vincekurtz): consider using operator Jacobians instead of matrix
  // Jacobians, to avoid the need for this sort of manual optimization.
  bool is_floating(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_is_floating[body] == 1;
  }

  // Returns the mass of `body`.
  const T& body_mass(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_mass[body];
  }

  // Returns the number of velocities in the given clique. In the case of an
  // anchored bodies (clique < 0), this returns 0.
  int clique_size(int clique) const {
    return clique < 0 ? 0 : clique_sizes_[clique];
  }

  // Returns the first element for `clique` in a full-model vector of
  // generalized velocities (or generalized forces).
  int clique_start(int clique) const {
    DRAKE_ASSERT(clique >= 0);
    return clique_start_[clique];
  }

  // Returns the current (at q₀, v₀) spatial velocity of the given body.
  const Vector6<T>& V_WB(int body) const {
    DRAKE_ASSERT(params_ != nullptr);
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return V_WB0_[body];
  }

  // Returns the spatial velocity Jacobian J_WB for `body`.
  ConstSpatialVelocityJacobianView get_jacobian(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().J_WB[body];
  }

  // Returns a "diagonal" estimation for the Delassus operator per-clique. The
  // Delassus operator is W = J⋅M⁻¹⋅Jᵀ. For constraints for which vc = v, i.e.
  // the constraint Jacobian is the identity, we have W = M⁻¹. Further, we
  // simplify this estimation as W = diag(M)⁻¹.
  typename EigenPool<VectorX<T>>::ConstElementView get_clique_delassus(
      int clique) const {
    DRAKE_ASSERT(params_ != nullptr);
    DRAKE_ASSERT(0 <= clique && clique < num_cliques());
    return clique_delassus_[clique];
  }

  // The linear term in the cost, r = M₀ v₀ - δt k₀  + δt τ₀.
  const VectorX<T>& r() const { return params_->r; }

  int num_bodies() const {
    DRAKE_ASSERT(params_ != nullptr);
    return num_bodies_;
  }

  int num_cliques() const {
    DRAKE_ASSERT(params_ != nullptr);
    return clique_sizes_.size();
  }

  const std::vector<int>& clique_sizes() const {
    DRAKE_ASSERT(params_ != nullptr);
    return clique_sizes_;
  }

  int max_clique_size() const {
    return *std::max_element(clique_sizes_.begin(), clique_sizes_.end());
  }

  int num_velocities() const {
    DRAKE_ASSERT(params_ != nullptr);
    return num_velocities_;
  }

  int num_constraints() const {
    return num_patch_constraints() + num_gain_constraints() +
           num_limit_constraints() + num_coupler_constraints();
  }

  // TODO(CENIC): set/parse this in a more reasonable place
  void set_stiction_tolerance(double stiction_tolerance) {
    patch_constraints_pool_.set_stiction_tolerance(stiction_tolerance);
  }

  CouplerConstraintsPool& coupler_constraints_pool() {
    return coupler_constraints_pool_;
  }

  PatchConstraintsPool& patch_constraints_pool() {
    return patch_constraints_pool_;
  }

  GainConstraintsPool& gain_constraints_pool() {
    return gain_constraints_pool_;
  }

  LimitConstraintsPool& limit_constraints_pool() {
    return limit_constraints_pool_;
  }

  int num_patch_constraints() const {
    return patch_constraints_pool_.num_patches();
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

  // Helpers to access the subset of elements from clique vectors (e.g.
  // generalized velocities, generalized forces)
  Eigen::VectorBlock<const VectorX<T>> clique_segment(
      int clique, const VectorX<T>& x) const;
  Eigen::VectorBlock<VectorX<T>> clique_segment(int clique,
                                                VectorX<T>* x) const;

  // Resizes `data` to fit this model.
  // No allocations are required if `data`'s capacity is already enough.
  void ResizeData(PooledSapData<T>* data) const;

  // Updates `data` as a function of v.
  void CalcData(const VectorX<T>& v, PooledSapData<T>* data) const;

  /* Makes a new Hessian matrix. If only `data` changes for the same SAP model,
   calling UpdateHessian() to reuse the sparsity pattern of the Hessian is
   cheaper, and incurs in no memory allocations.

   The workflow to use the Hessian should be:

   // The integrator will own the factorization.
   internal::BlockSparseCholeskySolver<Eigen::MatrixXd> factorization;

   // Make an entirely new Hessian when Newton's convergence is slow and the
   // sparsity changed (i.e. across time steps).
   auto hessian = model.MakeHessian(data);

   // This performs sparsity analysis, so only call when Hessian's sparsity
   changed (i.e. MakeHessian() was called)
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

  See documentation in  internal::BlockSparseCholeskySolver for further details.
  */
  std::unique_ptr<internal::BlockSparseSymmetricMatrixT<T>> MakeHessian(
      const PooledSapData<T>& data) const;

  // Updates the values of the Hessian for the input `data`.
  // @pre The sparsity of the `hessian` matches the structure of `this` model.
  void UpdateHessian(const PooledSapData<T>& data,
                     internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  // Pre-compute some quantities used to speed up CalcCostAlongLine() below.
  void UpdateSearchDirection(const PooledSapData<T>& data, const VectorX<T>& w,
                             SearchDirectionData<T>* search_data) const;

  /**
   * Computes ℓ(α) = ℓ(v + α⋅w) along w at α and its first dℓ/dα(α) and second
   * derivatives d²ℓ/dα²(α).
   *
   * @param alpha The value of α.
   * @param data Stores velocity v along with cached quantities. See CalcData().
   * @param search_direction Stores w along with cached quantities. See
   * UpdateSearchDirection().
   * @param scratch Scratch space properly resized with ResizeData().
   * @param dcost_dalpha dℓ/dα on output.
   * @param dcost_dalpha d²ℓ/dα² on output.
   * @returns The cost ℓ(α).
   *
   * TODO(CENIC): figure out a better scratch space solution than dragging
   * around a whole extra PooledSapData object.
   */
  T CalcCostAlongLine(const T& alpha, const PooledSapData<T>& data,
                      const SearchDirectionData<T>& search_direction,
                      PooledSapData<T>* scratch, T* dcost_dalpha,
                      T* d2cost_dalpha2) const;

  // Compute and store the Hessian sparsity pattern.
  void SetSparsityPattern();

  // Access the Hessian sparsity pattern. This is useful for when the sparsity
  // pattern is the same (in which case we use UpdateHessian()), and when it has
  // changed (in which case we use MakeHessian()).
  const BlockSparsityPattern& sparsity_pattern() const {
    DRAKE_ASSERT(sparsity_pattern_ != nullptr);
    return *sparsity_pattern_;
  }

 private:
  // Compute result = A⋅v, where A is the (sparse) linearized dynamics matrix.
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* result) const;

  // Compute the cost (1/2 v'Av - r'v) and gradient (Av - r) for the terms that
  // relate to momentum only (no constraints).
  void CalcMomentumTerms(const PooledSapData<T>& data,
                         typename PooledSapData<T>::Cache* cache) const;

  // Compute spatial velocities V_WB for all bodies, given generalized
  // velocities v.
  void CalcBodySpatialVelocities(const VectorX<T>& v,
                                 EigenPool<Vector6<T>>* V_pool) const;

  // Underlying parameters that define the optimization problem.
  std::unique_ptr<PooledSapParameters<T>> params_;

  // Sparsity pattern for the Hessian is set once at construction, and used
  // later to detect when the sparsity pattern has changed.
  std::unique_ptr<BlockSparsityPattern> sparsity_pattern_;

  int num_bodies_{0};
  int num_velocities_{0};
  std::vector<int> clique_start_;          // Clique first velocity index.
  std::vector<int> clique_sizes_;          // Number of velocities per clique.
  EigenPool<Vector6<T>> V_WB0_;            // Initial spatial velocities.
  EigenPool<VectorX<T>> clique_delassus_;  // W = diag(M)⁻¹.

  // Fixed set of constraints
  CouplerConstraintsPool coupler_constraints_pool_;
  GainConstraintsPool gain_constraints_pool_;
  LimitConstraintsPool limit_constraints_pool_;
  PatchConstraintsPool patch_constraints_pool_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
