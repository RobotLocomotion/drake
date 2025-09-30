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
#include "drake/multibody/contact_solvers/pooled_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

using internal::BlockSparsityPattern;

template <typename T>
struct PooledSapParameters {
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

  // Discrete time step.
  T time_step{0.0};
  // Linear dynamics matrix. Of size num_cliques.
  EigenPool<MatrixX<T>> A;
  // Cost linear term
  VectorX<T> r;

  // Maps tree index to clique index, or -1 if tree is anchored.
  std::vector<int> tree_to_clique;

  // Scaling factor D = diag(M)^{-1/2} for convergence check. Scales all
  // components of the gradient to the same units, see [Castro 2021, IV.E].
  VectorX<T> D;

  // Clique for the b-th rigid body. Negative if anchored.
  // body_cliques[0]  < 0 must correspond to the world.
  std::vector<int> body_cliques;
  std::vector<int> body_is_floating;  // 1 if floating.
  std::vector<T>
      body_mass;  // mass of each body. Composite mass for zero massless bodies.
  EigenPool<Matrix6X<T>> J_WB;  // Rigid body spatial velocity Jacobians.
  VectorX<T> v0;                // The current generalized velocities.

  // Effort limits for the entire model.
  std::vector<int> clique_nu;  // Num actuators per clique.
  VectorX<T> effort_limits;  // of size model.num_velocities(). Zero if unused.
};

/* A model of the SAP problem.
 It models the discrete momentum equation:
    A⋅v = r + Jᵀ⋅γ,
 which corresponds to the convex cost:
    ℓ(v) = 1/2‖v‖²−r⋅v + ℓ(vc).

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
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

  // TODO(amcastro-tri).
  // class LimitConstraintsPool;
  // class ActuationConstraintsPool;

  using ConstSpatialVelocityJacobianView =
      typename EigenPool<Matrix6X<T>>::ConstElementView;

  // Constructor for an empty model.
  PooledSapModel()
      : params_(std::make_unique<PooledSapParameters<T>>()),
        coupler_constraints_pool_(this),
        gain_constraints_pool_(this),
        limit_constraints_pool_(this),
        patch_constraints_pool_(this) {}

  /* Resets problem parameters.
   @pre params->VerifyInvariants() is true. */
  void ResetParameters(std::unique_ptr<PooledSapParameters<T>> params) {
    DRAKE_DEMAND(params != nullptr);
    params->VerifyInvariants();
    params_ = std::move(params);
    num_bodies_ = ssize(params_->body_cliques);
    clique_sizes_.clear();
    clique_start_.clear();

    const int num_cliques = params_->A.size();
    clique_sizes_.reserve(num_cliques);
    clique_start_.reserve(num_cliques + 1);
    clique_start_.push_back(0);
    num_velocities_ = 0;
    auto& A = params_->A;
    for (int c = 0; c < A.size(); ++c) {
      const int clique_nv = A[c].rows();
      num_velocities_ += clique_nv;
      clique_sizes_.push_back(clique_nv);
      // Here we are pushing the start for the next clique, c+1.
      clique_start_.push_back(clique_start_.back() + clique_nv);
    }
    DRAKE_DEMAND(params_->r.size() == num_velocities_);
    coupler_constraints_pool_.Reset();
    gain_constraints_pool_.Clear();
    gain_constraints_pool_.Reset();
    limit_constraints_pool_.Reset();

    patch_constraints_pool_.Clear();
    patch_constraints_pool_.Reset(params_->time_step, clique_start_,
                                  clique_sizes_);

    V_WB0_.Resize(num_bodies_);
    CalcBodySpatialVelocities(params_->v0, &V_WB0_);

    // Diagonal approximation of the Delassus operator for constraints for which
    // the Jacobian J is the identity matrix.
    clique_delassus_.Reserve(clique_sizes_);
    for (int c = 0; c < A.size(); ++c) {
      clique_delassus_.PushBack(A[c].diagonal().cwiseInverse());
    }
  }

  /* Releases ownership of parameters so that we can re-use previously allocated
  memory. */
  std::unique_ptr<PooledSapParameters<T>> ReleaseParameters() {
    return std::move(params_);
  }

  const PooledSapParameters<T>& params() const {
    DRAKE_ASSERT(params_ != nullptr);
    return *params_;
  }

  PooledSapParameters<T>& params() {
    DRAKE_ASSERT(params_ != nullptr);
    return *params_;
  }

  const T& time_step() const {
    DRAKE_ASSERT(params_ != nullptr);
    return params().time_step;
  }

  int body_clique(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params_->body_cliques[body];
  }

  /* Returns true iff body is anchored to the world.
   Anchored bodies have a zero-sized (clique_size(body_clique(body)) == 0)
   negative clique (body_clique(body) < 0). */
  bool is_anchored(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return body_clique(body) < 0;
  }

  /* Returns `true` for a free floating body. This can be used to optimize out
   Jacobian multiplications when these are the identity matrix. */
  bool is_floating(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_is_floating[body] == 1;
  }

  /* Returns the mass for `body`. */
  const T& body_mass(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().body_mass[body];
  }

  /* Returns the number of velocities for `clique` or zero if clique < 0
   (anchored body). */
  int clique_size(int clique) const {
    return clique < 0 ? 0 : clique_sizes_[clique];
  }

  /* Returns the first element for `clique` in a full-model vector of
   generalized velocities (or generalized forces). */
  int clique_start(int clique) const {
    DRAKE_ASSERT(clique >= 0);
    return clique_start_[clique];
  }

  /* Spatial velocity at current state x = [q0, v0]. */
  const Vector6<T>& V_WB(int body) const {
    DRAKE_ASSERT(params_ != nullptr);
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return V_WB0_[body];
  }

  /* Returns the spatial velocity Jacobian J_WB for `body`. */
  ConstSpatialVelocityJacobianView get_jacobian(int body) const {
    DRAKE_ASSERT(0 <= body && body < num_bodies());
    return params().J_WB[body];
  }

  typename EigenPool<MatrixX<T>>::ConstElementView get_dynamics_matrix(
      int clique) const {
    DRAKE_ASSERT(params_ != nullptr);
    DRAKE_ASSERT(0 <= clique && clique < num_cliques());
    return params().A[clique];
  }

  /* Returns a "diagonal" estimation for the Delassus operator per-clique. The
   Delassus operator is W = J⋅M⁻¹⋅Jᵀ. For constraints for which vc = v, i.e. the
   constraint Jacobian is the identity, we have W = M⁻¹. Further, we simplify
   this estimation as W = diag(M)⁻¹. */
  typename EigenPool<VectorX<T>>::ConstElementView get_clique_delassus(
      int clique) const {
    DRAKE_ASSERT(params_ != nullptr);
    DRAKE_ASSERT(0 <= clique && clique < num_cliques());
    return clique_delassus_[clique];
  }

  const VectorX<T>& r() const { return params_->r; }

  int num_bodies() const {
    DRAKE_ASSERT(params_ != nullptr);
    return num_bodies_;
  }

  /* Returns the number of cliques. */
  int num_cliques() const {
    DRAKE_ASSERT(params_ != nullptr);
    return clique_sizes_.size();
  }

  const std::vector<int>& clique_sizes() const {
    DRAKE_ASSERT(params_ != nullptr);
    return clique_sizes_;
  }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const {
    DRAKE_ASSERT(params_ != nullptr);
    return num_velocities_;
  }

  /* Total number of constraints. */
  int num_constraints() const {
    return num_patch_constraints() + num_gain_constraints() +
           num_limit_constraints() + num_coupler_constraints();
  }

  /* Total number of constraint equations. */
  int num_constraint_equations() const {
    return coupler_constraints_pool_.num_constraint_equations() +
           patch_constraints_pool_.num_constraint_equations() +
           gain_constraints_pool_.num_constraint_equations() +
           limit_constraints_pool_.num_constraint_equations();
  }

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

  /* Limit constraints are added on a per-clique basis. Therefore this method
   * can only be called at most num_cliques() times. */
  // int AddConstraint(SapLimitConstraint<T> constraint);

  // TODO(amcastro-tri): Add these:
  //   SapActuationConstraint: actuation with effort limits. Per-clique.
  //   SapHolonomicConstraint: usually one or two cliques. Consider among more
  //   than two cliques?
  //   SapUnilateralConstraint: things like tendons. Maybe SapLimitConstraint
  //   covers this?

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

  /* Resizes data accordingly to store data for this model.
   No allocations are required if data's capacity is already enough. */
  void ResizeData(SapData<T>* data) const;

  // Updates `data` as a function of v.
  void CalcData(const VectorX<T>& v, SapData<T>* data) const;

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
      const SapData<T>& data) const;

  /* Updates the values of the Hessian for the input `data`.
   @pre The sparsity of the `hessian` matches the structure of `this` model. */
  void UpdateHessian(const SapData<T>& data,
                     internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  void UpdateSearchDirection(const SapData<T>& data, const VectorX<T>& w,
                             SearchDirectionData<T>* search_data) const;

  /* Computes ℓ(α) = ℓ(v + α⋅w) along w at α and its first dℓ/dα(α) and second
   derivatives d²ℓ/dα²(α).
   @param alpha The value of α.
   @param data Stores velocity v along with cached quantities. See CalcData().
   @param search_direction Stores w along with cached quantities. See
   UpdateSearchDirection().
   @param scratch Scratch space properly resized with ResizeData().
   @param dcost_dalpha dℓ/dα on output.
   @param dcost_dalpha d²ℓ/dα² on output.
   @returns The cost ℓ(α). */
  T CalcCostAlongLine(const T& alpha, const SapData<T>& data,
                      const SearchDirectionData<T>& search_direction,
                      SapData<T>* scratch, T* dcost_dalpha,
                      T* d2cost_dalpha2) const;

  /* Compute and store the Hessian sparsity pattern. */
  void SetSparsityPattern();

  const BlockSparsityPattern& sparsity_pattern() const {
    DRAKE_ASSERT(sparsity_pattern_ != nullptr);
    return *sparsity_pattern_;
  }

  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* result) const;

 private:
  void CalcMomentumTerms(const SapData<T>& data,
                         typename SapData<T>::Cache* cache) const;
  void CalcBodySpatialVelocities(const VectorX<T>& v,
                                 EigenPool<Vector6<T>>* V_pool) const;

  std::unique_ptr<PooledSapParameters<T>> params_;

  // Sparsity pattern for the Hessian is set once at construction, and used
  // later to detect when the sparsity pattern has changed.
  std::unique_ptr<BlockSparsityPattern> sparsity_pattern_;

  // Total number of generalized velocities. = sum(clique_sizes_).
  int num_bodies_{0};
  int num_velocities_{0};
  std::vector<int> clique_start_;          // Clique first velocity.
  std::vector<int> clique_sizes_;          // Number of velocities per clique.
  EigenPool<Vector6<T>> V_WB0_;            // Initial spatial velocities.
  EigenPool<VectorX<T>> clique_delassus_;  // W = W = diag(M)⁻¹.

  CouplerConstraintsPool coupler_constraints_pool_;
  GainConstraintsPool gain_constraints_pool_;
  LimitConstraintsPool limit_constraints_pool_;
  PatchConstraintsPool patch_constraints_pool_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
