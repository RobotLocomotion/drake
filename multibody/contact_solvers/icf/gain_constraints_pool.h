#pragma once

#include <span>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* A pool of gain constraints organized by cliques.

A gain constraint models generalized forces acting on a given clique as

 τ = clamp(-K⋅v + b, -e, e)

where K is a non-negative diagonal gain matrix, b is a bias term, and e is a
double-sided effort limit.

Each gain constraint is associated with a convex cost ℓ(v) that is added to the
overall ICF cost. The gradient of this cost produces an impulse

  γ = -∇ℓ(v) = -δt⋅τ

that enforces the constraint when the convex ICF problem is solved.

@tparam_nonsymbolic_scalar */
template <typename T>
class GainConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GainConstraintsPool);

  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  explicit GainConstraintsPool(const IcfModel<T>* parent_model);

  ~GainConstraintsPool();

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the total number of gain constraints stored in this pool. */
  int num_constraints() const { return clique_.size(); }

  /* Returns the number of velocities for each gain constraint. */
  std::span<const int> constraint_sizes() const {
    return std::span<const int>(constraint_size_);
  }

  /* Resizes this pool to store gain constraints of the given sizes.

  @param sizes The number of velocities associated with each gain constraint.

  @warning After resizing, constraints may hold invalid data until Set() is
  called for each constraint index in [0, num_constraints()). */
  void Resize(std::span<const int> sizes);

  /* Defines the gain constraint τ = clamp(-K⋅v + b, -e, e) for a given clique.

  @param index The index of this gain constraint in the pool.
  @param clique The clique to which this gain constraint applies.
  @param K The diagonal entries of gain matrix K. They must be >= 0.
  @param b The bias term.
  @param e The vector of double-sided effort limits for each DoF of the clique.

  Calling this function several times with the same `index` overwrites the
  previous constraint for that index.

  @pre K, b, e are of size model().clique_size(clique). */
  void Set(int index, int clique, const VectorX<T>& K, const VectorX<T>& b,
           const VectorX<T>& e);

  /* Computes problem data as a function of the generalized velocities `v` for
  the full plant. */
  void CalcData(const VectorX<T>& v,
                GainConstraintsDataPool<T>* gain_data) const;

  /* Adds the gradient contribution of this constraint, ∇ℓ = −γ, to the
  model-wide gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the contribution of this constraint to the model-wide Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃(α) = ℓ(v + α⋅w).

  @param gain_data Constraint data computed at v + α⋅w.
  @param w The search direction.
  @param Gw_scratch Scratch space for intermediate values for each clique.
  @param[out] dcost The first derivative dℓ̃ /dα on output.
  @param[out] d2cost The second derivative d²ℓ̃ /dα² on output. */
  void CalcCostAlongLine(const GainConstraintsDataPool<T>& gain_data,
                         const VectorX<T>& w, EigenPool<VectorX<T>>* Gw_scratch,
                         T* dcost, T* d2cost) const;

  /* Testing only access. */
  const std::vector<int>& clique() const { return clique_; }
  const std::vector<int>& constraint_size() const { return constraint_size_; }
  const EigenPool<VectorX<T>>& K() const { return K_; }
  const EigenPool<VectorX<T>>& b() const { return b_; }
  const EigenPool<VectorX<T>>& le() const { return le_; }
  const EigenPool<VectorX<T>>& ue() const { return ue_; }

 private:
  /* Computes the cost, gradient, and Hessian contribution for a single gain
  constraint.

  @param k The index of the gain constraint.
  @param v The velocity associated with the corresponding clique.
  @param[out] gamma The computed impulse γ = -∇ℓ(v) on output.
  @param[out] G The computed (diagonal) Hessian G = -∂γ/∂v on output.
  @returns The cost ℓ(v) associated with this gain constraint. */
  T Clamp(int k, const Eigen::Ref<const VectorX<T>>& v,
          EigenPtr<VectorX<T>> gamma, EigenPtr<VectorX<T>> G) const;

  const IcfModel<T>* const model_;  // The parent model.

  // We always add gain constraints per-clique. Each of the following has size
  // num_constraints().
  std::vector<int> clique_;           // Clique the k-th gain belongs to.
  std::vector<int> constraint_size_;  // Clique size for the k-th constraint.
  EigenPool<VectorX<T>> K_;
  EigenPool<VectorX<T>> b_;
  EigenPool<VectorX<T>> le_;  // Lower effort limit.
  EigenPool<VectorX<T>> ue_;  // Upper effort limit.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsPool);
