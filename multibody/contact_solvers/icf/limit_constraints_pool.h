#pragma once

#include <span>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/limit_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* A pool of joint limit constraints, qu ≥ q ≥ ql.

Each limit constraint is associated with a convex cost ℓ(v) that is added to
the overall ICF cost. The gradient of this cost produces an impulse γ = -∇ℓ(v)
that enforces the constraint when the convex ICF problem is solved.

@tparam_nonsymbolic_scalar */
template <typename T>
class LimitConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LimitConstraintsPool);

  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  explicit LimitConstraintsPool(const IcfModel<T>* parent_model);

  ~LimitConstraintsPool();

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the total number of limit constraints. */
  int num_constraints() const { return clique_.size(); }

  /* Returns the number of generalized velocities associated with each limit
  constraint. */
  std::span<const int> constraint_sizes() const {
    return std::span<const int>(constraint_size_);
  }

  /* Resizes this pool to store limit constraints of the given sizes.

  Sets all constraints as infinite (e.g., disabled) by default. Constraints must
  be explicitly enabled for each DoF by calling Set().

  @param sizes The number of velocities associated with each limit constraint in
               the pool.
  @param constraint_to_clique The mapping from constraint index to clique
                              index for each limit constraint.

  @pre `sizes` and `constraint_to_clique` must have the same length. */
  void Resize(std::span<const int> sizes,
              std::span<const int> constraint_to_clique);

  /* Sets the limit constraint parameters for the given clique and DoF.

  @param index The index of this limit constraint in the pool.
  @param clique The clique to which this limit constraint applies.
  @param dof The degree of freedom within the clique to which this limit
             constraint applies.
  @param q0 The current configuration value for this DoF.
  @param ql The lower limit for this DoF.
  @param qu The upper limit for this DoF.

  Calling this function several times with the same `index` overwrites the
  previous constraint for that index. */
  void Set(int index, int clique, int dof, const T& q0, const T& ql,
           const T& qu);

  /* Computes problem data for the given generalized velocities `v`. */
  void CalcData(const VectorX<T>& v,
                LimitConstraintsDataPool<T>* limit_data) const;

  /* Adds the gradient contribution of this constraint, ∇ℓ = −γ, to the
  model-wide gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the contribution of this constraint to the model-wide Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃ (α) = ℓ(v + α⋅w).

  @param coupler_data Constraint data computed at v + α⋅w.
  @param w The search direction.
  @param Gw_scratch Scratch space for intermediate values for each clique.
  @param[out] dcost The first derivative dℓ̃ /dα on output.
  @param[out] d2cost The second derivative d²ℓ̃ /dα² on output. */
  void CalcCostAlongLine(const LimitConstraintsDataPool<T>& limit_data,
                         const VectorX<T>& w, EigenPool<VectorX<T>>* Gw_scratch,
                         T* dcost, T* d2cost) const;

 private:
  /* Computes cost, gradient, and Hessian contribution for a single limit
  constraint.

  @param v_hat The target velocity.
  @param R Near-rigid regularization parameter.
  @param v The current velocity.
  @param[out] gamma The computed impulse on output.
  @param[out] G The computed Hessian diagonal on output.

  @returns The cost associated with this limit constraint. */
  T CalcLimitData(const T& v_hat, const T& R, const T& v, T* gamma, T* G) const;

  const IcfModel<T>* const model_;  // The parent model.

  // We always add limit constraints per-clique. Each of the following has size
  // num_constraints().
  std::vector<int> clique_;           // Clique the k-th limit belongs to.
  std::vector<int> constraint_size_;  // Clique size for the k-th constraint.
  EigenPool<VectorX<T>> ql_;          // Lower limit.
  EigenPool<VectorX<T>> qu_;          // Upper limit.
  EigenPool<VectorX<T>> q0_;          // Initial configuration.
  EigenPool<VectorX<T>> gl_hat_;  // Lower bound velocity target scaled by dt.
  EigenPool<VectorX<T>> gu_hat_;  // Upper bound velocity target scaled by dt.
  EigenPool<VectorX<T>> R_;       // Near-rigid regularization parameter.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        LimitConstraintsPool);
