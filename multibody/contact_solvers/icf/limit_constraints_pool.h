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

/* A pool of limit constraints, qu ≥ q ≥ ql. */
template <typename T>
class LimitConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LimitConstraintsPool);

  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  explicit LimitConstraintsPool(const IcfModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Re-allocates memory as needed, setting all constraints as infinite by
  default so they are disabled to start with. */
  void Resize(std::span<const int> constrained_clique_sizes,
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

  /* Returns the lower limit for the given constraint and DoF. */
  T& lower_limit(int k, int dof) { return ql_[k](dof); }

  /* Returns the upper limit for the given constraint and DoF. */
  T& upper_limit(int k, int dof) { return qu_[k](dof); }

  /* Returns the initial configuration q0 for the given constraint and DoF. */
  T& configuration(int k, int dof) { return q0_[k](dof); }

  /* Returns the near-rigid regularization parameter R. */
  T& regularization(int k, int dof) { return R_[k](dof); }
  const T regularization(int k, int dof) const { return R_[k](dof); }

  /* Returns upper and lower bound velocity targets, scaled by the time step. */
  T& gl_hat(int k, int dof) { return gl_hat_[k](dof); }
  T& gu_hat(int k, int dof) { return gu_hat_[k](dof); }
  const T gl_hat(int k, int dof) const { return gl_hat_[k](dof); }
  const T gu_hat(int k, int dof) const { return gu_hat_[k](dof); }

  /* Computes problem data for the given generalized velocities `v`. */
  void CalcData(const VectorX<T>& v,
                LimitConstraintsDataPool<T>* limit_data) const;

  /* Adds the gradient contribution of this constraint, ∇ℓ = −γ, to the overall
  gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the Hessian contribution of this constraint to the overall Hessian. */
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

  /* Returns the total number of limit constraints. */
  int num_constraints() const { return constraint_to_clique_.size(); }

  /* Returns the number of velocities for each limit constraint. */
  std::span<const int> constraint_sizes() const {
    return std::span<const int>(constraint_sizes_);
  }

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

 private:
  /* Computes cost, gradient, and Hessian contribution for a single limit
  constraint. */
  T CalcLimitData(const T& v_hat, const T& R, const T& v, T* gamma, T* G) const;

  const IcfModel<T>* model_{nullptr};  // The parent model.

  // Clique for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_to_clique_;

  // Clique size for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_sizes_;

  EigenPool<VectorX<T>> ql_;
  EigenPool<VectorX<T>> qu_;
  EigenPool<VectorX<T>> q0_;
  EigenPool<VectorX<T>> gl_hat_;
  EigenPool<VectorX<T>> gu_hat_;
  EigenPool<VectorX<T>> R_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        LimitConstraintsPool);
