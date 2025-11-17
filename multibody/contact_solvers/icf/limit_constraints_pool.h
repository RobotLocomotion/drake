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

using internal::BlockSparseSymmetricMatrixT;

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

  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstMatrixView;

  /* Constructor for an empty pool. */
  explicit LimitConstraintsPool(const IcfModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Reset, zeroing out the constraints while keeping memory allocated. */
  void Clear();

  /* Re-allocate memory as needed, setting all constraints as infinite by
  default so they are disabled to start with. */
  void Resize(std::span<const int> constrained_clique_sizes,
              std::span<const int> constraint_to_clique);

  /* Set the limit constraint parameters for the given clique and DoF.

   @param index The index of this limit constraint in the pool.
   @param clique The clique to which this limit constraint applies.
   @param dof The degree of freedom within the clique to which this limit
              constraint applies.
   @param q0 The current configuration value for this DoF.
   @param ql The lower limit for this DoF.
   @param qu The upper limit for this DoF. */
  void Set(int index, int clique, int dof, const T& q0, const T& ql,
           const T& qu);

  /* Upper and lower limits */
  T& lower_limit(int k, int dof) { return ql_[k](dof); }
  T& upper_limit(int k, int dof) { return qu_[k](dof); }

  /* Initial configuration q0 */
  T& configuration(int k, int dof) { return q0_[k](dof); }

  /* Near-rigid regularization */
  T& regularization(int k, int dof) { return R_[k](dof); }
  const T regularization(int k, int dof) const { return R_[k](dof); }

  /* Upper and lower bound velocity targets */
  T& vl_hat(int k, int dof) { return vl_hat_[k](dof); }
  T& vu_hat(int k, int dof) { return vu_hat_[k](dof); }
  const T vl_hat(int k, int dof) const { return vl_hat_[k](dof); }
  const T vu_hat(int k, int dof) const { return vu_hat_[k](dof); }

  /* Compute problem data for the given generalized velocities `v`. */
  void CalcData(const VectorX<T>& v,
                LimitConstraintsDataPool<T>* limit_data) const;

  /* Add the gradient contribution of this constraint, ∇ℓ = −γ, to the overall
  gradient.
  TODO(amcastro-tri): factor out this method into a
  GeneralizedVelocitiesConstraintsPool parent class, along with other common
  functionality to all constraint pools on generalized velocities. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Add the Hessian contribution of this constraint to the overall Hessian. */
  void AccumulateHessian(const IcfData<T>& data,
                         BlockSparseSymmetricMatrixT<T>* hessian) const;

  /* Compute the first and second derivatives of ℓ(α) = ℓ(v + αw) at α = 0. Used
  for exact line search. */
  void ProjectAlongLine(const LimitConstraintsDataPool<T>& limit_data,
                        const VectorX<T>& w, VectorX<T>* v_sized_scratch,
                        T* dcost, T* d2cost) const;

  /* Total number of limit constraints. */
  int num_constraints() const { return constraint_to_clique_.size(); }

  /* Number of velocities for each limit constraint. */
  std::span<const int> constraint_sizes() const {
    return std::span<const int>(constraint_sizes_);
  }

  /* Return a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

 private:
  /* Compute cost, gradient, and Hessian contribution for a single limit
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
  EigenPool<VectorX<T>> vl_hat_;
  EigenPool<VectorX<T>> vu_hat_;
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
