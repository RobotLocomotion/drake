#pragma once

#ifndef DRAKE_ICF_MODEL_NESTED_CLASS_INCLUDES
#error Do not directly include this file; instead, use icf_model.h.
#endif

#include <span>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* A pool of gain constraints organized by cliques.

A clique can have a gain constraint that models generalized forces on the
clique according to:

 τ = clamp(−K⋅v + b, e)

where K is a positive semi-definite diagonal gain matrix, b is a bias term
and e is an effort limit. Generalized impulses for that clique are thus γ =
δt⋅τ. */
template <typename T>
class IcfModel<T>::GainConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GainConstraintsPool);

  /* Constructor for an empty pool. */
  GainConstraintsPool(const IcfModel<T>* parent_model) : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Reset, zeroing out the constraints while keeping memory allocated. */
  void Clear();

  /* Resize this pool to store gain constraints of the given sizes. */
  void Resize(std::span<const int> sizes);

  /* Set the given gain constraint for the given clique.

   @param index The index of this gain constraint in the pool.
   @param clique The clique to which this gain constraint applies.
   @param K The diagonal entries of gain matrix K. They must be >= 0.
   @param b The bias term.
   @param e The vector of effort limits for each DoF of the clique.
   @pre K, b, e are of size model().clique_size(clique). */
  void Set(const int index, int clique, const VectorX<T>& K,
           const VectorX<T>& b, const VectorX<T>& e);

  /* Compute problem data for the given generalized velocities `v`. */
  void CalcData(const VectorX<T>& v,
                GainConstraintsDataPool<T>* gain_data) const;

  /* Add the gradient contribution of this constraint, ∇ℓ = −γ, to the overall
  gradient.
  TODO(amcastro-tri): factor out this method into a
  GeneralizedVelocitiesConstraintsPool parent class, along with other common
  functionality to all constraint pools on generalized velocities. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Add the Hessian contribution of this constraint to the overall Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  /* Compute the first and second derivatives of ℓ(α) = ℓ(v + αw) at α = 0. Used
  for exact line search. */
  void ProjectAlongLine(const GainConstraintsDataPool<T>& gain_data,
                        const VectorX<T>& w, VectorX<T>* v_sized_scratch,
                        T* dcost, T* d2cost) const;

  /* Total number of gain constraints. */
  int num_constraints() const { return clique_.size(); }

  /* Number of velocities for each gain constraint. */
  std::span<const int> constraint_sizes() const {
    return std::span<const int>(constraint_sizes_);
  }

  /* Return a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

 private:
  /* For the k-th gain constraint, compute:
    - The clamped cost ℓ(v)
    - The clamped gradient/impulse γ = -∇ℓ(v) = clamp(−K⋅v + b, e)
    - The clamped Hessian G = -∂γ/∂v (diagonal) */
  T Clamp(int k, const Eigen::Ref<const VectorX<T>>& v,
          EigenPtr<VectorX<T>> gamma, EigenPtr<MatrixX<T>> G) const;

  const IcfModel<T>* model_{nullptr};  // The parent model.

  // We always add gain constraints per-clique.
  std::vector<int> clique_;            // Clique the k-th gain belongs to.
  std::vector<int> constraint_sizes_;  // Clique size for the k-th constraint.
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
