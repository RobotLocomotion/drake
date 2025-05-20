#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class ConstraintDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstraintDataPool);

  using VectorXPool = EigenPool<VectorX<T>>;
  using VectorXView = VectorXPool::ElementView;
  using ConstVectorXView = VectorXPool::ConstElementView;
  using MatrixXPool = EigenPool<MatrixX<T>>;
  using MatrixXView = MatrixXPool::ElementView;
  using ConstMatrixXView = MatrixXPool::ConstElementView;

  /* Constructs an empty pool. */
  ConstraintDataPool() = default;

  /* @param num_equations Number of equations for the k-th constraint.
     @param num_velocities Number of velocities for the k-th constraint. */
  ConstraintDataPool(const std::vector<int>& num_equations,
                     const std::vector<int>& num_velocities) {
    DRAKE_ASSERT(num_equations.size() == num_velocities.size());
    Resize(num_equations, num_velocities);
  }

  int num_constraints() const { return vc_.size(); }

  /* Clears all the data. The capacity does not change. */
  void Clear() {
    vc_.Clear();
    gamma_.Clear();
    H_.Clear();
    // data_views_.clear();
  }

  /* Resizes to store data of the given size. There's no memory allocation if
   * there is sufficient capacity for the requested amount of data. */
  void Resize(const std::vector<int>& num_equations,
              const std::vector<int>& num_velocities) {
    vc_.Resize(num_equations);
    gamma_.Resize(num_equations);
    H_.Resize(num_velocities, num_velocities);
  }

  const ConstVectorXView& vc(int constraint_index) const {
    return vc_[constraint_index];
  }
  VectorXView& vc(int constraint_index) { return vc_[constraint_index]; }
  const ConstVectorXView& gamma(int constraint_index) const {
    return gamma_[constraint_index];
  }
  VectorXView& gamma(int constraint_index) { return gamma_[constraint_index]; }
  const ConstMatrixXView& H(int constraint_index) const {
    return H_[constraint_index];
  }
  MatrixXView& H(int constraint_index) { return H_[constraint_index]; }

 private:
  EigenPool<VectorX<T>> vc_;
  EigenPool<VectorX<T>> gamma_;
  EigenPool<MatrixX<T>> H_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
