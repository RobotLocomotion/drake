#pragma once

#include <numeric>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class CouplerConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintsDataPool);

  /* Default constructor for an empty pool. */
  CouplerConstraintsDataPool() = default;

  void Resize(int num_couplers) { gamma_pool_.resize(num_couplers); }

  /* Returns the number of gain constraints. */
  int num_constraints() const { return gamma_pool_.size(); }

  const T& gamma(int k) const { return gamma_pool_[k]; }
  T& gamma(int k) { return gamma_pool_[k]; }

  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};  // Total cost over all gain constraints.
  std::vector<T> gamma_pool_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::
        CouplerConstraintsDataPool);
