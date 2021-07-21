#include "drake/multibody/fixed_fem/dev/fem_state_base.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
void FemStateBase<T>::SetQ(const Eigen::Ref<const VectorX<T>>& value) {
  DRAKE_THROW_UNLESS(value.size() == q_.size());
  InvalidateAllCacheEntries();
  q_ = value;
}

template <typename T>
void FemStateBase<T>::SetQdot(const Eigen::Ref<const VectorX<T>>& value) {
  DRAKE_THROW_UNLESS(ode_order() >= 1);
  DRAKE_THROW_UNLESS(value.size() == qdot_.size());
  InvalidateAllCacheEntries();
  qdot_ = value;
}

template <typename T>
void FemStateBase<T>::SetQddot(const Eigen::Ref<const VectorX<T>>& value) {
  DRAKE_THROW_UNLESS(ode_order() == 2);
  DRAKE_THROW_UNLESS(value.size() == qddot_.size());
  InvalidateAllCacheEntries();
  qddot_ = value;
}

template <typename T>
void FemStateBase<T>::ApplyBoundaryCondition(
    const DirichletBoundaryCondition<T>& bc) {
  const auto& bcs = bc.get_bcs();
  if (bcs.size() == 0) {
    return;
  }
  bc.VerifyBcIndexes(this->num_generalized_positions());
  /* Write the BC to the mutable state. */
  for (const auto& [dof_index, boundary_state] : bcs) {
    q_(int{dof_index}) = boundary_state(0);
    if (ode_order() >= 1) {
      qdot_(int{dof_index}) = boundary_state(1);
    }
    if (ode_order() == 2) {
      qddot_(int{dof_index}) = boundary_state(2);
    }
  }
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemStateBase);
