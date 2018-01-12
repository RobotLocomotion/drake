#include "drake/multibody/multibody_tree/multibody_forces.h"

#include <algorithm>
#include <functional>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyForces<T>::MultibodyForces(const MultibodyTree<T>& model) {
  DRAKE_DEMAND(model.topology_is_valid());
  F_B_W_.resize(model.get_num_bodies(), SpatialForce<T>::Zero());
  tau_.resize(model.get_num_velocities());
}

template <typename T>
MultibodyForces<T>& MultibodyForces<T>::SetZero() {
  std::fill(F_B_W_.begin(), F_B_W_.end(), SpatialForce<T>::Zero());
  tau_.setZero();
  return *this;
}

template <typename T>
bool MultibodyForces<T>::CheckInvariants(
    const MultibodyTree<T>& model) const {
  return
      model.get_num_velocities() == num_velocities() &&
      model.get_num_bodies() == num_bodies();
}

template <typename T>
void MultibodyForces<T>::AddInForcing(
    const MultibodyForces<T>& addend) {
  // Add in body forces:
  std::transform(
      F_B_W_.begin(), F_B_W_.end(),
      addend.body_forces().begin(),
      F_B_W_.begin(), std::plus<SpatialForce<T>>());
  // Add in generalized forces:
  tau_ += addend.generalized_forces();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyForces)
