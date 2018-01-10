#include "drake/multibody/multibody_tree/multibody_forcing.h"

#include <algorithm>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyForcing<T>::MultibodyForcing(const MultibodyTree<T>& model) :
    F_B_W_(model.get_num_bodies(), SpatialForce<T>::Zero()),
    tau_(VectorX<T>::Zero(model.get_num_velocities())) {}

template <typename T>
MultibodyForcing<T>& MultibodyForcing<T>::SetZero() {
  std::fill(F_B_W_.begin(), F_B_W_.end(), SpatialForce<T>::Zero());
  tau_.setZero();
  return *this;
}

template <typename T>
bool MultibodyForcing<T>::CheckInvariants(
    const MultibodyTree<T>& model) const {
  return
      model.get_num_velocities() == num_mobilities() &&
      model.get_num_bodies() == num_bodies();
}

template <typename T>
void MultibodyForcing<T>::AddInForcing(
    const MultibodyForcing<T>& addend) {
  // Add in body forces:
  std::transform(
      F_B_W_.begin( ), F_B_W_.end( ),
      addend.body_forces().begin( ),
      F_B_W_.begin( ), std::plus<SpatialForce<T>>());
  // Add in generalized forces:
  tau_ += addend.generalized_forces();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyForcing)
