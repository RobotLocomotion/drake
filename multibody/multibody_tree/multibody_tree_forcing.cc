#include "drake/multibody/multibody_tree/multibody_tree_forcing.h"

#include <algorithm>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyTreeForcing<T>::MultibodyTreeForcing(const MultibodyTree<T>& model) :
    F_B_W_(model.get_num_bodies(), SpatialForce<T>::Zero()),
    tau_(VectorX<T>::Zero(model.get_num_velocities())) {}

template <typename T>
MultibodyTreeForcing<T>& MultibodyTreeForcing<T>::SetZero() {
  std::fill(F_B_W_.begin(), F_B_W_.end(), SpatialForce<T>::Zero());
  tau_.setZero();
  return *this;
}

template <typename T>
bool MultibodyTreeForcing<T>::CheckInvariants(
    const MultibodyTree<T>& model) const {
  return
      model.get_num_velocities() == num_mobilities() &&
      model.get_num_bodies() == num_bodies();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyTreeForcing)
