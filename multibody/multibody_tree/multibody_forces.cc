#include "drake/multibody/multibody_tree/multibody_forces.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyForces<T>::MultibodyForces(const MultibodyTree<T>& model) {
  DRAKE_DEMAND(model.topology_is_valid());
  F_B_W_.resize(model.get_num_bodies(), SpatialForce<T>::Zero());
  tau_ = VectorX<T>::Zero(model.get_num_velocities());
}

template <typename T>
MultibodyForces<T>& MultibodyForces<T>::SetZero() {
  std::fill(F_B_W_.begin(), F_B_W_.end(), SpatialForce<T>::Zero());
  tau_.setZero();
  return *this;
}

template <typename T>
bool MultibodyForces<T>::CheckHasRightSizeForModel(
    const MultibodyTree<T> &model) const {
  return
      model.get_num_velocities() == num_velocities() &&
      model.get_num_bodies() == num_bodies();
}

template <typename T>
void MultibodyForces<T>::AddInForces(const MultibodyForces<T>& addend) {
  DRAKE_DEMAND(this->num_bodies() == addend.num_bodies());
  DRAKE_DEMAND(this->num_velocities() == addend.num_velocities());
  // Add in body forces:
  auto Faddend = addend.body_forces().cbegin();
  for (auto& F : F_B_W_)
    F += *Faddend++;
  // Add in generalized forces:
  tau_ += addend.generalized_forces();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyForces)
