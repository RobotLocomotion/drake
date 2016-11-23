#include "drake/multibody/rigid_body_plant/rigid_body_plant_context.h"

namespace drake {
namespace systems {

template <typename T>
RigidBodyPlantContext<T>::RigidBodyPlantContext(const RigidBodyPlant<T>& rbp) :
    kinematics_cache_(rbp.get_rigid_body_tree().bodies) {
}

template class RigidBodyPlantContext<double>;

}  // namespace systems
}  // namespace drake
