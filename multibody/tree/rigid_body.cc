#include "drake/multibody/tree/rigid_body.h"

#include <memory>

#include "drake/multibody/tree/model_instance.h"

namespace drake {
namespace multibody {

template <typename T>
RigidBody<T>::RigidBody(const SpatialInertia<double>& M)
    : Body<T>("", default_model_instance(), M.get_mass()),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, default_model_instance(), M.get_mass()),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        ModelInstanceIndex model_instance,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, model_instance, M.get_mass()),
      default_spatial_inertia_(M) {}

template <typename T>
void RigidBody<T>::DoLock(systems::Context<T>* context) const {
  DRAKE_DEMAND(this->is_floating());
  const auto& tree = this->get_parent_tree();
  auto& state = context->get_mutable_state();
  const int start = this->floating_velocities_start();
  static constexpr int kVelocities = 6;
  tree.template get_mutable_state_segment<kVelocities>(
      &state, start).setZero();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody)
