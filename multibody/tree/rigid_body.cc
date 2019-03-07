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

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody)
