#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

template <typename T>
RigidBody<T>::RigidBody(const SpatialInertia<double> M) :
    default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double> M)
    : Body<T>(body_name),
      default_spatial_inertia_(M) {}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
