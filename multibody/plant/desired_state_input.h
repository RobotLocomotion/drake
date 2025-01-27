#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct DesiredStateInput {
  // Desired states for all PD controlled DoFs in the plant.
  VectorX<T> xd;
  // Whether a model instance is armed, see @ref pd_controllers_and_ports.
  // Vector indexed by ModelInstanceIndex.
  VectorX<bool> instance_is_armed;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DesiredStateInput);
