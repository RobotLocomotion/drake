#pragma once

#include <string>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace hsr {

/// Contains the necessary information about a loaded model instance.
/// This information will be useful if extra care is needed for this instance.
/// For example, the X_PC will be used to set the initial position if this
/// instance has a floating base.
template <typename T>
struct ModelInstanceInfo {
  std::string model_name;
  std::string model_path;
  std::string parent_frame_name;
  std::string child_frame_name;
  multibody::ModelInstanceIndex index;
  math::RigidTransform<T> X_PC;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
