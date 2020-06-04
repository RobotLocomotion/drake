#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// Sensor to represent a simple accelerometer sensor. Currently does not
/// represent noise or bias, but this could and should be added at a later
/// date.
///
/// There are three inputs to this sensor (nominally from a MultibodyPlant):
///   1) A vector of body poses (e.g. plant.body_poses)
///   2) A vector of spatial velocities (e.g. plant.body_spatial_velocities)
///   3) A vector of spatial accelerations
///      (e.g. plant.body_spatial_accelerations)
/// This class is therefore defined by:
///   1) An integer body index into the two vector inputs above. The sensor is
///      fixed to this body.
///   2) A rigid transform from the body frame to the sensor frame
/// @ingroup sensor_systems
template <typename T>
class Accelerometer : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Accelerometer)

  /// @param body_index The index of body B
  /// @param X_BC the transform from body B to the accelerometer frame C
  Accelerometer(multibody::BodyIndex index, math::RigidTransform<T> X_BC);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Accelerometer(const Accelerometer<U>&);

  const InputPort<T>& get_body_poses_input_port() const {
    return *body_poses_input_port_;
  }

  const InputPort<T>& get_body_velocities_input_port() const {
    return *body_velocities_input_port_;
  }

  const InputPort<T>& get_body_accelerations_input_port() const {
    return *body_accelerations_input_port_;
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class Accelerometer;

  // Outputs the transformed signal.
 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const;

  const multibody::BodyIndex index_;  // Index into the input body velocities/accelerations
  const math::RigidTransform<T> X_BC_;
  const InputPort<T>* body_poses_input_port_{};
  const InputPort<T>* body_velocities_input_port_{};
  const InputPort<T>* body_accelerations_input_port_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
