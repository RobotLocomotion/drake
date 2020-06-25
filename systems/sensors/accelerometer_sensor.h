#pragma once
// TODO(Posa): Rename header to accelerometer.h once attic/ code is removed.
// Currently requires this alternate name to avoid confliciting with
// attic/systems/sensors/accelerometer.h
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// Sensor to represent an ideal accelerometer sensor. Currently does not
/// represent noise or bias, but this could and should be added at a later
/// date. This sensor measures the proper acceleration of a point on a given
/// body B. Proper acceleration subtracts gravity from the coordinate
/// acceleration a_WS_S. That is,
///   aproper_WS_S = a_WS_S - g_S
/// Note that measurement is taken with respect to the world frame,
/// but expressed in the coordinates of the local sensor frame S.
/// Sensor frame S is rigidly affixed to the given body B. Note, also, the sign
/// of the gravity component. For typical settings (e.g. on Earth assuming a
/// constant gravitational field), the direction of "-g_S" is "upwards."
///
/// There are three inputs to this sensor (nominally from a MultibodyPlant):
///   1. A vector of body poses (e.g. plant.get_body_poses_output_port())
///   2. A vector of spatial velocities
///    (e.g. plant.get_body_spatial_velocities_output_port())
///   3. A vector of spatial accelerations
///      (e.g. plant.get_body_spatial_accelerations_output_port())
///
/// This class is therefore defined by:
///   1. The BodyIndex of the body to which this sensor is rigidly affixed.
///   2. A rigid transform from the body frame to the sensor frame.
///
/// @system{Accelerometer,
///    @input_port{body_poses}
///    @input_port{body_spatial_velocities}
///    @input_port{body_spatial_accelerations},
///    @output_port{measured_acceleration}
/// }
/// @ingroup sensor_systems
template <typename T>
class Accelerometer : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Accelerometer)

  /// @param body_index The index of body B
  /// @param X_BS the transform from body B to the accelerometer frame S
  /// @param gravity_vector the constant acceleration due to gravity
  ///    expressed in world coordinates
  Accelerometer(
      multibody::BodyIndex body_index, const math::RigidTransform<double>& X_BS,
      const Eigen::Vector3d& gravity_vector = Eigen::Vector3d::Zero());

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

  /// Static factory method that creates an Accelerometer object and connects
  /// it to the given plant. Modifies a Diagram by connecting the input ports
  /// of the new Accelerometer to the appropriate output ports of a
  /// MultibodyPlant. Must be called during Diagram building and given the
  /// appropriate builder. This is a convenience method to simplify some common
  /// boilerplate of Diagram wiring. Specifically, this makes three connections:
  ///
  /// 1. plant.get_body_poses_output_port() to this.get_body_poses_input_port()
  /// 2. plant.get_body_spatial_velocities_output_port() to
  ///        this.get_body_velocities_input_port()
  /// 3. plant.get_body_spatial_accelerations_output_port() to
  ///        this.get_body_spatial_accelerations_output_port()
  /// @param body_index the index of body B
  /// @param X_BS the transform from body B to the accelerometer frame S
  /// @param gravity_vector the constant acceleration due to gravity
  ///    expressed in world coordinates
  /// @param builder a pointer to the DiagramBuilder
  static const Accelerometer& AddToDiagram(
      multibody::BodyIndex body_index, const math::RigidTransform<double>& X_BS,
      const Eigen::Vector3d& gravity_vector,
      const multibody::MultibodyPlant<T>& plant, DiagramBuilder<T>* builder);

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class Accelerometer;

  // Outputs the transformed signal.
 private:
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const;

  // Index into the input body velocities/accelerations
  const multibody::BodyIndex body_index_;
  const math::RigidTransform<double> X_BS_;
  const Eigen::Vector3d gravity_vector_;
  const InputPort<T>* body_poses_input_port_{nullptr};
  const InputPort<T>* body_velocities_input_port_{nullptr};
  const InputPort<T>* body_accelerations_input_port_{nullptr};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
