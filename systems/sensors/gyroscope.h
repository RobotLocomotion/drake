#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// Sensor to represent an ideal gyroscopic sensor. Currently does not
/// represent noise or bias, but this could and should be added at a later
/// date. This sensor measures the angular velocity of a given body B relative
/// to the world frame. The sensor frame S is rigidly affixed to the given
/// body B. The measurement, written w_WS_S, is expressed in the coordinates
/// of frame S. Note that, since S is fixed to B, the angular velocity of the
/// two frames is identical, w_WS_S = w_WB_S.
///
/// There are two inputs to this sensor (nominally from a MultibodyPlant):
///   1. A vector of body poses (e.g. plant.get_body_poses_output_port()),
///   2. A vector of spatial velocities,
///    (e.g. plant.get_body_spatial_velocities_output_port()).
///
/// This class is therefore defined by:
///   1. The Body to which this sensor is rigidly affixed,
///   2. The pose of the sensor frame in the body frame.
/// Note that the translational component of the transformation is not strictly
/// needed by a gyroscope, as the position of the sensor does not affect the
/// measurement. However, as a sensor does have a physical location, it is
/// included here for completeness and in case it might be useful for display or
/// other purposes.
///
/// @system
/// name: Gyroscope
/// input_ports:
/// - body_poses
/// - body_spatial_velocities
/// output_ports:
/// - measurement
/// @endsystem
///
/// @ingroup sensor_systems
template <typename T>
class Gyroscope final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Gyroscope)

  /// Constructor for %Gyroscope using full transform.
  /// @param body the body B to which the sensor is affixed
  /// @param X_BS the pose of sensor frame S in body B
  Gyroscope(const multibody::Body<T>& body,
            const math::RigidTransform<double>& X_BS);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Gyroscope(const Gyroscope<U>&);

  const InputPort<T>& get_body_poses_input_port() const {
    return *body_poses_input_port_;
  }

  const InputPort<T>& get_body_velocities_input_port() const {
    return *body_velocities_input_port_;
  }

  const OutputPort<T>& get_measurement_output_port() const {
    return *measurement_output_port_;
  }

  /// Returns the index of the Body that was supplied in the constructor.
  const multibody::BodyIndex& body_index() const { return body_index_; }

  /// Gets X_BS, the pose of sensor frame S in body B.
  const math::RigidTransform<double>& pose() const {
    return X_BS_;
  }

  /// Static factory method that creates a %Gyroscope object and connects
  /// it to the given plant. Modifies a Diagram by connecting the input ports
  /// of the new %Gyroscope to the appropriate output ports of a
  /// MultibodyPlant. Must be called during Diagram building and given the
  /// appropriate builder. This is a convenience method to simplify some common
  /// boilerplate of Diagram wiring. Specifically, this makes three connections:
  ///
  /// 1. plant.get_body_poses_output_port() to this.get_body_poses_input_port()
  /// 2. plant.get_body_spatial_velocities_output_port() to
  ///        this.get_body_velocities_input_port()
  /// @param body the body B to which the sensor is affixed
  /// @param X_BS X_BS the pose of sensor frame S in body B
  /// @param plant the plant to which the sensor will be connected
  /// @param builder a pointer to the DiagramBuilder
  static const Gyroscope& AddToDiagram(
      const multibody::Body<T>& body, const math::RigidTransform<double>& X_BS,
      const multibody::MultibodyPlant<T>& plant, DiagramBuilder<T>* builder);

 private:
  Gyroscope(const multibody::BodyIndex& body_index,
            const math::RigidTransform<double>& X_BS);

  // Computes the transformed signal.
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const;

  const multibody::BodyIndex body_index_;
  const math::RigidTransform<double> X_BS_;
  const InputPort<T>* body_poses_input_port_{nullptr};
  const InputPort<T>* body_velocities_input_port_{nullptr};
  const OutputPort<T>* measurement_output_port_{nullptr};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
