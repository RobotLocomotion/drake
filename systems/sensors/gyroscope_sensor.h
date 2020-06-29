#pragma once
// TODO(Posa): Rename header to gyroscope.h once attic/ code is removed.
// Currently requires this alternate name to avoid confliciting with
// attic/systems/sensors/gyroscope.h
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

/// Sensor to represent an ideal gyroscopicse sensor. Currently does not
/// represent noise or bias, but this could and should be added at a later
/// date. This sensor measures the angular velocity of a given body B relative
/// to the world frame. The sensor frame S is rigidly affixed to the given
/// body B. The measurement, written w_WS_S, is is expressed in the coordinates 
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
///   2. A rotation matrix from the body frame to the sensor frame, R_BS.
///
/// @system{Gyroscope,
///    @input_port{body_poses}
///    @input_port{body_spatial_velocities}
///    @output_port{measured_angular_velocity}
/// }
/// @ingroup sensor_systems
template <typename T>
class Gyroscope : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Gyroscope)

  /// @param body the body B to which the sensor is affixed
  /// @param R_BS the rotation from body B to the sensor frame S
  Gyroscope(const multibody::Body<T>& body,
            const math::RotationMatrix<double>& X_BS);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Gyroscope(const Gyroscope<U>&);

  const InputPort<T>& get_body_poses_input_port() const {
    return *body_poses_input_port_;
  }

  const InputPort<T>& get_body_velocities_input_port() const {
    return *body_velocities_input_port_;
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
  /// @param R_BS the rotation from body B to the sensor frame S
  /// @param plant the plant to which the sensor will be connected
  /// @param builder a pointer to the DiagramBuilder
  static const Gyroscope& AddToDiagram(
      const multibody::Body<T>& body, const math::RotationMatrix<double>& R_BS,
      const multibody::MultibodyPlant<T>& plant, DiagramBuilder<T>* builder);

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class Gyroscope;

 private:
  Gyroscope(const multibody::BodyIndex& body_index,
            const math::RotationMatrix<double>& X_BS);

  // Computes the transformed signal.
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const;

  const multibody::BodyIndex body_index_;
  const math::RotationMatrix<double> R_BS_;
  const InputPort<T>* body_poses_input_port_{nullptr};
  const InputPort<T>* body_velocities_input_port_{nullptr};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
