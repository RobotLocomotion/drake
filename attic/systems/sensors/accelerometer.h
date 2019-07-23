#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/sensors/accelerometer_output.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simulated ideal accelerometer that measures the linear acceleration of a
/// frame associated with a RigidBodyPlant.
///
/// The math implemented by this sensor is as follows. Let:
///
/// - `v` be the RigidBodyPlant's generalized velocity state vector.
/// - `A` be the accelerometer's frame with origin point `Ao`.
/// - `W` be the world frame with origin point `Wo`.
/// - `v_WAo_W` be `Ao`'s translational velocity in `W` expressed in `W`.
/// - `J_WA` be the Jacobian matrix that relates `v_WAo_W` to `v`.
/// - `R_AW` be the rotation matrix from `W` to `A`.
///
/// The equation for `v_WAo_W` is as follows:
///
/// <pre>
/// v_WAo_W = J_WA v
/// </pre>
///
/// Let `a_WAo_W` be the `Ao`'s translational acceleration in `W` expressed in
/// `W`. `a_WAo_W` can be computed by taking the time derivative of `v_WAo_W`,
/// which is derived as follows using the chain rule:
///
/// <pre>
/// a_WAo_W = d/dt v_WAo_W = J_WA vdot_WAo_W + Jdot_WA v_WAo_W
/// </pre>
///
/// `a_WAo_W` is then re-expressed in `A` using the rotation matrix `R_AW`:
///
/// <pre>
/// a_WAo_A = R_AW a_WAo_W
/// </pre>
///
/// Note that `a_WA_A` does not include gravity. The constructor includes
/// boolean input parameter `include_gravity`, which allows the user to specify
/// whether the accelerometer should also sense the acceleration due to gravity.
/// If gravity is to be included, this acceleration performs the following
/// additional math:
///
/// Let:
///
/// - `g_W` be the acceleration due to gravity in `W`, which is equal to
///   (-1) * RigidBodyTree::a_grav. Note that the gravity pulls the mass inside
///   the accelerometer downward, resulting in an upward measured acceleration.
///   See the first paragraph of Wikipedia for more detail:
///   https://en.wikipedia.org/wiki/Accelerometer
/// - `g_A` be the acceleration due to gravity in `A`.
///
/// `g_A` is computed as follows:
///
/// <pre>
/// g_A = R_AW * g_W
/// </pre>
///
/// `g_A` is then added to `a_WAo_A`:
///
/// <pre>
/// a_WAo_A = a_WAo_A + g_A
/// </pre>
///
/// This concludes the discussion of what is computed by this accelerometer.
///
/// <B>%System Input Ports:</B>
///
/// There are two input ports that are accessible via the following accessors:
///
///  - get_plant_state_input_port(): Contains the state (i.e., position and
///    velocity) vector, `x`, of the RigidBodyPlant is being sensed by this
///    sensor.
///
///  - get_plant_state_derivative_input_port(): Contains the derivative of the
///    state vector, `xdot`, of the RigidBodyPlant being sensed by this sensor.
///
/// <B>%System Output Ports:</B>
///
/// This system has one output port that is accessible via the following
/// accessor method:
///
///  - get_output_port(): Contains the sensed acceleration data in this sensor's
///    frame.
///
/// @ingroup sensor_systems
///
class Accelerometer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Accelerometer);

  /// A constructor that initializes an Accelerometer.
  ///
  /// @param[in] name The name of the accelerometer. This can be any value.
  ///
  /// @param[in] frame The frame `A` to which this accelerometer is attached
  /// (see class documentation for definition of `A`). This is the frame in
  /// which this accelerometer's output is given. This reference must remain
  /// valid for the lifetime of this class's instance.
  ///
  /// @param[in] tree The RigidBodyTree that belongs to the RigidBodyPlant being
  /// sensed by this sensor. This should be a reference to the same
  /// RigidBodyTree that is being used by the RigidBodyPlant whose outputs are
  /// fed into this sensor. This parameter's lifespan must exceed that of this
  /// class's instance.
  ///
  /// @param[in] include_gravity Whether to include the acceleration due to
  /// gravity in the sensor's readings. See this class's description for more
  /// details about the meaning of this parameter.
  ///
  Accelerometer(const std::string& name, const RigidBodyFrame<double>& frame,
      const RigidBodyTree<double>& tree, bool include_gravity = true);

  /// Instantiates and attaches an Accelerometer to a RigidBodyPlant. It
  /// connects the Accelerometer's plant state input port. <b>It does not
  /// connect the plant state derivative input port.</b>
  ///
  /// @param[in] name The name of the sensor. This can be any value.
  ///
  /// @param[in] frame The frame to which the Accelerometer is attached.
  ///
  /// @param[in] plant The plant that the newly instantiated Accelerometer must
  /// sense. The provided plant must be in the provided builder.
  ///
  /// @param[in] include_gravity Whether to include the acceleration due to
  /// gravity in the sensor's readings.
  ///
  /// @param[out] builder A pointer to the DiagramBuilder to which the newly
  /// instantiated Accelerometer is added. This must not be `nullptr`.
  ///
  /// @return A pointer to the newly instantiated and added accelerometer. The
  /// accelerometer is initially owned by the builder. Ownership will
  /// subsequently be transferred to the Diagram that is built by the builder.
  static Accelerometer* AttachAccelerometer(
      const std::string& name,
      const RigidBodyFrame<double>& frame,
      const RigidBodyPlant<double>& plant,
      bool include_gravity,
      DiagramBuilder<double>* builder);

  /// Returns whether gravity is included in this sensor's measurements.
  bool get_include_gravity() const { return include_gravity_; }

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  ///
  /// @see get_tree()
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns the input port that should contain the generalized position and
  /// velocity vector of the RigidBodyPlant that this sensor is sensing.
  const InputPort<double>& get_plant_state_input_port() const {
    return System<double>::get_input_port(plant_state_input_port_index_);
  }

  /// Returns the input port that should contain the derivative of the
  /// generalized position and velocity vector of the RigidBodyPlant that this
  /// sensor is sensing.
  const InputPort<double>& get_plant_state_derivative_input_port()
      const {
    return System<double>::get_input_port(
        plant_state_derivative_input_port_index_);
  }

  /// Returns the state output port, which contains the sensor's
  /// sensed values.
  const OutputPort<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }

 private:
  // Computes the "sensed" linear acceleration.
  void CalcAccelerationOutput(const Context<double>& context,
                              AccelerometerOutput<double>* output_vector) const;

  const std::string name_;
  const RigidBodyFrame<double> frame_;
  const RigidBodyTree<double>& tree_;
  const bool include_gravity_{true};

  int plant_state_input_port_index_{};
  int plant_state_derivative_input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
