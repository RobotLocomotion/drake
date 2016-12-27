#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simulated ideal accelerometer that measures the linear acceleration of a
/// frame associated with a RigidBodyPlant.
///
/// The math implemented by this sensor is as follows. Let `V` be the
/// RigidBodyPlant's velocity state vector, `V_wp` be the linear velocity of
/// a point `p` in the world frame, and `J_wp` be the Jacobian matrix that
/// relates `p`'s linear velocity vector to `V`. The equation for `V_wp` is as
/// follows:
///
/// @f[
/// V_{wp} = J_{wp} V
/// @f]
///
/// The linear acceleration of point `p` in the world frame, `A_wp`, can
/// be computed by taking the time derivative of `V_wp`, which is derived as
/// follows using the chain rule:
///
/// @f[
/// A_{wp} = J_{wp} \dot{V} + \dot{J}_{wp} V
/// @f]
///
/// The acceleration vector `A_wp` is then transformed into the sensor's frame
/// and outputted optionally including the effects of gravity.
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
  /// This accelerometer measures linear acceleration along 3 axes within its
  /// frame: X, Y, and Z.
  static constexpr int kNumMeasurements{3};

  /// A constructor that initializes an Accelerometer.
  ///
  /// @param[in] name The name of the accelerometer. This can be any value.
  ///
  /// @param[in] frame The frame to which this accelerometer is attached. This
  /// is the frame in which this sensor's output is given. A copy of this frame
  /// is stored as a class member variable.
  ///
  /// @param[in] tree The RigidBodyTree that belongs to the RigidBodyPlatn being
  /// sensed by this sensor. This should be a reference to the same
  /// RigidBodyTree that is being used by the RigidBodyPlant whose outputs are
  /// fed into this sensor. This parameter is aliased by a class member variable
  /// so its lifespan must exceed that of this class' instance.
  ///
  /// @param[in] include_gravity Whether to include the acceleration due to
  /// gravity in the sensor's readings.
  ///
  Accelerometer(const std::string& name, const RigidBodyFrame<double>& frame,
      const RigidBodyTree<double>& tree, bool include_gravity = true);

  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// Accelerometer is neither copyable nor moveable.
  ///@{
  explicit Accelerometer(const Accelerometer&) = delete;
  Accelerometer& operator=(const Accelerometer&) = delete;
  ///@}

  /// Instantiates and attaches an Accelerometer to a RigidBodyPlant. It
  /// connects the Accelerometer's plant state input port. <b>It does not
  /// connect the plant state derivative input port.</b>
  ///
  /// @param[in] name The name of the sensor. This can be any value.
  ///
  /// @param[in] frame The frame to which the Accelerometer is attached. This
  /// must be a frame within the provided plant.
  ///
  /// @param[in] plant The plant that the newly instantiated Accelerometer must
  /// sense. The provided plant must be in the provided builder.
  ///
  /// @param[in] include_gravity Whether to include the acceleration due to
  /// gravity in the sensor's readings.
  ///
  /// @param[out] builder A pointer to the DiagramBuilder to which the newly
  /// instantiated Accelerometer is added. This must not be nullptr.
  ///
  /// @return A pointer to the newly instantiated and added Accelerometer.
  static Accelerometer* AttachAccelerometer(
      const std::string& name,
      const RigidBodyFrame<double>& frame,
      const RigidBodyPlant<double>& plant,
      bool include_gravity,
      DiagramBuilder<double>* builder);

  /// Returns the name of this sensor. The name can be any user-specified value.
  const std::string& get_name() const { return name_; }

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  ///
  /// @see get_tree()
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns a descriptor of the input port that should contain the generalized
  /// (i.e., linear and rotational) position and velocity state of the
  /// RigidBodyPlant that this sensor is sensing.
  const InputPortDescriptor<double>& get_plant_state_input_port() const {
    return System<double>::get_input_port(plant_state_input_port_index_);
  }

  /// Returns a descriptor of the input port that should contain the derivative
  /// of the generalized (i.e., linear and rotational) position and velocity
  /// state of the RigidBodyTree DOFs.
  const InputPortDescriptor<double>& get_plant_state_derivative_input_port()
      const {
    return System<double>::get_input_port(
        plant_state_derivative_input_port_index_);
  }

  /// Returns a descriptor of the state output port, which contains the sensor's
  /// sensed values.
  const OutputPortDescriptor<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }

  /// Allocates the output vector. See this class' description for details of
  /// this output vector.
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const OutputPortDescriptor<double>& descriptor) const override;

  friend std::ostream& operator<<(std::ostream& out,
                                  const Accelerometer& depth_sensor);

 protected:
  /// Computes the "sensed" linear acceleration.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const std::string name_;
  const RigidBodyFrame<double> frame_;
  const RigidBodyTree<double>& tree_;
  const bool include_gravity_{};

  int plant_state_input_port_index_{};
  int plant_state_derivative_input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
