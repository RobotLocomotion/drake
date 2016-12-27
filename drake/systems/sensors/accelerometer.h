#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simulated ideal accelerometer that measures linear acceleration.
///
/// <B>System Input Ports:</B>
///
/// There are two input ports that are accessible via the following accessors:
///
///  - get_state_input_port(): Contains the state (i.e., postion and velocity)
///    vector, `x`, of the RigidBodyPlant that contains the RigidBodyTree being
///    sensed by this sensor.
///  - get_state_derivatives_input_port(): Contains the derivative of the state
///    vector, `xdot`, of the RigidBodyPlant that contains the RigidBodyTree
///    being sensed by this sensor.
///
/// <B>System Output Ports:</B>
///
/// This system has one output port that is accessible via the following
/// accessor method:
///
///  - get_acceleration_output_port(): Contains the sensed acceleration data in
///    this sensor's frame.
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
  /// @param[in] name The name of the accelerometer. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance within @p tree.
  ///
  /// @param[in] frame The frame to which this accelerometer is attached. This
  /// is the frame in which this sensor's output is given. A copy of this frame
  /// is stored as a class member variable.
  ///
  /// @param[in] tree The RigidBodyTree being sensed by this sensor. This should
  /// be a reference to the same RigidBodyTree that is being used by the
  /// RigidBodyPlant whose outputs are fed into this sensor. This parameter is
  /// aliased by a class member variable so its lifespan must exceed that of
  /// this class' instance.
  ///
  /// @param[in] include_gravity_compensation Whether to include the
  //  accleration due to gravity in the sensor's reading.
  ///
  Accelerometer(const std::string& name, const RigidBodyFrame<double>& frame,
    const RigidBodyTree<double>& tree,
    bool include_gravity_compensation = true);

  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// Accelerometer is neither copyable nor moveable.
  ///@{
  explicit Accelerometer(const Accelerometer&) = delete;
  Accelerometer& operator=(const Accelerometer&) = delete;
  ///@}

  /// Returns the name of this sensor. The name can be any user-specified value.
  const std::string& get_name() const { return name_; }

  /// Returns the RigidBodyPlant that contains the RigidBodyTree to which this
  /// sensor is attached.
  // const RigidBodyPlant<double>& get_plant() const { return plant_; }

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  ///
  /// @see get_tree()
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns a descriptor of the input port that should contain the generalized
  /// (i.e., linear and rotational) position and velocity state of the
  /// RigidBodyTree DOFs.
  const InputPortDescriptor<double>& get_state_input_port() const {
    return System<double>::get_input_port(state_input_port_index_);
  }

  /// Returns a descriptor of the input port that should contain the derivative
  /// of the generalized (i.e., linear and rotational) position and velocity
  /// state of the RigidBodyTree DOFs.
  const InputPortDescriptor<double>& get_state_derivative_input_port() const {
    return System<double>::get_input_port(state_derivative_input_port_index_);
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
  /// Computes the linear acceleration as sensed by this sensor.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const std::string name_;
  const RigidBodyFrame<double> frame_;
  const RigidBodyTree<double>& tree_;
  const bool include_gravity_compensation_{};

  int state_input_port_index_{};
  int state_derivative_input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
