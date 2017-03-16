#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simulated ideal gyroscope. It measures angular velocity along three
/// dimensions.
///
/// <B>The Gyroscope Math:</B>
///
/// Let:
///  - `G` be the gyroscope's frame.
///  - `W` be the world frame.
///  - `ω_G_W` be `G`'s rotational velocity expressed in `W`.
///  - `R_GW` be the rotation matrix from `W` to `G`.
///  - `ω_G_G` be `G`'s rotational velocity expressed in `G`. This is the output
///     of the gyroscope.
///
/// The math implemented by this sensor is as follows:
///
/// <pre>
/// ω_G_G = R_GW * ω_G_W
/// </pre>
///
/// <B>System Input Ports:</B>
///
/// This system has one input port that is accessible via the following
/// accessor:
///
///  - get_input_port(): Contains the state (i.e., position and velocity)
///    vector, `x`, of the RigidBodyPlant being sensed by this sensor.
///
/// <B>System Output Ports:</B>
///
/// This system has one output port that is accessible via the following
/// accessor method:
///
///  - get_output_port(): Contains the sensed angular velocity data in this
///    sensor's frame. See GyroscopeOutput.
///
/// @ingroup sensor_systems
///
class Gyroscope : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Gyroscope);

  /// A constructor.
  ///
  /// @param[in] name The name of the gyroscope. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance within @p tree.
  ///
  /// @param[in] frame The frame to which this gyroscope is attached. This
  /// is the frame in which this sensor's output is given. It need not be in the
  /// provided `tree`, but must reference a body in the `tree`.
  ///
  /// @param[in] tree The RigidBodyTree that belongs to the RigidBodyPlant being
  /// sensed by this sensor. This should be a reference to the same
  /// RigidBodyTree that is being used by the RigidBodyPlant whose outputs are
  /// fed into this sensor. This parameter's lifespan must exceed that of this
  /// class's instance.
  ///
  Gyroscope(const std::string& name, const RigidBodyFrame<double>& frame,
            const RigidBodyTree<double>& tree);

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
  /// RigidBodyTree DOFs.
  const InputPortDescriptor<double>& get_input_port() const {
    return System<double>::get_input_port(input_port_index_);
  }

  /// Returns a descriptor of the state output port, which contains the sensor's
  /// sensed values.
  const OutputPortDescriptor<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }

 protected:
  /// Computes the angular velocity as sensed by this sensor.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const std::string name_;
  const RigidBodyFrame<double> frame_;
  const RigidBodyTree<double>& tree_;

  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
