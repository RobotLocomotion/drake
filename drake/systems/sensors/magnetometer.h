#pragma once

#include <limits>
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

/// A simulated ideal magnetometer that measures the unit vector that points to
/// true north in the sensor's frame.
///
/// <B>The Magnetometer Math:</B>
///
/// Let:
///  - `M` be the magnetometer's frame.
///  - `W` be the world frame with origin `Wo`.
///  - `N` be a point in the direction of true north.
///  - `p_WoN_W` be a unit vector from `Wo` to `N` that points to true
///     north express in `W`.
///  - `p_WoN_M` be the same as `p_WoN_W` except expressed in `M`.
///  - `R_MW` be the rotation matrix from `W` to `M`.
///
/// The math implemented by this sensor is as follows:
///
/// <pre>
/// p_WoN_M = R_MW * p_WoN_W
/// </pre>
///
/// The output is `p_WoN_M`.
///
/// <B>System Input Ports:</B>
///
/// This system has one input port that is accessible via the following
/// accessor:
///
///  - get_input_port(): Contains the state vector, `x`, of the RigidBodyPlant
///    being sensed by this sensor.
///
/// <B>System Output Ports:</B>
///
/// This system has one output port that is accessible via the following
/// accessor method:
///
///  - get_output_port(): Contains `p_WoN_M`. See MagnetometerOutput.
///
/// @ingroup sensor_systems
///
class Magnetometer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Magnetometer);

  /// A constructor.
  ///
  /// @param[in] name The name of the magnetometer. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance within @p tree.
  ///
  /// @param[in] frame The frame, `M`, to which this magnetometer is attached.
  /// This is the frame in which this sensor's output is given. It need not be
  /// in the provided `tree`, but must reference a body in the `tree`. For more
  /// details, see this class's documentation.
  ///
  /// @param[in] tree The RigidBodyTree that belongs to the RigidBodyPlant being
  /// sensed by this sensor. This parameter's lifespan must exceed this class's
  /// instance.
  ///
  /// @param[in] north_vector A unit vector, `p_WoN_W`, that points to true
  /// north in the world frame.
  ///
  Magnetometer(const std::string& name, const RigidBodyFrame<double>& frame,
               const RigidBodyTree<double>& tree,
               const Eigen::Vector3d& north_vector = Eigen::Vector3d(1, 0, 0));

  /// Returns this sensor's name.
  const std::string& get_name() const { return name_; }

  /// Returns the RigidBodyTree being used by this sensor.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns `M` (see this class's documentation).
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns a descriptor of the input port that contains `x`, the generalized
  /// state of the RigidBodyPlant.
  const InputPortDescriptor<double>& get_input_port() const {
    return System<double>::get_input_port(input_port_index_);
  }

  /// Returns a descriptor of the state output port, which contains the sensed
  /// values.
  ///
  /// @see MagnetometerOutput
  const OutputPortDescriptor<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }

  /// Allocates the output vector.
  ///
  /// @see MagnetometerOutput
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const OutputPortDescriptor<double>& descriptor) const override;

 protected:
  /// Computes the angular divergence from true north as sensed by this sensor.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const std::string name_;
  const RigidBodyFrame<double> frame_;
  const RigidBodyTree<double>& tree_;
  const Eigen::Vector3d p_WoN_W_;
  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
