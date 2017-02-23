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
///  - `p_WoN_W` be a unit vector from `Wo` to point `N` that points to true
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
  /// @param[in] frame The frame to which this magnetometer is attached. This
  /// is the frame in which this sensor's output is given. It need not be in the
  /// provided `tree`, but must reference a body in the `tree`. It defines `M`,
  /// which is described in this class's documentation.
  ///
  /// @param[in] tree The RigidBodyTree that belongs to the RigidBodyPlant being
  /// sensed by this sensor. This should be a reference to the same
  /// RigidBodyTree that is being used by the RigidBodyPlant whose outputs are
  /// fed into this sensor. This parameter's lifespan must exceed that of this
  /// class's instance.
  ///
  /// @param[in] north_vector A unit vector in `W` that points to true north,
  /// i.e., `p_WoN_W`.
  ///
  Magnetometer(const std::string& name, const RigidBodyFrame<double>& frame,
               const RigidBodyTree<double>& tree,
               const Eigen::Vector3d& north_vector = Eigen::Vector3d(1, 0, 0));

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

  /// Allocates the output vector. See this class' description for details of
  /// this output vector.
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
