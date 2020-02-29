#pragma once

#include <memory>
#include <vector>

#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {


// TODO(sam.creasey) Right now this class just outputs a position
// which is not going to be sufficient to capture the entire control
// state of the gripper (particularly the maximum force).

/// This system defines input ports for the desired finger position
/// represented as the desired distance between the fingers in
/// meters and the desired force limit in newtons, and emits target
/// position/velocity for the actuated finger to reach the commanded target,
/// expressed as the negative of the distance between the two fingers in
/// meters.  The force portion of the command message is passed through this
/// system, but does not affect the generated trajectory.  The
/// desired_position and force_limit are scalars (BasicVector<double> of size
/// 1).
///
/// @ingroup manipulation_systems
class SchunkWsgTrajectoryGenerator : public systems::LeafSystem<double> {
 public:
  /// @param input_size The size of the state input port to create
  /// (one reason this may vary is passing in the entire state of a
  /// rigid body tree vs. having already demultiplexed the actuated
  /// finger).
  /// @param position_index The index in the state input vector
  /// which contains the position of the actuated finger.
  SchunkWsgTrajectoryGenerator(int input_size, int position_index);

  const systems::InputPort<double>& get_desired_position_input_port() const {
    return get_input_port(desired_position_input_port_);
  }

  const systems::InputPort<double>& get_force_limit_input_port() const {
    return get_input_port(force_limit_input_port_);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return get_input_port(state_input_port_);
  }

  const systems::OutputPort<double>& get_target_output_port() const {
    return this->get_output_port(target_output_port_);
  }

  const systems::OutputPort<double>& get_max_force_output_port() const {
    return this->get_output_port(max_force_output_port_);
  }

 private:
  void OutputTarget(const systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

  void OutputForce(const systems::Context<double>& context,
                   systems::BasicVector<double>* output) const;

  /// Latches the input port into the discrete state.
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const override;

  void UpdateTrajectory(double cur_position, double target_position) const;

  /// The minimum change between the last received command and the
  /// current command to trigger a trajectory update.  Based on
  /// manually driving the actual gripper using the web interface, it
  /// appears that it will at least attempt to respond to commands as
  /// small as 0.1mm.
  const double kTargetEpsilon = 0.0001;

  const int position_index_{};
  const systems::InputPortIndex desired_position_input_port_{};
  const systems::InputPortIndex force_limit_input_port_{};
  const systems::InputPortIndex state_input_port_{};
  const systems::OutputPortIndex target_output_port_{};
  const systems::OutputPortIndex max_force_output_port_{};

  // TODO(sam.creasey) I'd prefer to store the trajectory as
  // discrete state, but unfortunately that's not currently possible
  // as DiscreteValues may only contain BasicVector.
  mutable std::unique_ptr<trajectories::Trajectory<double>> trajectory_;
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
