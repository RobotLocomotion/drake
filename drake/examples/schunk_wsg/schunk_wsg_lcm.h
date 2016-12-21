#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the Schunk WSG gripper.

#include "drake/common/trajectories/trajectory.h"
#include "drake/examples/schunk_wsg/gen/schunk_wsg_trajectory_generator_state_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace schunk_wsg {

// TODO(sam.creasey) Right now this class just outputs a position
// which is not going to be sufficient to capture the entire control
// state of the gripper (particularly the maximum force).

/// Receives lcmt_schunk_wsg_command for a Schunk WSG (input port 0)
/// along with the current state of the simulated WSG (input port 1),
/// and emits target position/velocity for the actuated finger to
/// reach the commanded target.
class SchunkWsgTrajectoryGenerator : public systems::LeafSystem<double> {
 public:
  /// @param input_size The size of the state input port to create
  /// (one reason this may vary is passing in the entire state of a
  /// rigid body tree vs. having already demultiplexed the actuated
  /// finger).
  /// @param position_index The index in the state input vector
  /// which contains the position of the actuated finger.
  SchunkWsgTrajectoryGenerator(int input_size, int position_index);

  const systems::SystemPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(0);
  }

  const systems::SystemPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(1);
  }

 protected:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  /// Latches the input port into the discrete state.
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      systems::DiscreteState<double>* discrete_state) const override;

  std::unique_ptr<systems::DiscreteState<double>> AllocateDiscreteState()
      const override;

 private:
  void UpdateTrajectory(double cur_position, double target_position) const;

  /// The minimum change between the last received command and the
  /// current command to trigger a trajectory update.  Based on
  /// manually driving the actual gripper using the web interface, it
  /// appears that it will at least attempt to respond to commands as
  /// small as 0.1mm.
  const double kTargetEpsilon = 0.0001;

  const int position_index_{};
  // TODO(sam.creasey) I'd prefer to store the trajectory as
  // discrete state, but unfortunately that's not currently possible
  // as DiscreteState may only contain BasicVector.
  mutable std::unique_ptr<Trajectory> trajectory_;
};

/// Sends lcmt_schunk_wsg_status messages for a Schunk WSG.  This
/// system has one input port for the current state of the simulated
/// WSG (probably a RigidBodyPlant).
class SchunkWsgStatusSender : public systems::LeafSystem<double> {
 public:
  SchunkWsgStatusSender(int input_size,
                        int position_index, int velocity_index);

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

 private:
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  const int position_index_{};
  const int velocity_index_{};
};

}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
