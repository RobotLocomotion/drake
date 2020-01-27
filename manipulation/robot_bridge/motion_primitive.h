#pragma once

#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

enum class MotionStatus { kDone, kExecuting, kForceError, kMotionErr, kStuck };

// Output struct for motion primitives.
struct MotionSummary {
  std::string motion_name;
  MotionStatus status{MotionStatus::kDone};
  // Can add other misc outputs here later.
};

/**
 * All motion primitives shares these inputs:
 * - state
 * Outputs:
 * - position
 * - torque
 * - motion summary (status output)
 * Derived class can declare additional input / output ports as needed.
 *
 * All motion primitives are modeled as discrete time systems. The bulk of
 * computation happens in unrestricted updates, and the calc output functions
 * merely copies the stored values from context to output ports.
 *
 * The unrestricted update happens in three main stages:
 * - update primitive logic
 * - compute control (e.g. position, torque)
 * - update motion summary.
 * As a concrete example, let's consider a simple follow joint trajectory
 * example. The first stage will be checking for any new input
 * trajectory. If there is any, it will be copied into the internal state.
 * In the second stage, the internal trajectory is interpolated to set the
 * position command. And motion status is set to kExecuting when the current
 * context time is earlier than the end time of the internal trajectory, and
 * set to kDone afterwards.
 */
class MotionPrimitive : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MotionPrimitive)

  MotionPrimitive(const multibody::MultibodyPlant<double>* plant,
                  double timestep);

  void Initialize(const Eigen::VectorXd& x0,
                  systems::Context<double>* context) const;

  const systems::InputPort<double>& get_state_input() const {
    return *state_input_port_;
  }

  const systems::OutputPort<double>& get_position_output() const {
    return *position_output_port_;
  }

  const systems::OutputPort<double>& get_torque_output() const {
    return *torque_output_port_;
  }

  const systems::OutputPort<double>& get_motion_summary_output() const {
    return *summary_output_port_;
  }

  double timestep() const { return timestep_; }

 protected:
  const multibody::MultibodyPlant<double>& plant() const { return plant_; }

  virtual void DoCalcTorqueOutput(const systems::Context<double>&,
                                  systems::BasicVector<double>* output) const {
    output->SetZero();
  }

  virtual void UpdateLogic(const systems::Context<double>& context,
                           systems::State<double>* state) const = 0;

  virtual void UpdateCommand(const systems::Context<double>& context,
                             systems::State<double>* state) const = 0;

  virtual void UpdateMotionSummary(const systems::Context<double>& context,
                                   systems::State<double>* state) const = 0;

  const systems::BasicVector<double>& get_x_command(
      const systems::State<double>& state) const {
    return state.get_discrete_state(x_command_index_);
  }

  systems::BasicVector<double>& get_mutable_x_command(
      systems::State<double>* state) const {
    return state->get_mutable_discrete_state(x_command_index_);
  }

  const MotionSummary& get_motion_summary(
      const systems::State<double>& state) const {
    return state.get_abstract_state<MotionSummary>(motion_summary_index_);
  }

  MotionSummary& get_mutable_motion_summary(
      systems::State<double>* state) const {
    return state->get_mutable_abstract_state<MotionSummary>(
        motion_summary_index_);
  }

 private:
  void CalcTorqueOutput(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const {
    DoCalcTorqueOutput(context, output);
  }

  void CalcPositionOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  void CalcMotionSummary(const systems::Context<double>& context,
                         MotionSummary* summary) const {
    *summary = get_motion_summary(context.get_state());
  }

  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override final;

  const multibody::MultibodyPlant<double>& plant_;
  const double timestep_;

  const systems::InputPort<double>* state_input_port_;
  const systems::OutputPort<double>* position_output_port_;
  const systems::OutputPort<double>* torque_output_port_;
  const systems::OutputPort<double>* summary_output_port_;

  systems::DiscreteStateIndex x_command_index_;
  systems::AbstractStateIndex motion_summary_index_;
};

class MoveJoint final : public MotionPrimitive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MoveJoint)

  MoveJoint(const multibody::MultibodyPlant<double>* plant, double timestep);

  /**
   * Needs to be an AbstractInput of type
   * trajectories::PiecewisePolynomial<double>.
   */
  const systems::InputPort<double>& get_trajectory_input() const {
    return *traj_input_port_;
  }

 private:
  void UpdateLogic(const systems::Context<double>& context,
                   systems::State<double>* state) const override;

  void UpdateCommand(const systems::Context<double>& context,
                     systems::State<double>* state) const override;

  void UpdateMotionSummary(const systems::Context<double>& context,
                           systems::State<double>* state) const override;

  const trajectories::PiecewisePolynomial<double>& get_command_traj(
      const systems::State<double>& state) const {
    return state.get_abstract_state<trajectories::PiecewisePolynomial<double>>(
        command_traj_index_);
  }

  const systems::InputPort<double>* traj_input_port_;

  systems::AbstractStateIndex command_traj_index_;
};

/**
 * Base class for Cartesian move primitives.
 * Derived classes need to supply:
 * - ComputeDesiredToolVelocityInWorldFrame()
 * - UpdateLogic()
 * - UpdateMotionSummary()
 */
class MoveTool : public MotionPrimitive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MoveTool)

  MoveTool(const multibody::MultibodyPlant<double>* plant,
           const multibody::Frame<double>* tool_frame, double timestep);

  const multibody::Frame<double>& tool_frame() const { return tool_frame_; }

 protected:
  virtual Vector6<double> ComputeDesiredToolVelocityInWorldFrame(
      const systems::State<double>& state, double time) const = 0;

  void UpdateCommand(const systems::Context<double>& context,
                     systems::State<double>* state) const override final;

  math::RigidTransform<double> CalcRelativeTransform(
      const multibody::Frame<double>& frame_A,
      const multibody::Frame<double>& frame_B, const Eigen::VectorXd& q) const;

 private:
  // I am cheating here... Didn't want to make a new context every tick just to
  // do kinematics. I also didn't want to lump this into the state as abstract
  // value to avoid it being cloned every update. Any suggestions?
  mutable std::unique_ptr<systems::Context<double>> temp_context_;
  const multibody::Frame<double>& tool_frame_;
  planner::DifferentialInverseKinematicsParameters params_;

  const systems::InputPort<double>* tool_frame_vel_input_port_;
};

class MoveToolStraight final : public MoveTool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MoveToolStraight)

  MoveToolStraight(const multibody::MultibodyPlant<double>* plant,
                   const multibody::Frame<double>* tool_frame, double timestep);

  /*
   * Input needs to be an AbstractInput of type
   * SingleSegmentCartesianTrajectory<double>. I am not married to this, just
   * happens to be what we used in anzu up to now.
   */
  const systems::InputPort<double>& get_trajectory_input() const {
    return *trajectory_input_port_;
  }

 private:
  const SingleSegmentCartesianTrajectory<double>& get_command_traj(
      const systems::State<double>& state) const {
    return state.get_abstract_state<SingleSegmentCartesianTrajectory<double>>(
        command_traj_index_);
  }

  Vector6<double> ComputeDesiredToolVelocityInWorldFrame(
      const systems::State<double>& state, double time) const override;

  void UpdateLogic(const systems::Context<double>& context,
                   systems::State<double>* state) const override;

  void UpdateMotionSummary(const systems::Context<double>& context,
                           systems::State<double>* state) const override;

  const systems::InputPort<double>* trajectory_input_port_;
  systems::AbstractStateIndex command_traj_index_;
};

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake
