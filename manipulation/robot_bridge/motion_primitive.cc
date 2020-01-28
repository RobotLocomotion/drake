#include "drake/manipulation/robot_bridge/motion_primitive.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace manipulation {
namespace robot_bridge {

MotionPrimitive::MotionPrimitive(const multibody::MultibodyPlant<double>* plant,
                                 const double timestep)
    : plant_(*plant), timestep_(timestep) {
  state_input_port_ = &this->DeclareVectorInputPort(
      "state", systems::BasicVector<double>(plant_.num_multibody_states()));

  // These are being directly copied to output ports.
  x_command_index_ = this->DeclareDiscreteState(
      systems::BasicVector<double>(plant->num_multibody_states()));
  motion_summary_index_ = this->DeclareAbstractState(
      AbstractValue::Make<MotionSummary>(MotionSummary()));

  const systems::BasicVector<double> q0(plant_.num_positions());
  summary_output_port_ = &this->DeclareAbstractOutputPort(
      "motion_summary", MotionSummary(), &MotionPrimitive::CalcMotionSummary,
      {this->abstract_state_ticket(motion_summary_index_)});
  position_output_port_ = &this->DeclareVectorOutputPort(
      "position", q0, &MotionPrimitive::CalcPositionOutput,
      {this->discrete_state_ticket(x_command_index_)});
  torque_output_port_ = &this->DeclareVectorOutputPort(
      "torque", q0, &MotionPrimitive::CalcTorqueOutput,
      {this->discrete_state_ticket(x_command_index_)});

  this->DeclarePeriodicUnrestrictedUpdate(timestep, 0);
}

void MotionPrimitive::Initialize(const Eigen::VectorXd& x0,
                                 systems::Context<double>* context) const {
  auto* state = &context->get_mutable_state();
  get_mutable_x_command(state).set_value(x0);
}

void MotionPrimitive::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  UpdateLogic(context, state);
  UpdateCommand(context, state);
  UpdateMotionSummary(context, state);
}

void MotionPrimitive::CalcPositionOutput(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const auto& x_current = get_x_command(context.get_state()).get_value();
  output->set_value(x_current.head(plant().num_positions()));
  drake::log()->info("{}::CalcPositionOutput @ {}: {}", this->get_name(),
                     context.get_time(), output->get_value().transpose());
  if (output->get_value().array().isNaN().any()) {
    drake::log()->error("{}::CalcPositionOutput @ {}: {}", this->get_name(),
                        context.get_time(), output->get_value().transpose());
    throw std::runtime_error(
        "NaN, did you Initialize() after "
        "SetDefaultState() or SetRandomState()?");
  }
}

MoveJoint::MoveJoint(const multibody::MultibodyPlant<double>* plant,
                     const double timestep)
    : MotionPrimitive(plant, timestep) {
  this->set_name("MoveJoint");

  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(plant->num_positions());
  const trajectories::PiecewisePolynomial<double> traj0 =
      trajectories::PiecewisePolynomial<double>::ZeroOrderHold({0, 0.1},
                                                               {q0, q0});
  traj_input_port_ = &this->DeclareAbstractInputPort(
      "q_trajectory", Value<trajectories::PiecewisePolynomial<double>>(traj0));

  command_traj_index_ = this->DeclareAbstractState(
      AbstractValue::Make<trajectories::PiecewisePolynomial<double>>(traj0));
}

void MoveJoint::UpdateLogic(const systems::Context<double>& context,
                            systems::State<double>* state) const {
  const auto& input_traj =
      get_trajectory_input().Eval<trajectories::PiecewisePolynomial<double>>(
          context);
  auto& cmd_traj = state->get_mutable_abstract_state<
      trajectories::PiecewisePolynomial<double>>(command_traj_index_);
  bool init_new_traj = !input_traj.isApprox(cmd_traj, 1e-4);

  if (init_new_traj) {
    drake::log()->info("{} initializing at {}", this->get_name(),
                       context.get_time());
    // Check traj's start is close to current q.
    const Eigen::VectorXd q0 = input_traj.value(input_traj.start_time());
    const int num_q = plant().num_positions();
    const Eigen::VectorXd q =
        this->EvalVectorInput(context, get_state_input().get_index())
            ->get_value()
            .head(num_q);

    const Eigen::VectorXd q_diff = q - q0;
    const Eigen::VectorXd q_diff_thres =
        Eigen::VectorXd::Constant(num_q, 5. * M_PI / 180.);
    DRAKE_THROW_UNLESS((q_diff.array() < q_diff_thres.array()).all());
    DRAKE_THROW_UNLESS((q_diff.array() > -q_diff_thres.array()).all());

    cmd_traj = input_traj;
  }
}

void MoveJoint::UpdateCommand(const systems::Context<double>& context,
                              systems::State<double>* state) const {
  // interp
  const double interp_time = context.get_time();
  const auto& cmd_traj = get_command_traj(*state);
  const Eigen::VectorXd q_cmd = cmd_traj.value(interp_time);
  // TODO set v
  Eigen::VectorXd x_cmd = Eigen::VectorXd::Zero(plant().num_multibody_states());
  x_cmd.head(plant().num_positions()) = q_cmd;
  get_mutable_x_command(state).set_value(x_cmd);
}

void MoveJoint::UpdateMotionSummary(const systems::Context<double>& context,
                                    systems::State<double>* state) const {
  const double interp_time = context.get_time();
  const auto& cmd_traj = get_command_traj(*state);

  auto& summary = get_mutable_motion_summary(state);
  summary.motion_name = this->get_name();
  if (interp_time > cmd_traj.end_time()) {
    summary.status = MotionStatus::kDone;
  } else {
    summary.status = MotionStatus::kExecuting;
  }
}

MoveTool::MoveTool(const multibody::MultibodyPlant<double>* plant,
                   const multibody::Frame<double>* tool_frame,
                   const double timestep)
    : MotionPrimitive(plant, timestep),
      tool_frame_(*tool_frame),
      params_(plant->num_positions(), plant->num_velocities()) {
  params_.set_unconstrained_degrees_of_freedom_velocity_limit(0.6);
  params_.set_joint_position_limits(std::pair<Eigen::VectorXd, Eigen::VectorXd>(
      plant->GetPositionLowerLimits(), plant->GetPositionUpperLimits()));
  params_.set_joint_velocity_limits(std::pair<Eigen::VectorXd, Eigen::VectorXd>(
      plant->GetVelocityLowerLimits(), plant->GetVelocityUpperLimits()));
  // TODO
  params_.set_nominal_joint_position(
      Eigen::VectorXd::Zero(plant->num_positions()));
  params_.set_timestep(timestep);

  temp_context_ = plant->CreateDefaultContext();
  this->set_name("MoveTool");
}

void MoveTool::UpdateCommand(const systems::Context<double>& context,
                             systems::State<double>* state) const {
  auto& summary = get_motion_summary(*state);
  if (summary.status != MotionStatus::kExecuting) {
    return;
  }

  const Vector6<double> V_WT_desired =
      ComputeDesiredToolVelocityInWorldFrame(*state, context.get_time());

  const auto& x_current = get_x_command(*state).get_value();
  plant().SetPositionsAndVelocities(temp_context_.get(), x_current);

  Eigen::VectorXd q = x_current.head(plant().num_positions());
  Eigen::VectorXd v = x_current.tail(plant().num_velocities());

  Eigen::MatrixXd full_J_WT(6, plant().num_velocities());
  plant().CalcJacobianSpatialVelocity(
      *temp_context_, multibody::JacobianWrtVariable::kV, tool_frame_,
      Vector3<double>::Zero(), plant().world_frame(), plant().world_frame(),
      &full_J_WT);

  auto result = planner::DoDifferentialInverseKinematics(q, v, V_WT_desired,
                                                         full_J_WT, params_);

  // bool is_stuck = false;
  if (result.status ==
      planner::DifferentialInverseKinematicsStatus::kSolutionFound) {
    v = result.joint_velocities.value();
    // is_stuck = false;
  } else {
    v = Eigen::VectorXd::Zero(plant().num_velocities());
    // is_stuck = (result.status ==
    // planner::DifferentialInverseKinematicsStatus::kStuck);
    if (result.status ==
        planner::DifferentialInverseKinematicsStatus::kNoSolutionFound) {
      drake::log()->critical("{} can't solve its QP", this->get_name());
    }
  }
  q += v * timestep();
  Eigen::VectorXd x_next = x_current;
  x_next.head(plant().num_positions()) = q;
  x_next.tail(plant().num_velocities()) = v;
  get_mutable_x_command(state).set_value(x_next);
}

math::RigidTransform<double> MoveTool::CalcRelativeTransform(
    const multibody::Frame<double>& frame_A,
    const multibody::Frame<double>& frame_B, const Eigen::VectorXd& q) const {
  plant().SetPositions(temp_context_.get(), q);
  return plant().CalcRelativeTransform(*temp_context_, frame_A, frame_B);
}

MoveToolStraight::MoveToolStraight(
    const multibody::MultibodyPlant<double>* plant,
    const multibody::Frame<double>* tool_frame, const double timestep)
    : MoveTool(plant, tool_frame, timestep) {
  this->set_name("MoveToolStraight");

  const SingleSegmentCartesianTrajectory<double> dummy_traj(
      Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0, -2, -1);

  trajectory_input_port_ = &this->DeclareAbstractInputPort(
      "tool_trajectory",
      Value<SingleSegmentCartesianTrajectory<double>>(dummy_traj));
  command_traj_index_ = this->DeclareAbstractState(
      AbstractValue::Make<SingleSegmentCartesianTrajectory<double>>(
          dummy_traj));
}

Vector6<double> MoveToolStraight::ComputeDesiredToolVelocityInWorldFrame(
    const systems::State<double>& state, double time) const {
  const auto& traj = get_command_traj(state);
  Eigen::Isometry3d X_WT_cmd = traj.get_pose(time);
  const Eigen::VectorXd q =
      get_x_command(state).get_value().head(plant().num_positions());
  const math::RigidTransform<double> X_WT_current =
      CalcRelativeTransform(plant().world_frame(), tool_frame(), q);
  return planner::ComputePoseDiffInCommonFrame(X_WT_current.GetAsIsometry3(),
                                               X_WT_cmd) /
         timestep();
}

void MoveToolStraight::UpdateLogic(const systems::Context<double>& context,
                                   systems::State<double>* state) const {
  const auto& input_traj =
      get_trajectory_input().Eval<SingleSegmentCartesianTrajectory<double>>(
          context);
  auto& cmd_traj = state->get_mutable_abstract_state<
      SingleSegmentCartesianTrajectory<double>>(command_traj_index_);

  bool init_new_traj = !input_traj.is_approx(cmd_traj, 1e-4);

  if (init_new_traj) {
    drake::log()->info("{} initializing at {}", this->get_name(),
                       context.get_time());
    const int num_q = plant().num_positions();
    const Eigen::VectorXd q =
        this->EvalVectorInput(context, get_state_input().get_index())
            ->get_value()
            .head(num_q);

    // Check traj's start is close to current tool pose.
    math::RigidTransform<double> X_WT_now =
        CalcRelativeTransform(plant().world_frame(), tool_frame(), q);
    math::RigidTransform<double> X_WT_traj =
        math::RigidTransform<double>(input_traj.get_pose(
            input_traj.get_position_trajectory().get_start_time()));

    DRAKE_THROW_UNLESS(X_WT_now.IsNearlyEqualTo(X_WT_traj, 1e-3));

    cmd_traj = input_traj;
  }
}

void MoveToolStraight::UpdateMotionSummary(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  const double interp_time = context.get_time();
  const auto& cmd_traj = get_command_traj(*state);

  auto& summary = get_mutable_motion_summary(state);
  summary.motion_name = this->get_name();
  if (interp_time > cmd_traj.get_position_trajectory().get_end_time()) {
    summary.status = MotionStatus::kDone;
  } else {
    summary.status = MotionStatus::kExecuting;
  }
}

}  // namespace robot_bridge
}  // namespace manipulation
}  // namespace drake
