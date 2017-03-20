#include "drake/examples/Valkyrie/valkyrie_pd_ff_controller.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/Valkyrie/robot_state_decoder.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/util/drakeUtil.h"

namespace drake {
using lcm::DrakeLcm;
using examples::valkyrie::kRPYValkyrieDof;

namespace systems {

using lcm::LcmSubscriberSystem;
using lcm::LcmPublisherSystem;

ValkyriePDAndFeedForwardController::ValkyriePDAndFeedForwardController(
    const RigidBodyTree<double>& robot,
    const VectorX<double>& nominal_position,
    const VectorX<double>& nominal_torque,
    const VectorX<double>& Kp, const VectorX<double>& Kd)
      : robot_(robot),
      desired_position_(nominal_position),
      feedforward_torque_(nominal_torque),
      Kp_(Kp),
      Kd_(Kd) {
  input_port_index_kinematics_result_ = DeclareAbstractInputPort().get_index();
  output_port_index_atlas_command_ = DeclareAbstractOutputPort().get_index();

  if (!Kp_.allFinite())
    throw std::runtime_error("Invalid Kp.");
  if (!Kd_.allFinite())
    throw std::runtime_error("Invalid Kd.");
  if (!desired_position_.allFinite())
    throw std::runtime_error("Invalid set point.");
  if (!feedforward_torque_.allFinite())
    throw std::runtime_error("Invalid feedforward torque.");

  set_name("pd_and_ff_controller_for_val");
}

void ValkyriePDAndFeedForwardController::DoCalcOutput(
                  const Context<double>& context,
                  SystemOutput<double>* output) const {
  // State input
  const KinematicsCache<double>* state =
    EvalInputValue<KinematicsCache<double>>(
        context, input_port_index_kinematics_result_);

  // Output
  bot_core::atlas_command_t& msg =
    output->GetMutableData(output_port_index_atlas_command_)
    ->GetMutableValue<bot_core::atlas_command_t>();

  // Make bot_core::atlas_command_t message.
  int act_size = robot_.actuators.size();
  msg.num_joints = act_size;
  msg.joint_names.resize(msg.num_joints);
  msg.position.resize(msg.num_joints);
  msg.velocity.resize(msg.num_joints);
  msg.effort.resize(msg.num_joints);

  int act_ctr = 0;

  for (const auto& actuator : robot_.actuators) {
    const auto& body = *actuator.body_;

    // Find q and v index for this actuator.
    int q_index = body.get_position_start_index();
    int v_index = body.get_velocity_start_index();

    double q_err = desired_position_(q_index) - state->getQ()(q_index);
    double v_err = 0 - state->getV()(v_index);

    msg.joint_names[act_ctr] = actuator.name_;
    msg.position[act_ctr] = static_cast<float>(desired_position_(q_index));
    msg.velocity[act_ctr] = 0.0;
    double Kp = Kp_(q_index);
    double Kd = Kd_(v_index);

    if (std::isnan(state->getQ()(q_index)) ||
        std::isnan(state->getV()(v_index))) {
      std::cerr << actuator.name_ << ", q: " << state->getQ()(q_index)
                << ", v: " << state->getV()(v_index) << std::endl;
      throw std::runtime_error("state error.");
    }

    msg.effort[act_ctr] =
        Kp * q_err + Kd * v_err + feedforward_torque_(v_index);
    act_ctr++;
  }

  // Set gains.
  VectorX<double> zeros = VectorX<double>::Zero(act_size);
  eigenVectorToStdVector(zeros, msg.k_q_p);
  eigenVectorToStdVector(zeros, msg.k_q_i);
  eigenVectorToStdVector(zeros, msg.k_qd_p);
  eigenVectorToStdVector(zeros, msg.k_f_p);
  eigenVectorToStdVector(zeros, msg.ff_qd);
  eigenVectorToStdVector(zeros, msg.ff_qd_d);
  eigenVectorToStdVector(zeros, msg.ff_f_d);
  eigenVectorToStdVector(zeros, msg.ff_const);

  // This is only used for the Virtual Robotics Challenge's gazebo simulator.
  // Should be deprecated by now.
  msg.k_effort.resize(msg.num_joints, 0);
  // TODO(siyuan.feng): I am not sure what this does exactly, most likely for
  // deprecated simulation as well.
  // Consider removing this from atlas_command_t.
  msg.desired_controller_period_ms = 0;
}



void run_valkyrie_pd_ff_controller() {
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, robot.get());

  VectorX<double> Kp(kRPYValkyrieDof);
  Kp << 0, 0, 0, 0, 0, 0,            // base
      100, 300, 300,                 // spine
      10,                            // neck
      10, 10, 10,                    // r shoulder
      1, 1, 0.1, 0.1,                // r arm
      10, 10, 10,                    // l shoulder
      1, 1, 0.1, 0.1,                // l arm
      100, 100, 300, 300, 300, 100,  // r leg
      100, 100, 300, 300, 300, 100;  // l leg

  VectorX<double> Kd(kRPYValkyrieDof);
  Kd << 0, 0, 0, 0, 0, 0,      // base
      10, 10, 10,              // spine
      3,                       // neck
      3, 3, 3,                 // r shoulder
      0.1, 0.1, 0.01, 0.01,    // r arm
      3, 3, 3,                 // l shoulder
      0.1, 0.1, 0.01, 0.01,    // l arm
      10, 10, 10, 10, 10, 10,  // r leg
      10, 10, 10, 10, 10, 10;  // l leg

  DrakeLcm lcm;
  DiagramBuilder<double> builder;
  RobotStateDecoder* state_decoder =
      builder.AddSystem(std::make_unique<RobotStateDecoder>(*robot));
  ValkyriePDAndFeedForwardController* controller =
      builder.AddSystem(std::make_unique<ValkyriePDAndFeedForwardController>(
          *robot, examples::valkyrie::RPYValkyrieFixedPointState().head(
                     kRPYValkyrieDof),
          examples::valkyrie::RPYValkyrieFixedPointTorque(), Kp, Kd));

  // lcm
  auto& robot_state_subscriber =
      *builder.AddSystem(LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
  auto& atlas_command_publisher =
      *builder.AddSystem(LcmPublisherSystem::Make<bot_core::atlas_command_t>(
          "ROBOT_COMMAND", &lcm));

  // lcm sub -> state decoder
  builder.Connect(robot_state_subscriber.get_output_port(0),
                  state_decoder->get_input_port(0));

  // state decoder -> controller
  builder.Connect(state_decoder->get_output_port(0),
                  controller->get_input_port_kinematics_result());

  // controller -> lcm pub
  builder.Connect(controller->get_output_port_atlas_command(),
                  atlas_command_publisher.get_input_port(0));

  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);

  lcm.StartReceiveThread();
  std::cout << "controller started\n";

  // Call controller.
  while (true) {
    const systems::Context<double>& pub_context =
        diagram->GetSubsystemContext(*context.get(), &atlas_command_publisher);
    atlas_command_publisher.Publish(pub_context);
  }
}

}  // namespace systems
}  // namespace drake

int main() { drake::systems::run_valkyrie_pd_ff_controller(); }
