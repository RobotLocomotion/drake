/// @file
///
/// This file set up a simulation environment of an allegro hand and an object.
/// The system is designed for position control of the hand, with a PID
/// controller to control the output torque. The system communicate with the
/// external program through LCM system, with a publisher to publish the
/// current state of the hand, and a subscriber to read the posiiton commands
/// of the finger joints.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::JointActuator;
using multibody::JointActuatorIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");
DEFINE_bool(add_gravity, false,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(
    mbp_discrete_update_period, 1.0e-2,
    "The fixed-time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model.");
DEFINE_double(gear_ratio, 369.0,
              "The gear ratio of each actuator, dimensionless.");
DEFINE_double(
    rotor_inertia, 1.0e-6,
    "Estimated rotor inertia for the rotor of each actuator, in [kg⋅m²]. If "
    "zero the effect of reflected inertia is not modeled.");
DEFINE_double(
    pid_frequency, 10.0,
    "This frequency determines the time scale of the PID controller.");

// Modeling the Allegro hand with and without reflected inertia.
// The default command line parameters are set to model an Allegro hand that
// includes the effect of reflected inertia due to the high gear ratio of the
// actuators.
// In these notes we provide recommended values of the parameters for when
// reflected inertia is not modeled. Notice in particular that we need to reduce
// the time step significantly.
//
// When reflected inertia is not modeled we recommend:
//   rotor_inertia = 0.0.
//   mbp_discrete_update_period = 1.5e-4 sec.
//   pid_frequency = 30 Hz.

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(
      &builder, FLAGS_mbp_discrete_update_period);

  std::string hand_model_path;
  if (FLAGS_use_right_hand) {
    hand_model_path = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_right.sdf");
  } else {
    hand_model_path = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_left.sdf");
  }

  const std::string object_model_path = FindResourceOrThrow(
      "drake/examples/allegro_hand/joint_control/simple_mug.sdf");
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(hand_model_path);
  ModelInstanceIndex mug_model = parser.AddModelFromFile(object_model_path);

  // Weld the hand to the world frame
  const auto& joint_hand_root = plant.GetBodyByName("hand_root");
  plant.AddJoint<multibody::WeldJoint>("weld_hand", plant.world_body(),
                                       std::nullopt,
                                       joint_hand_root,
                                       std::nullopt,
                                       RigidTransformd::Identity());

  // Model gear ratio and rotor inertia at each finger. In order to model the
  // effect of reflected inertia, we need to have the gear ratio and rotor
  // inertia of the actuators. We know from the Allegro hand's specs that the
  // gear ratio is ρ = 369. More details in
  // http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_Overview
  // We do not know the exact motor used in the actuator.
  // However we only need an approximate estimate of the rotor inertia. What we
  // know from the Allegro hand's specs:
  // 1. High gear ratio of ρ = 369.
  // 2. maximum actuator torque: 0.7 Nm, i.e. 0.7/369 = 1.9 mN⋅m at the motor.
  //    That is, the stall torque of the motor is in the order of 1.9 mN⋅m.
  // 3. From drawings, we see the finger has a cross section of about 20 mm
  //    wide. Therefore the diameter of the motor should be Ø ≲ 20 mm.
  // 4. The hand's power requirement is 7.5 V and at 5 A (minimum). Therefore at
  //    a minimum it runs at 37 Watts. For 16 actuators, we estimate motors of
  //    about 2 Watts.
  // 5. Max. joint speed of 0.11 sec/60° or 91 RPM. Thus the motor speed is
  //    about 33500 RPM.
  //
  // We looked at brushed DC motors from Maxon at
  // https://www.maxongroup.com/maxon/view/product/
  // For motors in the range 14-16 mm in diameter, 2.5 W, we find that RPM and
  // stall torque are in the right order of magnitude.
  // We also find that most of them have a rotor inertia of about 1 g⋅cm².
  //
  // This information then lead us to the following actuator parameters:
  //   - Gear ratio ρ = 369.
  //   - Rotor inertia Iᵣ = 1×10⁻⁶ kg⋅m².
  //
  // This allow us to esimate the reflected inertia as:
  //   - Irefl = ρ²⋅Iᵣ = 0.14 kg⋅m².
  //
  // As a reference, the mass of a finger is 0.17 kg. Each finger has phalanges
  // of about 5 cm length. Therefore we estimate their rotational inertia as
  // that for a rod about its end (1/3⋅m⋅ℓ²) at about 4.7×10⁻⁵ kg⋅m². That's
  // 3000 times smaller than the reflected inertia!.
  // That is, the effective inertia of the joint is 0.14 kg⋅m².
  // We the estimate the time it'd take to move the joint a full revolution from
  // zero velocity when applying the maximum torque of 0.7 Nm.
  // We obtain t = sqrt(2θ/ẇ) = 1.6 secs, which seems consistent with
  // experience.

  // Rotor inertia in kg⋅m².
  DRAKE_DEMAND(FLAGS_rotor_inertia >= 0.0);
  DRAKE_DEMAND(FLAGS_gear_ratio >= 1.0);
  const double rotor_inertia = FLAGS_rotor_inertia;
  // Gear ratio, dimensionless.
  const double gear_ratio = FLAGS_gear_ratio;
  const double reflected_inertia = rotor_inertia * gear_ratio * gear_ratio;
  // We expect all fingers to be actuated.
  DRAKE_DEMAND(plant.num_actuators() == 16);
  // N.B. This change MUST be performed before Finalize() in order to take
  // effect.
  for (JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuators(); ++actuator_index) {
    JointActuator<double>& actuator =
        plant.get_mutable_joint_actuator(actuator_index);
    actuator.set_default_rotor_inertia(rotor_inertia);
    actuator.set_default_gear_ratio(gear_ratio);
  }

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Finished building the plant
  plant.Finalize();

  // Visualization
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, plant, scene_graph, lcm);

  // Estimate rotational inertia for an average finger of mass 0.17/3 kg (0.17
  // is the mass of one finger) and length 5 cm. We estimate it using the
  // rotational inertia for a rod about its end, i.e. I = 1/3⋅m⋅ℓ².
  const double mass_finger = 0.17 / 3.0;
  const double length_finger = 0.05;
  const double Ifinger = mass_finger / 3.0 * length_finger * length_finger;

  // Approximate effective finger inertia.
  const double Ieff = Ifinger + reflected_inertia;

  // PID controller for position control of the finger joints
  VectorX<double> kp, kd, ki;
  MatrixX<double> Sx, Sy;
  GetControlPortMapping(plant, &Sx, &Sy);
  SetPositionControlledGains(FLAGS_pid_frequency, Ieff, &kp, &ki, &kd);
  auto& hand_controller = *builder.AddSystem<
      systems::controllers::PidController>(Sx, Sy, kp, ki, kd);
  builder.Connect(plant.get_state_output_port(),
                  hand_controller.get_input_port_estimated_state());
  builder.Connect(hand_controller.get_output_port_control(),
                  plant.get_actuation_input_port());

  // Create an output port of the continuous state from the plant that only
  // output the status of the hand finger joints related DOFs, and put them in
  // the pre-defined order that is easy for understanding.
  const auto& hand_status_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sx);
  builder.Connect(plant.get_state_output_port(),
                  hand_status_converter.get_input_port());
  const auto& hand_output_torque_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sy);
  builder.Connect(hand_controller.get_output_port_control(),
                  hand_output_torque_converter.get_input_port());

  // Create the command subscriber and status publisher for the hand.
  const double kLcmPeriod = examples::allegro_hand::kHardwareStatusPeriod;
  auto& hand_command_sub = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_allegro_command>(
          "ALLEGRO_COMMAND", lcm));
  hand_command_sub.set_name("hand_command_subscriber");
  auto& hand_command_receiver =
      *builder.AddSystem<AllegroCommandReceiver>(kAllegroNumJoints, kLcmPeriod);
  hand_command_receiver.set_name("hand_command_receiver");
  auto& hand_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_allegro_status>(
          "ALLEGRO_STATUS", lcm, kLcmPeriod /* publish period */));
  hand_status_pub.set_name("hand_status_publisher");
  auto& status_sender =
      *builder.AddSystem<AllegroStatusSender>(kAllegroNumJoints);
  status_sender.set_name("status_sender");

  builder.Connect(hand_command_sub.get_output_port(),
                  hand_command_receiver.get_input_port(0));
  builder.Connect(hand_command_receiver.get_commanded_state_output_port(),
                  hand_controller.get_input_port_desired_state());
  builder.Connect(hand_status_converter.get_output_port(),
                  status_sender.get_state_input_port());
  builder.Connect(hand_command_receiver.get_output_port(0),
                  status_sender.get_command_input_port());
  builder.Connect(hand_output_torque_converter.get_output_port(),
                  status_sender.get_commanded_torque_input_port());
  builder.Connect(status_sender.get_output_port(0),
                  hand_status_pub.get_input_port());

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  // Set the position of object
  const multibody::Body<double>& hand = plant.GetBodyByName("hand_root");
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize the mug pose to be right in the middle between the fingers.
  const Eigen::Vector3d& p_WHand =
      plant.EvalBodyPoseInWorld(plant_context, hand).translation();
  RigidTransformd X_WM(
      RollPitchYawd(M_PI / 2, 0, 0),
      p_WHand + Eigen::Vector3d(0.095, 0.062, 0.095));
  plant.SetFreeBodyPose(&plant_context,
                        plant.GetUniqueFreeBaseBodyOrThrow(mug_model), X_WM);

  // set the initial command for the hand
  hand_command_receiver.set_initial_position(
      &diagram->GetMutableSubsystemContext(hand_command_receiver,
                                           diagram_context.get()),
      VectorX<double>::Zero(plant.num_actuators()));

  // Set up simulator.
  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::allegro_hand::DoMain();
  return 0;
}
