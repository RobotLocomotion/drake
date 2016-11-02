#include <gflags/gflags.h>

#include "drake/automotive/automotive_common.h"
#include "drake/automotive/car_simulation.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/matrix_gain.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_lcm_publisher.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

using std::make_unique;
using std::move;

namespace drake {

using parsers::sdf::AddModelInstancesFromSdfFile;
using lcm::DrakeLcm;
using systems::ConstantVectorSource;
using systems::Context;
using systems::DiagramBuilder;
using systems::MatrixGain;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::RigidBodyTreeLcmPublisher;
using systems::Simulator;

namespace automotive {
namespace {

int main() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  DiagramBuilder<double> builder;

  // Instantiates an Multibody Dynamics (MBD) model of the world.
  auto tree = make_unique<RigidBodyTree>();
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
      systems::plants::joints::kQuaternion, nullptr /* weld to frame */,
      tree.get());
  AddFlatTerrainToWorld(tree.get());

  // Instantiates a RigidBodyPlant to simulate the MBD model.
  auto plant = make_unique<RigidBodyPlant<double>>(move(tree));
  plant->set_contact_parameters(5000.0 /* penetration_stiffness */,
      500 /* penetration_damping */, 10 /* friction_coefficient */);

  // Instantiates a PID controller for controlling the actuators in the
  // RigidBodyPlant. The vector order is [steering, left wheel, right wheel].
  const Vector3<double> Kp(400,   0,   0);
  const Vector3<double> Ki(0,     0,   0);
  const Vector3<double> Kd(80,  100, 100);

  // TODO(liang.fok) Automatically initialize `feedback_selector_matrix` based
  // on the simulation model, actuators, etc.
  MatrixX<double> feedback_selector_matrix;
  feedback_selector_matrix.setZero(plant->get_input_size() * 2,
                                   plant->get_output_size());

  // The feedback selector should output six values:
  //
  //   Index | Description
  //   ----- | -----------
  //     0   |   steering angle position
  //     1   |   left wheel position
  //     2   |   right wheel position
  //     3   |   steering angle velocity
  //     4   |   left wheel velocity
  //     5   |   right wheel velocity
  DRAKE_DEMAND(feedback_selector_matrix.rows() == 6);
  const int kFeedbackIndexSteeringAnglePosition = 0;
  const int kFeedbackIndexLeftWheelPosition     = 1;
  const int kFeedbackIndexRightWheelPosition    = 2;
  const int kFeedbackIndexSteeringAngleVelocity = 3;
  const int kFeedbackIndexLeftWheelVelocity     = 4;
  const int kFeedbackIndexRightWheelVelocity    = 5;

  // The feedback selector should input 27 values:
  //
  //   Index   Description
  //   ----- | -----------
  //     0   |   base_x
  //     1   |   base_y
  //     2   |   base_z
  //     3   |   base_qw
  //     4   |   base_qx
  //     5   |   base_qy
  //     6   |   base_qz
  //     7   |   steering
  //     8   |   left_pin
  //     9   |   left_wheel_joint
  //     10  |   axle_tie_rod_arm
  //     11  |   right_wheel_joint
  //     12  |   rear_left_wheel_joint
  //     13  |   rear_right_wheel_joint
  //     14  |   base_wx
  //     15  |   base_wy
  //     16  |   base_wz
  //     17  |   base_vx
  //     18  |   base_vy
  //     19  |   base_vz
  //     20  |   steeringdot
  //     21  |   left_pindot
  //     22  |   left_wheel_jointdot
  //     23  |   axle_tie_rod_armdot
  //     24  |   right_wheel_jointdot
  //     25  |   rear_left_wheel_jointdot
  //     26  |   rear_right_wheel_jointdot
  DRAKE_DEMAND(feedback_selector_matrix.cols() == 27);
  const int kStateIndexSteeringAnglePosition = 7;
  const int kStateIndexLeftWheelPosition     = 9;
  const int kStateIndexRightWheelPosition    = 11;
  const int kStateIndexSteeringAngleVelocity = 20;
  const int kStateIndexLeftWheelVelocity     = 22;
  const int kStateIndexRightWheelVelocity    = 24;
  feedback_selector_matrix(kFeedbackIndexSteeringAnglePosition,
                           kStateIndexSteeringAnglePosition) = 1;
  feedback_selector_matrix(kFeedbackIndexLeftWheelPosition,
                           kStateIndexLeftWheelPosition) = 1;
  feedback_selector_matrix(kFeedbackIndexRightWheelPosition,
                           kStateIndexRightWheelPosition) = 1;
  feedback_selector_matrix(kFeedbackIndexSteeringAngleVelocity,
                           kStateIndexSteeringAngleVelocity) = 1;
  feedback_selector_matrix(kFeedbackIndexLeftWheelVelocity,
                           kStateIndexLeftWheelVelocity) = 1;
  feedback_selector_matrix(kFeedbackIndexRightWheelVelocity,
                           kStateIndexRightWheelVelocity) = 1;
  auto feedback_selector =
      std::make_unique<MatrixGain<double>>(feedback_selector_matrix);

  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(plant), std::move(feedback_selector), Kp, Ki, Kd);

  // Instantiates a system for visualizing the MBD model.
  lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<RigidBodyTreeLcmPublisher>(
      dynamic_cast<const RigidBodyPlant<double>*>(controller->plant())->
          get_rigid_body_tree(), &lcm);

  lcm.StartReceiveThread();

  // Instantiates a system for receiving user commands. The user command
  // consists of the following three-vector:
  //
  // [steering angle position, throttle velocity, brake velocity]
  //
  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder.template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, &lcm);

  // Instantiates a MIMO gain system to covert from user command space to
  // actuator command space. As mentioned above, the user command space consists
  // of the following three-vector:
  //
  // [steering angle position, throttle velocity, brake velocity]
  //
  // The actuator command space consists of a six-vector:
  //
  // [steering angle position, left wheel position, right wheel position,
  //  steering angle velocity, left wheel velocity, right wheel velocity]
  //
  // The MatrixGain system computes the following equation where `y` is the
  // actuator command, `D` is the gain`, and `u` is the user command:
  //
  //   y = Du
  //
  // The user's steering angle position command can be passed straight
  // through. The user's throttle and brake commands need to be multipled by
  // a gain of 20 and -20 to get the reference velocities for the left and right
  // wheels, respectively. Thus, the gain (`D`) should be:
  //
  // -------------------------------------------------------
  // Index |   kSteeringAngle   |   kThrottle   |   kBrake
  // -------------------------------------------------------
  //   0   |         1          |       0       |      0
  //   1   |         0          |       0       |      0
  //   2   |         0          |       0       |      0
  //   3   |         0          |       0       |      0
  //   4   |         0          |       20      |     -20
  //   5   |         0          |       20      |     -20
  // -------------------------------------------------------
  MatrixX<double> mimo_gain(controller->get_input_port(1).get_size(),
                            command_subscriber->get_output_port(0).get_size());
  mimo_gain <<
      1,  0,   0,
      0,  0,   0,
      0,  0,   0,
      0,  0,   0,
      0, 20, -20,
      0, 20, -20;
  auto user_to_actuator_cmd_sys =
      builder.template AddSystem<MatrixGain<double>>(mimo_gain);

  // Instantiates a constant vector source for the feedforward torque command.
  // The feedforward torque is zero.
  VectorX<double> constant_vector(controller->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);

  // Connects the feedforward torque command.
  builder.Connect(constant_zero_source->get_output_port(),
                  controller->get_input_port(0));

  // Connects the system that converts from user commands to actuator commands.
  builder.Connect(command_subscriber->get_output_port(0),
                  user_to_actuator_cmd_sys->get_input_port());

  // Connects the controller, which includes the plant being controlled.
  builder.Connect(user_to_actuator_cmd_sys->get_output_port(),
                  controller->get_input_port(1));

  // Connects the LCM publisher, which is used for visualization.
  builder.Connect(controller->get_output_port(0),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);

  // Zeroes the state and initializes controller state.
  systems::Context<double>* controller_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), controller);
  controller->SetDefaultState(controller_context);

  // Zeroes the rigid body plant's state. This is necessary because it is by
  // default initialized to a vector a NaN values.
  RigidBodyPlant<double>* rigid_body_plant =
      dynamic_cast<RigidBodyPlant<double>*>(controller->plant());
  Context<double>* plant_context =
      controller->GetMutableSubsystemContext(controller_context,
                                             rigid_body_plant);
  rigid_body_plant->SetZeroConfiguration(plant_context);

  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::main();
}
