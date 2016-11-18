#include <gflags/gflags.h>

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/matrix_gain.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/rigid_body_tree_construction.h"

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
using systems::DrakeVisualizer;
using systems::MatrixGain;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;

namespace automotive {
namespace {

// Verifies that the order of rigid body names and actuator names within the
// provided tree are as expected.
void VerifyCarSimLcmTree(const RigidBodyTreed& tree) {
  DRAKE_DEMAND(tree.get_num_bodies() == 18);

  std::map<std::string, int> name_to_idx =
      tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("base_x"));
  DRAKE_DEMAND(name_to_idx["base_x"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_y"));
  DRAKE_DEMAND(name_to_idx["base_y"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_z"));
  DRAKE_DEMAND(name_to_idx["base_z"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qw"));
  DRAKE_DEMAND(name_to_idx["base_qw"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qx"));
  DRAKE_DEMAND(name_to_idx["base_qx"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qy"));
  DRAKE_DEMAND(name_to_idx["base_qy"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qz"));
  DRAKE_DEMAND(name_to_idx["base_qz"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("steering"));
  DRAKE_DEMAND(name_to_idx["steering"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("left_pin"));
  DRAKE_DEMAND(name_to_idx["left_pin"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("left_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["left_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("axle_tie_rod_arm"));
  DRAKE_DEMAND(name_to_idx["axle_tie_rod_arm"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("right_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["right_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("rear_left_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["rear_left_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("rear_right_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["rear_right_wheel_joint"] == joint_idx++);

  DRAKE_DEMAND(tree.actuators.size() == 3);
  DRAKE_DEMAND(tree.actuators.at(0).name_ == "steering");
  DRAKE_DEMAND(tree.actuators.at(1).name_ == "left_wheel_joint");
  DRAKE_DEMAND(tree.actuators.at(2).name_ == "right_wheel_joint");
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  DiagramBuilder<double> builder;

  // Instantiates a model of the world.
  auto rigid_body_tree = make_unique<RigidBodyTreed>();
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
      multibody::joints::kQuaternion, nullptr /* weld to frame */,
      rigid_body_tree.get());
  multibody::AddFlatTerrainToWorld(rigid_body_tree.get());
  VerifyCarSimLcmTree(*rigid_body_tree);

  // Instantiates a RigidBodyPlant to simulate the model.
  auto plant = make_unique<RigidBodyPlant<double>>(move(rigid_body_tree));
  plant->set_contact_parameters(1000000.0 /* penetration_stiffness */,
      2000.0 /* penetration_damping */, 10.0 /* friction_coefficient */);

  // Instantiates a PID controller for controlling the actuators in the
  // RigidBodyPlant. The vector order is [steering, left wheel, right wheel].
  const Vector3<double> Kp(100,   0,   0);  // Units: Nm / radians
  const Vector3<double> Ki(0,     0,   0);  // Units: Nm / radians
  const Vector3<double> Kd(100,  250, 250);  // Units: Nm / (radians / sec).

  // TODO(liang.fok) Automatically initialize `feedback_selector_matrix` based
  // on the simulation model, actuators, etc.
  MatrixX<double> feedback_selector_matrix;
  feedback_selector_matrix.setZero(plant->get_input_size() * 2,
                                   plant->get_output_size());

  // The feedback selector should output six values:
  //
  //   Index | Description                 | Units
  //   ----- | --------------------------- | --------
  //     0   |   steering angle position   | radians
  //     1   |   left wheel position       | radians
  //     2   |   right wheel position      | radians
  //     3   |   steering angle speed      | radians / sec
  //     4   |   left wheel speed          | radians / sec
  //     5   |   right wheel speed         | radians / sec
  DRAKE_DEMAND(feedback_selector_matrix.rows() == 6);
  const int kFeedbackIndexSteeringAnglePosition = 0;
  const int kFeedbackIndexLeftWheelPosition     = 1;
  const int kFeedbackIndexRightWheelPosition    = 2;
  const int kFeedbackIndexSteeringAngleSpeed = 3;
  const int kFeedbackIndexLeftWheelSpeed     = 4;
  const int kFeedbackIndexRightWheelSpeed    = 5;

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
  const int kStateIndexSteeringAngleSpeed = 20;
  const int kStateIndexLeftWheelSpeed     = 22;
  const int kStateIndexRightWheelSpeed    = 24;
  feedback_selector_matrix(kFeedbackIndexSteeringAnglePosition,
                           kStateIndexSteeringAnglePosition) = 1;
  feedback_selector_matrix(kFeedbackIndexLeftWheelPosition,
                           kStateIndexLeftWheelPosition) = 1;
  feedback_selector_matrix(kFeedbackIndexRightWheelPosition,
                           kStateIndexRightWheelPosition) = 1;
  feedback_selector_matrix(kFeedbackIndexSteeringAngleSpeed,
                           kStateIndexSteeringAngleSpeed) = 1;
  feedback_selector_matrix(kFeedbackIndexLeftWheelSpeed,
                           kStateIndexLeftWheelSpeed) = 1;
  feedback_selector_matrix(kFeedbackIndexRightWheelSpeed,
                           kStateIndexRightWheelSpeed) = 1;
  auto feedback_selector =
      std::make_unique<MatrixGain<double>>(feedback_selector_matrix);

  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(plant), std::move(feedback_selector), Kp, Ki, Kd);

  // Instantiates a system for visualizing the model.
  lcm::DrakeLcm lcm;
  const RigidBodyTreed& tree =
      dynamic_cast<const RigidBodyPlant<double>*>(controller->plant())->
          get_rigid_body_tree();
  auto publisher = builder.AddSystem<DrakeVisualizer>(tree, &lcm);

  // Instantiates a system for receiving user commands. The user command
  // consists of the following three-vector:
  //
  // [steering angle position, throttle speed, brake speed]
  //
  // The throttle and brake speeds are with respect to the vehicle's
  // longitudinal position.
  //
  const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder.template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, &lcm);

  // Computes the gain necessary to convert from vehicle speed (m / s) to
  // wheel rotational speed (rad / sec). Let:
  //
  //  - v be the vehicle speed (m / sec)
  //  - w be the wheel rotational speed (rad / sec)
  //  - r be the wheel's radius in (m)
  //
  // Let c be the number of meters the vehicle travels longitudinally per wheel
  // rotation:
  //
  //    c = 2 * pi * r (m / wheel rev)
  //
  // Since there are 2 * pi (rad / wheel rev), the equation for w in terms of v
  // is:
  //
  //    w = v / c * (2 * pi)
  //
  // Thus:
  //
  //    w / v = (2 * pi) / c
  //          = (2 * pi) / (2 * pi * r)
  //          = 1 / r
  //
  // Note that the unit of w / v is (rad / sec) / (m / sec) = (rad / m).
  //
  // TODO(liang.fok): Obtain the following hard-coded radius from tree. It is
  // currently hard-coded to match the wheel radius specified in
  // drake-distro/drake/automotive/models/prius/prius_with_lidar.sdf.
  const double kWheelRadius = 0.323342;

  // Instantiates a MatrixGain system to covert from user command space to
  // actuator command space. As mentioned above, the user command space consists
  // of the following three-vector:
  //
  // [steering angle position, throttle speed, brake speed]
  //
  // The actuator command space consists of a six-vector:
  //
  // [steering angle position, left wheel position, right wheel position,
  //  steering angle speed, left wheel speed, right wheel speed]
  //
  // The MatrixGain system computes the following equation where `y` is the
  // actuator command, `D` is the gain`, and `u` is the user command:
  //
  //   y = Du
  //
  // The user's steering angle position command can be passed straight
  // through using a gain of 1. The user's throttle and brake commands need to
  // be multiplied by a gain of 1 / kWheelRadius and -1. / kWheelRadius to get
  // the reference rotational velocities for the left and right wheels,
  // respectively (see calculations above that relate vehicle longitudinal speed
  // with wheel rotational speed). Thus, the gain (`D`) should be:
  //
  // ---------------------------------------------------------------------
  // Index |   kSteeringAngle   |   kThrottle         |    kBrake
  // ---------------------------------------------------------------------
  //   0   |         1          |       0             |      0
  //   1   |         0          |       0             |      0
  //   2   |         0          |       0             |      0
  //   3   |         0          |       0             |      0
  //   4   |         0          |  1. / kWheelRadius  | -1. / kWheelRadius
  //   5   |         0          |  1. / kWheelRadius  | -1. / kWheelRadius
  // ---------------------------------------------------------------------
  //
  // TODO(liang.fok): Add a system that accounts for the difference in reference
  // wheel rotational velocities necessary in vehicles with Ackermann steering.
  // When such a vehicle turns, the kinematics of the vehicle require that the
  // wheels on the inner side of the turn rotate slower than the wheels on the
  // outer side of the turn.
  //
  MatrixX<double> matrix_gain(
      controller->get_input_port(1).get_size(),
      command_subscriber->get_output_port(0).get_size());
  matrix_gain <<
      1,                 0,                  0,
      0,                 0,                  0,
      0,                 0,                  0,
      0,                 0,                  0,
      0, 1. / kWheelRadius, -1. / kWheelRadius,
      0, 1. / kWheelRadius, -1. / kWheelRadius;
  DRAKE_ASSERT(matrix_gain.rows() == controller->get_input_port(1).get_size());
  DRAKE_ASSERT(matrix_gain.cols() ==
      command_subscriber->get_output_port(0).get_size());

  // TODO(liang.fok): Consider replacing the the MatrixGain system below with a
  // custom system that converts the user's commands to the vehicle's actuator's
  // commands. Such a system would eliminate the long explanation above about
  // how matrix_gain was derived and instead provide named getters and setters
  // with immediately-relevant units and scale comments.
  auto user_to_actuator_cmd_sys =
      builder.template AddSystem<MatrixGain<double>>(matrix_gain);

  // Instantiates a constant vector source for the feed-forward torque command.
  // The feed-forward torque is zero.
  VectorX<double> constant_vector(controller->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);

  // TODO(liang.fok): Modify controller to provide named accessors to these
  // ports.
  const int kControllerFeedforwardInputPort = 0;
  const int kControllerFeedbackInputPort = 1;

  // Connects the feed-forward torque command.
  builder.Connect(constant_zero_source->get_output_port(),
                  controller->get_input_port(kControllerFeedforwardInputPort));

  // Connects the system that converts from user commands to actuator commands.
  builder.Connect(command_subscriber->get_output_port(0),
                  user_to_actuator_cmd_sys->get_input_port());

  // Connects the controller, which includes the plant being controlled.
  builder.Connect(user_to_actuator_cmd_sys->get_output_port(),
                  controller->get_input_port(kControllerFeedbackInputPort));

  // Connects the LCM publisher, which is used for visualization.
  builder.Connect(controller->get_output_port(0),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();
  lcm.StartReceiveThread();

  Simulator<double> simulator(*diagram);

  // TODO(liangfok): Modify System 2.0 to not require the following
  // initialization.
  //
  // Zeros the state and initializes controller state.
  systems::Context<double>* controller_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), controller);
  controller->SetDefaultState(controller_context);

  // TODO(liang.fok): Modify System 2.0 to not require the following
  // initialization.
  //
  // Zeros the rigid body plant's state. This is necessary because it is by
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
  return drake::automotive::main(argc, argv);
}
