#include "drake/automotive/car_sim_lcm_common.h"

#include <utility>

#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::make_unique;
using std::move;

namespace drake {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::MatrixGain;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;

namespace automotive {

std::unique_ptr<systems::Diagram<double>> CreateCarSimLcmDiagram(
    const DrivingCommandTranslator& driving_command_translator,
    std::unique_ptr<RigidBodyTree<double>> tree, lcm::DrakeLcmInterface* lcm) {
  DiagramBuilder<double> builder;
  // Instantiates a RigidBodyPlant to simulate the model.
  auto plant = make_unique<RigidBodyPlant<double>>(move(tree));

  // Contact parameters
  const double kStiffness = 500000;
  const double kDissipation = 2;
  const double kStaticFriction = 10;
  const double kDynamicFriction = 5;
  const double kStictionSlipTolerance = 0.001;
  plant->set_normal_contact_parameters(kStiffness, kDissipation);
  plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                         kStictionSlipTolerance);

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

  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(plant), feedback_selector_matrix, Kp, Ki, Kd);
  controller->set_name("controller");

  // Instantiates a system for visualizing the model.
  const RigidBodyTreed& tree_ptr =
      dynamic_cast<const RigidBodyPlant<double>*>(controller->plant())->
          get_rigid_body_tree();
  auto publisher = builder.AddSystem<DrakeVisualizer>(tree_ptr, lcm);
  publisher->set_name("publisher");

  // Instantiates a system for receiving user commands, of type
  // DrivingCommand.
  auto command_subscriber =
      builder.template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, lcm);

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
  // actuator command space.  The user command space consists of:
  //
  // [steering angle position, acceleration]
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
  // through using a gain of 1. The user's acceleration command needs to
  // be multiplied by a gain of 1 / kWheelRadius to get
  // the reference rotational velocities for the left and right wheels,
  // respectively (see calculations above that relate vehicle longitudinal speed
  // with wheel rotational speed). Thus, the gain (`D`) should be:
  //
  // -------------------------------------------------
  // Index |   kSteeringAngle   |   kAcceleration
  // -------------------------------------------------
  //   0   |         1          |       0
  //   1   |         0          |       0
  //   2   |         0          |       0
  //   3   |         0          |       0
  //   4   |         0          |  1. / kWheelRadius
  //   5   |         0          |  1. / kWheelRadius
  // -------------------------------------------------
  //
  // TODO(liang.fok): Add a system that accounts for the difference in reference
  // wheel rotational velocities necessary in vehicles with Ackermann steering.
  // When such a vehicle turns, the kinematics of the vehicle require that the
  // wheels on the inner side of the turn rotate slower than the wheels on the
  // outer side of the turn.
  //
  MatrixX<double> matrix_gain(
      controller->get_input_port(1).size(),
      command_subscriber->get_output_port(0).size());
  matrix_gain <<
      1,                 0,
      0,                 0,
      0,                 0,
      0,                 0,
      0, 1. / kWheelRadius,
      0, 1. / kWheelRadius;
  DRAKE_ASSERT(matrix_gain.rows() == controller->get_input_port(1).size());
  DRAKE_ASSERT(matrix_gain.cols() ==
      command_subscriber->get_output_port(0).size());

  // TODO(liang.fok): Consider replacing the the MatrixGain system below with a
  // custom system that converts the user's commands to the vehicle's actuator's
  // commands. Such a system would eliminate the long explanation above about
  // how matrix_gain was derived and instead provide named getters and setters
  // with immediately-relevant units and scale comments.
  auto user_to_actuator_cmd_sys =
      builder.template AddSystem<MatrixGain<double>>(matrix_gain);
  user_to_actuator_cmd_sys->set_name("user_to_actuator_command");

  // Instantiates a constant vector source for the feed-forward torque command.
  // The feed-forward torque is zero.
  VectorX<double> constant_vector(controller->get_input_port(0).size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);
  constant_zero_source->set_name("zero");

  // Connects the feed-forward torque command.
  builder.Connect(constant_zero_source->get_output_port(),
                  controller->get_control_input_port());

  // Connects the system that converts from user commands to actuator commands.
  builder.Connect(command_subscriber->get_output_port(0),
                  user_to_actuator_cmd_sys->get_input_port());

  // Connects the controller, which includes the plant being controlled.
  builder.Connect(user_to_actuator_cmd_sys->get_output_port(),
                  controller->get_state_input_port());

  // Connects the LCM publisher, which is used for visualization.
  builder.Connect(controller->get_output_port(0),
                  publisher->get_input_port(0));

  return builder.Build();
}

}  // namespace automotive
}  // namespace drake
