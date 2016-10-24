// #include <iostream>
// #include <memory>

// #include <gtest/gtest.h>

#include "drake/automotive/automotive_common.h"
#include "drake/automotive/car_simulation.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/common/drake_path.h"
// #include "drake/math/roll_pitch_yaw.h"
#include "drake/lcm/drake_lcm.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
// #include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
// #include "drake/systems/framework/primitives/adder.h"
// #include "drake/systems/framework/primitives/constant_vector_source.h"
// #include "drake/systems/framework/primitives/demultiplexer.h"
// #include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/mimo_gain.h"
// #include "drake/systems/framework/primitives/pid_controller.h"
// #include "drake/systems/framework/primitives/time_varying_polynomial_source.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_lcm_publisher.h"


// TODO(liang.fok) Temporary! Remove once actual input can be provided to
// RigidBodyPlant!
#include "drake/systems/framework/primitives/constant_vector_source.h"
// Includes for the planner.
// #include "drake/systems/plants/IKoptions.h"
// #include "drake/systems/plants/RigidBodyIK.h"

// using Eigen::Vector2d;
// using Eigen::Vector3d;
// using Eigen::VectorXd;
// using Eigen::VectorXi;
// using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFile;

using lcm::DrakeLcm;
// using systems::Adder;
using systems::ConstantVectorSource;
using systems::Context;
// using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
// using systems::Gain;
// using systems::GravityCompensator;
using systems::MimoGain;
// using systems::PidController;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::RigidBodyTreeLcmPublisher;
using systems::Simulator;
// using systems::TimeVaryingPolynomialSource;
using systems::plants::joints::kQuaternion;

namespace automotive {
namespace {

// // A demo of a single vehicle on a flat terrain.
// template<typename T>
// class CarSimLcm2Demo : public systems::Diagram<T> {
//  public:
//   CarSimLcm2Demo() {
//     this->set_name("CarSimLcmDemo");

//     // state_minus_target_ = builder.template AddSystem<Adder<T>>
//     //     (2 /*number of inputs*/, plant_->get_num_states() /* size */);

//     // // Create and add PID controller.
//     // // Constants are chosen by trial and error to qualitatively match an
//     // // experimental run with the same initial conditions and planner.
//     // // Quantitative comparisons would require torque control and a more careful
//     // // estimation of the model constants such as friction in the joints.
//     // const double kp = 2.0;  // proportional constant.
//     // const double ki = 0.0;  // integral constant.
//     // const double kd = 1.0;  // derivative constant.
//     // controller_ = builder.template AddSystem<PidController<T>>(
//     //     kp, ki, kd, plant_->get_num_positions());

//     // gravity_compensator_ = builder.template AddSystem<GravityCompensator<T>>(
//     //     plant_->get_rigid_body_tree());
//     // gcomp_minus_pid_ = builder.template AddSystem<Adder<T>>(
//     //     2 /*number of inputs*/, plant_->get_num_actuators() /* size */);

//     // // Split the input state into two signals one with the positions and one
//     // // with the velocities.
//     // // For Kuka:
//     // // -  get_num_states() = 14
//     // // -  get_num_positions() = 7
//     // error_demux_ = builder.template AddSystem<Demultiplexer<T>>(
//     //     plant_->get_num_states(), plant_->get_num_positions());
//     // rbp_state_demux_ = builder.template AddSystem<Demultiplexer<T>>(
//     //     plant_->get_num_states(), plant_->get_num_positions());

//     // controller_inverter_ = builder.template AddSystem<Gain<T>>(
//     //     -1.0, plant_->get_num_actuators());
//     // error_inverter_ = builder.template AddSystem<Gain<T>>(
//     //     -1.0, plant_->get_num_states());

//     // // Creates a plan and wraps it into a source system.
//     // poly_trajectory_ = MakePlan();
//     // desired_plan_ = builder.template AddSystem<TimeVaryingPolynomialSource<T>>(
//     //     *poly_trajectory_);

//     // Creates and adds LCM publisher for visualization.
//     viz_publisher_ = builder.template AddSystem<RigidBodyTreeLcmPublisher>(
//         plant_->get_rigid_body_tree(), &lcm_);

//     // // Generates an error signal for the PID controller by subtracting the
//     // // desired plan state from the RigidBodyPlant's (iiwa arm) state.
//     // builder.Connect(desired_plan_->get_output_port(0),
//     //                 error_inverter_->get_input_port());
//     // builder.Connect(error_inverter_->get_output_port(),
//     //                 state_minus_target_->get_input_port(0));
//     // builder.Connect(plant_->get_output_port(0),
//     //                 state_minus_target_->get_input_port(1));

//     // // Splits the error signal into positions and velocities components.
//     // builder.Connect(state_minus_target_->get_output_port(),
//     //                 error_demux_->get_input_port(0));

//     // // Splits the RBP output into positions (q) and velocities (v).
//     // builder.Connect(plant_->get_output_port(0),
//     //                 rbp_state_demux_->get_input_port(0));

//     // // Connects PID controller.
//     // builder.Connect(error_demux_->get_output_port(0),
//     //                 controller_->get_error_port());
//     // builder.Connect(error_demux_->get_output_port(1),
//     //                 controller_->get_error_derivative_port());

//     // // Connects the gravity compensator to the output generalized positions.
//     // builder.Connect(rbp_state_demux_->get_output_port(0),
//     //                 gravity_compensator_->get_input_port(0));
//     // builder.Connect(gravity_compensator_->get_output_port(0),
//     //                 gcomp_minus_pid_->get_input_port(0));

//     // // Adds feedback.
//     // builder.Connect(controller_->get_output_port(0),
//     //                 controller_inverter_->get_input_port());
//     // builder.Connect(controller_inverter_->get_output_port(),
//     //                 gcomp_minus_pid_->get_input_port(1));

//     // builder.Connect(gcomp_minus_pid_->get_output_port(),
//     //                 plant_->get_input_port(0));

//     // TODO(liang.fok) Remove this once actual inputs an be provided to
//     // RigidBodyPlant.
//     builder.Connect(const_source_->get_output_port(),
//                     plant_->get_input_port(0));

//     // Connects to publisher for visualization.
//     builder.Connect(plant_->get_output_port(0),
//                     viz_publisher_->get_input_port(0));

//     builder.ExportOutput(plant_->get_output_port(0));
//     builder.BuildInto(this);
//   }

//   // const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

//   void SetDefaultState(Context<T>* context) const {
//     // Context<T>* controller_context =
//     //     this->GetMutableSubsystemContext(context, controller_);
//     // controller_->set_integral_value(controller_context, VectorX<T>::Zero(7));

//     Context<T>* plant_context =
//         this->GetMutableSubsystemContext(context, plant_);
//     plant_->SetZeroConfiguration(plant_context);
//   }

//  private:
//   RigidBodyPlant<T>* plant_;
//   // PidController<T>* controller_;
//   // Demultiplexer<T>* error_demux_;
//   // Demultiplexer<T>* rbp_state_demux_;
//   // Gain<T>* controller_inverter_;
//   // Gain<T>* error_inverter_;
//   // GravityCompensator<T>* gravity_compensator_;
//   // Adder<T>* state_minus_target_;
//   // Adder<T>* gcomp_minus_pid_;
//   // TimeVaryingPolynomialSource<T>* desired_plan_;
//   // std::unique_ptr<PiecewisePolynomial<T>> poly_trajectory_;
//   RigidBodyTreeLcmPublisher* viz_publisher_;
//   DrakeLcm lcm_;

//   ConstantVectorSource<T>* const_source_;
// };

int main() {
  DiagramBuilder<double> builder;

  // Instantiates an Multibody Dynamics (MBD) model of the world.
  auto tree = make_unique<RigidBodyTree>();
  ModelInstanceIdTable vehicle_instance_id_table = AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
      kQuaternion, nullptr /* weld to frame */, tree.get());

  // DEBUG print statement.
  for (auto& actuator : tree->actuators) {
    std::cout << "Actuator: " << actuator.name_ << std::endl;
  }

  AddFlatTerrainToWorld(tree.get());

  // Instantiates a RigidBodyPlant to simulate the MBD model.
  auto plant = make_unique<RigidBodyPlant<double>>(move(tree));
  plant->set_contact_parameters(5000.0 /* penetration_stiffness */,
      500 /* penetration_damping */, 10 /* friction_coefficient */);

  // Instantiates a PID controller for controlling the actuators in the
  // RigidBodyPlant.
  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(plant), 0. /* Kp */, 0. /* Ki */, 0. /* Kd */);

  // DEBUG print statement.
  std::cout << "Size of controller's input port 0: "
            << controller->get_input_port(0).get_size() << std::endl;

  // DEBUG print statement.
  std::cout << "Size of controller's input port 1: "
            << controller->get_input_port(1).get_size() << std::endl;

  // Instantiates a system for visualizing the MBD model.
  lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<RigidBodyTreeLcmPublisher>(
      dynamic_cast<const RigidBodyPlant<double>*>(controller->system())->
          get_rigid_body_tree(), &lcm);

  // Instantiates a system for receiving user commands. The user command
  // consists of the following three-vector:
  //
  // [steering angle position, throttle velocity, brake velocity]
  //
  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder.template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, &lcm);

  // DEBUG print statement.
  std::cout << "Size of command subscriber's output port 0: "
            << command_subscriber->get_output_port(0).get_size() << std::endl;

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
  // The MimoGain system computes the following equation where `y` is the
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
  MatrixX<double> mimo_gain(controller->get_input_port(0).get_size(),
                            command_subscriber->get_output_port(0).get_size());
  mimo_gain.setZero();
  auto user_to_actuator_cmd_sys =
      builder.template AddSystem<MimoGain<double>>(mimo_gain);

  // DEBUG constant vector source.
  VectorX<double> constant_vector(controller->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);

  // VectorX<double> constant_value2(controller->get_input_port(1).get_size());
  // constant_value2.setZero();
  // auto constant_source2 =
  //     builder.template AddSystem<ConstantVectorSource<double>>(constant_value2);

  // The output of vector of the command subscriber is a DrivingCommand<double>,
  // which is a BasicVector<double> of length 3:
  //
  //     [steering angle, throttle, brake].
  //
  // These values need to be processed before they can be sent into the PID
  // controller, which has an input port of size 6. In
  // drake-distro/drake/automotive/car_simulation.cc, method
  // CreateVehicleSystem(), the values of the PID controller are multipled by
  // a matrix called `map_driving_cmd_to_x_d`, which serves as a translator
  // from the reference signal provided by the driving command to the reference
  // signal to provide to the PID controller.
  //
  // Here is the gain matrix that we want to have:
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
  //
  // The RigidBodyPlant has three actuators, each accepting a torque as input.
  // Here is the order:
  //
  //     0: steering
  //     1: left wheel
  //     2: right wheel
  //

  // Connects the feedforward control signal (all zeros).
  builder.Connect(constant_zero_source->get_output_port(),
                  controller->get_input_port(0));

  // Makes the connection that converts from user commands to actuator commands.
  builder.Connect(command_subscriber->get_output_port(0),
                  user_to_actuator_cmd_sys->get_input_port());

  // Makes the connection that converts from actuator commands to the actuator
  // controller.
  builder.Connect(user_to_actuator_cmd_sys->get_output_port(),
                  controller->get_input_port(1));

  // Makes the connection that takes the controller's output and publishes it
  // on an LCM channel for visualization.
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
      dynamic_cast<RigidBodyPlant<double>*>(controller->system());
  Context<double>* plant_context =
      controller->GetMutableSubsystemContext(controller_context,
                                             rigid_body_plant);
  rigid_body_plant->SetZeroConfiguration(plant_context);

  // VectorX<double> desired_state = VectorX<double>::Zero(14);
  // model.get_kuka_plant().set_state_vector(
  //     simulator.get_mutable_context(), desired_state);

  simulator.Initialize();

  // Simulate for 20 seconds.
  simulator.StepTo(20.0);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::automotive::main();
}
