#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/sensors/rotary_encoders.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the Acrobot, started near the upright
// configuration, with an LQR controller designed to stabilize the unstable
// fixed point and a state estimator in the loop. Run drake-visualizer to
// see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Make the robot.
  systems::DiagramBuilder<double> builder;
  auto acrobot_w_encoder = builder.AddSystem<AcrobotWEncoder<double>>(true);
  acrobot_w_encoder->set_name("acrobot_w_encoder");

  // Get a pointer to the actual plant subsystem (will be used below).
  auto acrobot = acrobot_w_encoder->acrobot_plant();

  // Attach a DrakeVisualizer so we can animate the robot.
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(acrobot_w_encoder->get_output_port(1),
                  publisher->get_input_port(0));

  // Make a Kalman filter observer.
  auto observer_acrobot = std::make_unique<AcrobotWEncoder<double>>();
  auto observer_context = observer_acrobot->CreateDefaultContext();
  {  // Set context to upright fixed point.
    AcrobotStateVector<double>* x0 =
        observer_acrobot->get_mutable_acrobot_state(observer_context.get());
    x0->set_theta1(M_PI);
    x0->set_theta2(0.0);
    x0->set_theta1dot(0.0);
    x0->set_theta2dot(0.0);
    observer_context->FixInputPort(0, Vector1d::Constant(0.0));
  }
  // Make a linearization here for the exercise below.  Need to do it before I
  // std::move the pointers.
  auto linearized_acrobot =
      systems::Linearize(*observer_acrobot, *observer_context);

  Eigen::Matrix4d W = Eigen::Matrix4d::Identity();
  Eigen::Matrix2d V =
      0.1 * Eigen::Matrix2d::Identity();  // Position measurements are clean.
  auto observer =
      builder.AddSystem(systems::estimators::SteadyStateKalmanFilter(
          std::move(observer_acrobot), std::move(observer_context), W, V));
  observer->set_name("observer");
  builder.Connect(acrobot_w_encoder->get_output_port(0),
                  observer->get_input_port(0));

  {  // As a simple exercise, analyze the closed-loop estimation error dynamics.
    std::cout << "Is this system observable? "
              << systems::IsObservable(*linearized_acrobot) << std::endl;

    Eigen::Matrix4d error_sys =
        linearized_acrobot->A() - observer->L() * linearized_acrobot->C();
    std::cout << "L = " << observer->L() << std::endl;
    std::cout << "A - LC = " << std::endl << error_sys << std::endl;
    std::cout << "eig(A-LC) = " << error_sys.eigenvalues() << std::endl;
  }

  // Make the LQR Controller.
  auto controller = builder.AddSystem(BalancingLQRController(*acrobot));
  controller->set_name("controller");
  builder.Connect(observer->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot_w_encoder->get_input_port(0));
  builder.Connect(controller->get_output_port(), observer->get_input_port(1));

  // Log the true state and the estimated state.
  auto x_logger = LogOutput(acrobot_w_encoder->get_output_port(1), &builder);
  x_logger->set_name("x_logger");
  auto xhat_logger = LogOutput(observer->get_output_port(0), &builder);
  xhat_logger->set_name("xhat_logger");

  // Build the system/simulator.
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Set an initial condition near the upright fixed point.
  auto x0 = acrobot_w_encoder->get_mutable_acrobot_state(
      &(diagram->GetMutableSubsystemContext(*acrobot_w_encoder,
                                            &simulator.get_mutable_context())));
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.1);
  x0->set_theta2(-.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  // Set the initial conditions of the observer.
  auto& xhat0 = diagram
                    ->GetMutableSubsystemContext(
                        *observer, &simulator.get_mutable_context())
                    .get_mutable_continuous_state_vector();
  xhat0.SetAtIndex(0, M_PI);
  xhat0.SetAtIndex(1, 0.0);
  xhat0.SetAtIndex(2, 0.0);
  xhat0.SetAtIndex(3, 0.0);

  // Simulate.
  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.Initialize();
  simulator.StepTo(5);

  // Plot the results (launch call_matlab_client to see the plots).
  using common::CallMatlab;
  CallMatlab("figure", 1);
  CallMatlab("plot", x_logger->sample_times(),
             (x_logger->data().row(0).array() - M_PI).matrix(),
             x_logger->sample_times(), x_logger->data().row(1));
  CallMatlab("legend", "theta1 - PI", "theta2");
  CallMatlab("axis", "tight");

  CallMatlab("figure", 2);
  CallMatlab("plot", x_logger->sample_times(),
             (x_logger->data().array() - xhat_logger->data().array())
                 .matrix().transpose());
  CallMatlab("ylabel", "error");
  CallMatlab("legend", "theta1", "theta2", "theta1dot", "theta2dot");
  CallMatlab("axis", "tight");

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
