#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcm_call_matlab.h"
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

  // Get a pointer to the actual plant subsystem (will be used below).
  auto acrobot = acrobot_w_encoder->acrobot_plant();

  // Attach a DrakeVisualizer so we can animate the robot.
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
      multibody::joints::kFixed, tree.get());
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
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
  auto controller = builder.AddSystem(BalancingLQRController(acrobot));
  builder.Connect(observer->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot_w_encoder->get_input_port(0));
  builder.Connect(controller->get_output_port(), observer->get_input_port(1));

  // Log the true state and the estimated state.
  auto x_logger = builder.AddSystem<systems::SignalLogger<double>>(4);
  builder.Connect(acrobot_w_encoder->get_output_port(1),
                  x_logger->get_input_port(0));
  auto xhat_logger = builder.AddSystem<systems::SignalLogger<double>>(4);
  builder.Connect(observer->get_output_port(0), xhat_logger->get_input_port(0));

  // Build the system/simulator.
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Set an initial condition near the upright fixed point.
  auto x0 = dynamic_cast<AcrobotStateVector<double>*>(
      dynamic_cast<systems::DiagramContext<double>*>(
          diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                              acrobot_w_encoder))
          ->GetMutableSubsystemContext(0)
          ->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.1);
  x0->set_theta2(-.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  // Set the initial conditions of the observer.
  auto xhat0 = diagram
                   ->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                                observer)
                   ->get_mutable_continuous_state_vector();
  DRAKE_DEMAND(xhat0 != nullptr);
  xhat0->SetAtIndex(0, M_PI);
  xhat0->SetAtIndex(1, 0.0);
  xhat0->SetAtIndex(2, 0.0);
  xhat0->SetAtIndex(3, 0.0);

  // Simulate.
  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
  simulator.get_mutable_integrator()->set_minimum_step_size(0.01);
  simulator.Initialize();
  simulator.StepTo(5);

  // Plot the results (launch lcm_call_matlab_client to see the plots).
  using lcm::LcmCallMatlab;
  LcmCallMatlab("figure", 1);
  LcmCallMatlab("plot", x_logger->sample_times(),
                (x_logger->data().row(0).array() - M_PI).matrix(),
                x_logger->sample_times(), x_logger->data().row(1));
  LcmCallMatlab("legend", "theta1 - PI", "theta2");
  LcmCallMatlab("axis", "tight");

  LcmCallMatlab("figure", 2);
  LcmCallMatlab("plot", x_logger->sample_times(),
                (x_logger->data().array() - xhat_logger->data().array())
                    .matrix()
                    .transpose());
  LcmCallMatlab("ylabel", "error");
  LcmCallMatlab("legend", "theta1", "theta2", "theta1dot", "theta2dot");
  LcmCallMatlab("axis", "tight");

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
