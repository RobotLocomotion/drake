#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/proto/call_python.h"
#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/sensors/rotary_encoders.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the Acrobot, started near the upright
// configuration, with an LQR controller designed to stabilize the unstable
// fixed point and a state estimator in the loop. Run drake-visualizer to
// see the animated result.

DEFINE_double(simulation_sec, 5.0,
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main() {
  // Make the robot.
  systems::DiagramBuilder<double> builder;
  auto acrobot_w_encoder = builder.AddSystem<AcrobotWEncoder<double>>(true);
  acrobot_w_encoder->set_name("acrobot_w_encoder");

  // Get a pointer to the actual plant subsystem (will be used below).
  auto acrobot = acrobot_w_encoder->acrobot_plant();

  // Attach a DrakeVisualizer so we can animate the robot.
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  AcrobotGeometry::AddToBuilder(
      &builder, acrobot_w_encoder->get_output_port(1), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  // Make a Kalman filter observer.
  auto observer_acrobot = std::make_unique<AcrobotWEncoder<double>>();
  auto observer_context = observer_acrobot->CreateDefaultContext();
  {  // Set context to upright fixed point.
    AcrobotState<double>& x0 =
        observer_acrobot->get_mutable_acrobot_state(observer_context.get());
    x0.set_theta1(M_PI);
    x0.set_theta2(0.0);
    x0.set_theta1dot(0.0);
    x0.set_theta2dot(0.0);
    observer_acrobot->GetInputPort("elbow_torque")
        .FixValue(observer_context.get(), 0.0);
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
  auto x_logger = LogVectorOutput(acrobot_w_encoder->get_output_port(1),
                                  &builder);
  x_logger->set_name("x_logger");
  auto xhat_logger = LogVectorOutput(observer->get_output_port(0), &builder);
  xhat_logger->set_name("xhat_logger");

  // Build the system/simulator.
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Set an initial condition near the upright fixed point.
  AcrobotState<double>& x0 = acrobot_w_encoder->get_mutable_acrobot_state(
      &(diagram->GetMutableSubsystemContext(*acrobot_w_encoder,
                                            &simulator.get_mutable_context())));
  x0.set_theta1(M_PI + 0.1);
  x0.set_theta2(-.1);
  x0.set_theta1dot(0.0);
  x0.set_theta2dot(0.0);

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
  simulator.get_mutable_integrator().set_maximum_step_size(0.01);
  simulator.get_mutable_integrator().set_fixed_step_mode(true);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_sec);

  // Plot the results (launch call_python_client to see the plots).
  const auto& x_log = x_logger->FindLog(simulator.get_context());
  const auto& xhat_log = xhat_logger->FindLog(simulator.get_context());
  using common::CallPython;
  using common::ToPythonTuple;
  CallPython("figure", 1);
  CallPython("clf");
  CallPython("plot", x_log.sample_times(),
             (x_log.data().row(0).array() - M_PI)
                 .matrix().transpose());
  CallPython("plot", x_log.sample_times(), x_log.data().row(1).transpose());
  CallPython("legend", ToPythonTuple("theta1 - PI", "theta2"));
  CallPython("axis", "tight");

  CallPython("figure", 2);
  CallPython("clf");
  CallPython("plot", x_log.sample_times(),
             (x_log.data().array() - xhat_log.data().array())
                 .matrix().transpose());
  CallPython("ylabel", "error");
  CallPython("legend", ToPythonTuple("theta1", "theta2", "theta1dot",
                                     "theta2dot"));
  CallPython("axis", "tight");

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::do_main();
}
