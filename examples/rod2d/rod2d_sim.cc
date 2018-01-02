#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/rod2d/rod2d.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/drake_visualizer_client.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

using Rod2D = drake::examples::rod2d::Rod2D<double>;
using drake::lcm::DrakeLcm;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::Serializer;
using drake::systems::DiagramBuilder;
using drake::systems::rendering::MakeGeometryData;
using drake::systems::rendering::PoseAggregator;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::Simulator;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::RungeKutta3Integrator;

// Simulation parameters.
DEFINE_string(simulation_type, "timestepping",
              "Type of simulation, valid values are "
              "'timestepping','compliant'");
DEFINE_double(dt, 1e-2, "Integration step size");
DEFINE_double(rod_radius, 5e-2, "Radius of the rod (for visualization only)");
DEFINE_double(sim_duration, 10, "Simulation duration in virtual seconds");
DEFINE_double(accuracy, 1e-5,
              "Requested simulation accuracy (ignored for time stepping)");

int main(int argc, char* argv[]) {
  // Parse any flags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Emit a one-time load message.
  Serializer<drake::lcmt_viewer_load_robot> load_serializer;
  std::vector<uint8_t> message_bytes;

  // TODO(edrumwri): Remove the DRAKE_VIEWER_DRAW, DRAKE_VIEWER_LOAD_ROBOT
  //                 magic strings as soon as they are a named constant within
  //                 Drake (or, even better, remove as much of this
  //                 copy/pasted visualization code when possible).
  // Build the simulation diagram.
  DrakeLcm lcm;
  DiagramBuilder<double> builder;
  PoseAggregator<double>* aggregator =
      builder.template AddSystem<PoseAggregator>();
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(0.01);

  // Create the rod and add it to the diagram.
  Rod2D* rod;
  if (FLAGS_simulation_type == "timestepping") {
    rod = builder.template AddSystem<Rod2D>(
        Rod2D::SimulationType::kTimeStepping, FLAGS_dt);
  } else if (FLAGS_simulation_type == "compliant") {
    rod = builder.template AddSystem<Rod2D>(Rod2D::SimulationType::kCompliant,
                                            0.0);
  } else {
    std::cerr << "Invalid simulation type '" << FLAGS_simulation_type
              << "'; note that types are case sensitive." << std::endl;
    return -1;
  }

  // Create the rod visualization.
  DrakeShapes::VisualElement rod_vis(
      DrakeShapes::Cylinder(FLAGS_rod_radius, rod->get_rod_half_length() * 2),
      Eigen::Isometry3d::Identity(), Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  // Create the load message.
  drake::lcmt_viewer_load_robot message;
  message.num_links = 1;
  message.link.resize(1);
  message.link.back().name = "rod";
  message.link.back().robot_num = 0;
  message.link.back().num_geom = 1;
  message.link.back().geom.resize(1);
  message.link.back().geom[0] = MakeGeometryData(rod_vis);

  // Send a load mesage.
  const int message_length = message.getEncodedSize();
  message_bytes.resize(message_length);
  message.encode(message_bytes.data(), 0, message_length);
  lcm.Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(),
              message_bytes.size());

  // Set the names of the systems.
  rod->set_name("rod");
  aggregator->set_name("aggregator");
  converter->set_name("converter");

  builder.Connect(rod->pose_output(), aggregator->AddSingleInput("rod", 0));
  builder.Connect(*aggregator, *converter);
  builder.Connect(*converter, *publisher);
  auto diagram = builder.Build();

  // Make no external forces act on the rod.
  auto context = diagram->CreateDefaultContext();
  Context<double>& rod_context =
      diagram->GetMutableSubsystemContext(*rod, context.get());
  auto ext_input = std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 0.0);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  rod_context.FixInputPort(0, std::move(ext_input));

  // Set up the integrator.
  Simulator<double> simulator(*diagram, std::move(context));
  if (FLAGS_simulation_type == "compliant") {
    Context<double>& mut_context = simulator.get_mutable_context();
    simulator.reset_integrator<ImplicitEulerIntegrator<double>>(*diagram,
                                                                &mut_context);
  } else {
    Context<double>& mut_context = simulator.get_mutable_context();
    simulator.reset_integrator<RungeKutta3Integrator<double>>(*diagram,
                                                              &mut_context);
  }
  simulator.get_mutable_integrator()->set_target_accuracy(FLAGS_accuracy);
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);

  // Start simulating.
  simulator.set_target_realtime_rate(1.0);
  while (simulator.get_context().get_time() < FLAGS_sim_duration) {
    const double t = simulator.get_context().get_time();
    simulator.StepTo(std::min(t + 1, FLAGS_sim_duration));
  }
}
