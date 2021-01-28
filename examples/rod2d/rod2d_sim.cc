#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod2d_geometry.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using Rod2D = drake::examples::rod2d::Rod2D<double>;
using drake::examples::rod2d::Rod2dGeometry;
using drake::geometry::DrakeVisualizerd;
using drake::geometry::SceneGraph;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::Simulator;

// Simulation parameters.
DEFINE_string(system_type, "discretized",
              "Type of rod system, valid values are "
              "'discretized','continuous'");
DEFINE_double(dt, 1e-2, "Integration step size");
DEFINE_double(rod_radius, 5e-2, "Radius of the rod (for visualization only)");
DEFINE_double(sim_duration, 10, "Simulation duration in virtual seconds");
DEFINE_double(accuracy, 1e-5,
              "Requested simulation accuracy (ignored for discretized system)");

int main(int argc, char* argv[]) {
  // Parse any flags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  // Create the rod and add it to the diagram.
  Rod2D* rod;
  if (FLAGS_system_type == "discretized") {
    rod = builder.template AddSystem<Rod2D>(
        Rod2D::SystemType::kDiscretized, FLAGS_dt);
  } else if (FLAGS_system_type == "continuous") {
    rod = builder.template AddSystem<Rod2D>(Rod2D::SystemType::kContinuous,
                                            0.0);
  } else {
    std::cerr << "Invalid simulation type '" << FLAGS_system_type
              << "'; note that types are case sensitive." << std::endl;
    return -1;
  }

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();
  Rod2dGeometry::AddToBuilder(FLAGS_rod_radius, rod->get_rod_half_length() * 2,
                              &builder, rod->state_output(), &scene_graph);
  DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  // Set the names of the systems.
  rod->set_name("rod");

  auto diagram = builder.Build();

  // Make no external forces act on the rod.
  auto context = diagram->CreateDefaultContext();
  Context<double>& rod_context =
      diagram->GetMutableSubsystemContext(*rod, context.get());
  const Eigen::Vector3d ext_input(0, 0, 0);
  rod->get_input_port(0).FixValue(&rod_context, ext_input);

  // Set up the integrator.
  Simulator<double> simulator(*diagram, std::move(context));
  if (FLAGS_system_type == "continuous") {
    simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  } else {
    simulator.reset_integrator<RungeKutta3Integrator<double>>();
  }
  simulator.get_mutable_integrator().set_target_accuracy(FLAGS_accuracy);
  simulator.get_mutable_integrator().set_maximum_step_size(FLAGS_dt);

  // Start simulating.
  simulator.set_target_realtime_rate(1.0);
  while (simulator.get_context().get_time() < FLAGS_sim_duration) {
    const double t = simulator.get_context().get_time();
    simulator.AdvanceTo(std::min(t + 1, FLAGS_sim_duration));
  }
}
