#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(integration_accuracy, 1e-6, "Integration accuracy.");
DEFINE_double(simulation_time, 1.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(static_friction, 0.5, "Static friction coefficient.");
DEFINE_double(dynamic_friction, 0.3, "Dynamic friction coefficient.");
DEFINE_double(stiction_tolerance, 0.001, "Stribeck model stiction tolerance.");

using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using drake::multibody::benchmarks::inclined_plane::MakeInclinedPlanePlant;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = FLAGS_integration_accuracy;

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double slope = 15.0 / 180 * M_PI;  // rad.
  const CoulombFriction<double> surface_friction(FLAGS_static_friction,
                                                 FLAGS_dynamic_friction);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeInclinedPlanePlant(
      radius, mass, slope, surface_friction, g, &scene_graph));
  const MultibodyTree<double>& model = plant.model();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(0.001);
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  // Choose a maximum time step based on the fastest dynamics of the system. For
  // this case, the fastest dynamics corresponds to the time scale introduced by
  // the penalty method. Thus we set the maximum time step to a fraction of this
  // time scale.
  const double max_time_step =
      plant.get_contact_penalty_method_time_scale() / 30;

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // This will set a default initial condition with the sphere located at
  // p_WBcm = (0; 0; 0) and zero spatial velocity.
  model.SetDefaultContext(&plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator =
      simulator.get_mutable_integrator();
  integrator->set_maximum_step_size(max_time_step);
  integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A rolling sphere down an inclined plane demo using Drake's "
      "MultibodyPlant with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::bouncing_ball::do_main();
}
