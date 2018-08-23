#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using geometry::SceneGraph;
using lcm::DrakeLcm;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::Body;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::UniformGravityFieldElement;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the cart_pole model.
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");
  MultibodyPlant<double>& cart_pole =
      *builder.AddSystem<MultibodyPlant>(FLAGS_time_step);
  AddModelFromSdfFile(full_name, &cart_pole, &scene_graph);

  // Add gravity to the model.
  cart_pole.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  cart_pole.Finalize(&scene_graph);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!cart_pole.get_source_id());

  builder.Connect(
      cart_pole.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(cart_pole.get_source_id().value()));

  // Last thing before building the diagram; configure the system for
  // visualization.
  DrakeLcm lcm;
  geometry::ConnectVisualization(scene_graph, &builder, &lcm);
  auto diagram = builder.Build();

  // Load message must be sent before creating a Context.
  geometry::DispatchLoadMessage(scene_graph, &lcm);

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& cart_pole_context =
      diagram->GetMutableSubsystemContext(cart_pole, diagram_context.get());

  // There is no input actuation in this example for the passive dynamics.
  cart_pole_context.FixInputPort(
      cart_pole.get_actuation_input_port().get_index(), Vector1d(0));

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");

  // Set initial state.
  cart_slider.set_translation(&cart_pole_context, 0.0);
  pole_pin.set_angle(&cart_pole_context, 2.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple cart pole demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::cart_pole::do_main();
}
