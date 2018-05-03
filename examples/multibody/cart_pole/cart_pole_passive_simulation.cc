#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/math/transform.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using multibody::Body;
using multibody::multibody_plant::MultibodyPlant;
using multibody::UniformGravityFieldElement;
using multibody::parsing::AddModelFromSdfFile;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using systems::ImplicitEulerIntegrator;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the cart_pole model.
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");
  MultibodyPlant<double>& cart_pole = *builder.AddSystem<MultibodyPlant>();
  AddModelFromSdfFile(full_name, &cart_pole);

  // Add gravity to the model.
  cart_pole.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Get model links.
  const Body<double>& cart = cart_pole.GetBodyByName("Cart");
  const Body<double>& pole = cart_pole.GetBodyByName("Pole");

  // Register geometry for visualization.
  cart_pole.RegisterAsSourceForSceneGraph(&scene_graph);
  //const math::Transform<double> X_CV(
//      math::RotationMatrix<double>(
  //        AngleAxis<double>(M_PI_2, Vector3<double>::UnitY())),
    //  Vector3<double>::Zero());
  std::string box_mesh_path =
      FindResourceOrThrow("drake/examples/multibody/cart_pole/box_2x1.obj");
  cart_pole.RegisterVisualGeometry(
      cart, Isometry3<double>::Identity(),
      geometry::Mesh(box_mesh_path, 0.12), &scene_graph);
      //geometry::Cylinder(0.05, 0.3), &scene_graph);

  // Geometry for the pole's massless rod.
  cart_pole.RegisterVisualGeometry(
      pole, Isometry3d(Translation3d(0, 0, 0.25)),
      geometry::Cylinder(0.025, 0.5), &scene_graph);
  // Geometry for the pole's point mass at the end of the rod.
  cart_pole.RegisterVisualGeometry(
      pole, Isometry3d::Identity(),
      geometry::Sphere(0.05), &scene_graph);

  // Now the model is complete.
  cart_pole.Finalize();

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
  DRAKE_DEMAND(!!cart_pole.get_source_id());

  builder.Connect(
      cart_pole.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(cart_pole.get_source_id().value()));

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
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(cart_pole, diagram_context.get());

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");

  // Set initial state.
  cart_slider.set_translation(&acrobot_context, 0.0);
  pole_pin.set_angle(&acrobot_context, 1.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace acrobot
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
  return drake::examples::multibody::acrobot::do_main();
}
