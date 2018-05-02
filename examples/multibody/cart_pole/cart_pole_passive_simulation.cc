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

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double simulation_time = FLAGS_simulation_time;

  // Make the desired maximum time step a fraction of the simulation time.
  const double max_time_step = simulation_time / 1000.0;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

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
  cart_pole.RegisterVisualGeometry(
      pole, Isometry3<double>::Identity(),
      geometry::Cylinder(0.05, 0.5), &scene_graph);

  // Now the model is complete.
  cart_pole.Finalize();

#if 0
  // A constant source for a zero applied torque at the joints.
  Vector2<double> applied_torque = Vector2<double>::Zero();
  auto torque_source =
      builder.AddSystem<systems::ConstantVectorSource>(applied_torque);
  torque_source->set_name("Applied Torque");
  builder.Connect(torque_source->get_output_port(),
                  cart_pole.get_actuation_input_port());
#endif

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

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
        "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyTree,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::acrobot::do_main();
}
