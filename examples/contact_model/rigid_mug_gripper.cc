/*! @file
 A toy example of rubber grips holding a coffee mug.

 The rubber grips are two, symmetric toroidal pieces of rubber. The toroidal
 surface is approximated by uniform sampling its major radius with spheres.
 The number of samples (and many other features of the pads) are configurable
 on the command-line. The gripper is positioned to grab the mug around the
 cylinder's center, assuming the cylinder is in its "identity" pose.

 The mug is modeled as a cylinder with an affixed box approximating the handle.
 The box is *not* a collision element; it merely gives visual representation to
 the mug's asymmetrical inertia properties.

 Ideally, the simulation should show the mug being held in a static grip -- no
 slip.
*/

#include <algorithm>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/util/drakeGeometryUtil.h"

// Global default contact parameters
DEFINE_double(us, 0.9, "The coefficient of static friction");
DEFINE_double(ud, 0.5, "The coefficient of dynamic friction");
DEFINE_double(youngs_modulus, 1e8, "The contact material Young's modulus (Pa)");
DEFINE_double(dissipation, 2.0, "The contact material dissipation (s/m)");
DEFINE_double(v_stiction_tolerance, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(contact_radius, 1e-4,
              "The characteristic scale of radius (m) of the contact area");

// Simulation parameters: solver, integrator, playback
DEFINE_double(sim_duration, 5, "Amount of time to simulate (s)");
DEFINE_bool(playback, true,
            "If true, simulation begins looping playback when complete");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
    "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
    "'simulation_type=timestepping' (ignored for "
    "'simulation_type=compliant'");
DEFINE_double(accuracy, 5e-5, "Sets the simulation accuracy for "
    "'simulation_type=compliant'");

// Parameters for specifying the ring pad approximation.
DEFINE_int32(ring_samples, 4,
             "The number of spheres used to sample the pad ring");
DEFINE_double(pad_depth, 4e-3, "The depth the foremost pads penetrate the mug. "
    "Deeper penetration implies stronger contact forces");
DEFINE_double(ring_orient, 0, "Rotation of pads around x-axis (in degrees)");
DEFINE_double(ring_youngs_modulus, -1,
              "The Young's modulus for the ring pad. "
              "Negative values use the global default");
DEFINE_double(ring_dissipation, -1,
              "The dissipation for the ring pad. "
              "Negative values use the global default");
DEFINE_double(ring_static_friction, -1,
              "The coefficient of static friction for the ring pad. "
              "Negative values use the global default");
DEFINE_double(ring_dynamic_friction, -1,
              "The coefficient of dynamic friction for the ring pad."
              " Negative values use the global default");

// Parameters for posing the mug.
DEFINE_double(px, 0, "The x-position of the center, bottom of the mug");
DEFINE_double(py, 0, "The y-position of the center, bottom of the mug");
DEFINE_double(pz, 0, "The z-position of the center, bottom of the mug");
DEFINE_double(rx, 0,
              "The x-rotation of the mug around its origin - the center of its "
                  "bottom (in degrees). Rotation order: X, Y, Z");
DEFINE_double(ry, 0,
              "The y-rotation of the mug around its origin - the center of its "
                  "bottom (in degrees). Rotation order: X, Y, Z");
DEFINE_double(rz, 0,
              "The z-rotation of the mug around its origin - the center of its "
                  "bottom (in degrees). Rotation order: X, Y, Z");

namespace drake {
namespace examples {

using drake::SquareTwistMatrix;
using drake::systems::RungeKutta3Integrator;
using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;
using Eigen::Matrix3d;
using std::make_unique;

// These values should match the cylinder defined in:
// drake/examples/contact_model/cylinder_mug.urdf
const double kMugHeight = 0.1;
const double kMugRadius = 0.04;
// The pad was measured as a torus with the following major and minor radii.
const double kPadMajorRadius = 14e-3;  // 14 mm
const double kPadMinorRadius = 6e-3;  // 6 mm

// This uses the parameters and pad specifications to add collision pads to the
// gripper rigid body. Places *two* sets of pads symmetrically across the x-z
// plane. The two tori are centered on the y-axis. The distance between the
// origin and a pad's center is the mug's radius minus the target penetration
// depth.
void AddGripperPads(RigidBodyTree<double>* tree, RigidBody<double>* gripper) {
  const double penetration_depth = FLAGS_pad_depth;
  const int sample_count = FLAGS_ring_samples;
  const double sample_rotation = FLAGS_ring_orient * M_PI / 180.0;  // radians

  const double y = kMugRadius + kPadMinorRadius - penetration_depth;
  systems::CompliantMaterial material;
  if (FLAGS_ring_youngs_modulus >= 0)
    material.set_youngs_modulus(FLAGS_ring_youngs_modulus);
  if (FLAGS_ring_dissipation >= 0)
    material.set_dissipation(FLAGS_ring_dissipation);
  if (FLAGS_ring_static_friction >= 0 && FLAGS_ring_dynamic_friction >= 0) {
    material.set_friction(FLAGS_ring_static_friction,
                          FLAGS_ring_dynamic_friction);
  } else if (FLAGS_ring_static_friction >= 0 ||
      FLAGS_ring_dynamic_friction >= 0) {
    drake::log()->warn("Both static and dynamic friction should be specified. "
                           "Using global values instead.");
  }
  std::cout << "Ring contact material\n";
  std::cout << "  Youngs modulus:   " << material.youngs_modulus() << "\n";
  std::cout << "  Dissipation:      " << material.dissipation() << "\n";
  std::cout << "  Static friction:  " << material.static_friction() << "\n";
  std::cout << "  Dynamic friction: " << material.dynamic_friction() << "\n";

  // Sample the torus with a sphere. The sphere is located at X_GS, relative to
  // the gripper.
  auto add_ball = [&gripper, &tree, &material] (auto X_GS) {
    const DrakeShapes::Sphere ball(kPadMinorRadius);
    DrakeShapes::VisualElement vis(X_GS);
    vis.setGeometry(ball);
    gripper->AddVisualElement(vis);
    multibody::collision::Element collide(X_GS, gripper);
    collide.setGeometry(ball);
    collide.set_compliant_material(material);
    tree->addCollisionElement(collide, *gripper, "default");
  };

  const double d_theta = 2 * M_PI / sample_count;
  for (int i = 0; i < sample_count; ++i) {
    const double x = std::cos(d_theta * i + sample_rotation) * kPadMajorRadius;
    const double z = std::sin(d_theta * i + sample_rotation) * kPadMajorRadius;

    add_ball(Isometry3<double>{Translation3<double>{x, y, z}});
    add_ball(Isometry3<double>{Translation3<double>{x, -y, z}});
  }
}

std::unique_ptr<RigidBodyTreed> BuildTestTree() {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();

  // Add the gripper.  Move it up so it's aligned with the center of the mug's
  // barrel. NOTE: This urdf is an "empty" body. It has mass but no geometry
  // (collision or visual). We add them procedurally.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "gripper_pose_frame",
      &tree->world(), Eigen::Vector3d(0, 0, kMugHeight / 2),
      Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(
          "drake/examples/contact_model/rigid_mug_gripper.urdf"),
      multibody::joints::kFixed, gripper_frame, tree.get());

  // Add the procedural gripper pads.
  RigidBody<double>& gripper_body = *tree->FindBody("gripper_pads", "gripper");
  AddGripperPads(tree.get(), &gripper_body);

  // Add the "Mug" to grip. It assumes that with the identity pose, the center
  // of the bottom of the mug sits on the origin with the rest of the mug
  // oriented in the +z direction.
  std::cout << "Mug pose:\n";
  std::cout << "  Position:    " << FLAGS_px << ", " << FLAGS_py << ", "
            << FLAGS_pz << "\n";
  std::cout << "  Orientation: " << FLAGS_rx << ", " << FLAGS_ry << ", "
            << FLAGS_rz << "\n";
  auto mug_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "mug_pose",
      &tree->world(), Eigen::Vector3d(FLAGS_px, FLAGS_py, FLAGS_pz),
      Eigen::Vector3d(FLAGS_rx * M_PI / 180, FLAGS_ry * M_PI / 180,
                      FLAGS_rz * M_PI / 180));
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/contact_model/cylinder_mug.urdf"),
      multibody::joints::kQuaternion, mug_frame, tree.get());

  return tree;
}

int main() {
  systems::DiagramBuilder<double> builder;

  if (FLAGS_simulation_type != "timestepping")
    FLAGS_dt = 0.0;
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(BuildTestTree(),
                                                         FLAGS_dt);
  plant->set_name("plant");

  // Command-line specified contact parameters.
  std::cout << "Contact properties:\n";
  std::cout << "\tYoung's modulus:          " << FLAGS_youngs_modulus << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";
  std::cout << "\tstatic friction:          " << FLAGS_us << "\n";
  std::cout << "\tdynamic friction:         " << FLAGS_ud << "\n";
  std::cout << "\tAllowed stiction speed:   " << FLAGS_v_stiction_tolerance
            << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_stiction_tolerance;
  plant->set_contact_model_parameters(model_parameters);

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm, true);
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));

  // Enable contact force visualization.
  const ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  systems::Context<double>& context = simulator.get_mutable_context();

  simulator.reset_integrator<RungeKutta3Integrator<double>>(*model, &context);
  simulator.get_mutable_integrator()->request_initial_step_size_target(1e-4);
  simulator.get_mutable_integrator()->set_target_accuracy(FLAGS_accuracy);
  std::cout << "Variable-step integrator accuracy: " << FLAGS_accuracy << "\n";

  simulator.Initialize();

  // Print a time stamp update every tenth of a second.  This helps communicate
  // progress in the event that the integrator crawls to a very small timestep.
  const double kPrintPeriod = std::min(0.1, FLAGS_sim_duration);
  int step_count =
      static_cast<int>(std::ceil(FLAGS_sim_duration / kPrintPeriod));
  for (int i = 1; i <= step_count; ++i) {
    double t = context.get_time();
    std::cout << "time: " << t << "\n";
    simulator.StepTo(i * kPrintPeriod);
  }

  while (FLAGS_playback) viz_publisher->ReplayCachedSimulation();
  return 0;
}

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::main();
}
