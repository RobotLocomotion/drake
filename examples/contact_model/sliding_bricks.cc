#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_bridge.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {

using drake::lcm::DrakeLcm;
using drake::multibody::joints::kFixed;
using Eigen::VectorXd;
using std::make_unique;

// Simple example of the "stiction" properties of the contact model.
// Based on the default values (50 kg brick) and the friction coefficients,
// a force of 260 N is insufficient to move the stationary block.  This is
// because the static friction is too great. However, if its initial velocity is
// 0.1 m/s, the force is sufficient to accelerate the moving box against the
// dynamic friction.
//
// After performing the initial simulation up to `sim_duration`, the example
// will promptly begin infinitely looping playback in wall clock time.

// Simulation parameters.
DEFINE_double(v, 0.1, "The initial speed of the second brick (m/s)");
DEFINE_double(timestep, 1e-4, "The simulator time step (s)");
DEFINE_double(push, 260,
              "The magnitude of the force pushing on the bricks (N)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.9, "The static coefficient of friction");
DEFINE_double(ud, 0.5, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 1.0, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 1e-3,
              "The characteristic scale of radius (m) of the contact area");
DEFINE_double(sim_duration, 3, "The simulation duration (s)");
DEFINE_string(system_type, "continuous", "The type of system to use: "
              "'continuous' or 'discretized'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'system_type=discretized' (ignored for "
              "'system_type=continuous'");

namespace {
const char* kSlidingBrickUrdf =
    "drake/examples/contact_model/sliding_brick.urdf";
}  // namespace

// Simple scenario of two blocks being pushed across a plane.  The first block
// has zero initial velocity.  The second has a small initial velocity in the
// pushing direction.
int main() {
  std::cout << "Parameters:\n";
  std::cout << "\tTime step:        " << FLAGS_timestep << "\n";
  std::cout << "\tPushing force:    " << FLAGS_push << "\n";
  std::cout << "\tẋ:                " << FLAGS_v << "\n";
  std::cout << "\tYoung's modulus:  " << FLAGS_youngs_modulus << "\n";
  std::cout << "\tStatic friction:  " << FLAGS_us << "\n";
  std::cout << "\tDynamic friction: " << FLAGS_ud << "\n";
  std::cout << "\tSlip Threshold:   " << FLAGS_v_tol << "\n";
  std::cout << "\tContact radius    " << FLAGS_contact_radius << "\n";
  std::cout << "\tDissipation:      " << FLAGS_dissipation << "\n";

  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(kSlidingBrickUrdf),
      kFixed, nullptr /* weld to frame */, tree_ptr.get());
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(kSlidingBrickUrdf),
      kFixed, nullptr /* weld to frame */, tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  if (FLAGS_system_type != "discretized")
    FLAGS_dt = 0.0;
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(
      move(tree_ptr), FLAGS_dt);
  plant.set_name("plant");

  // Contact parameters set arbitrarily.
  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant.set_default_compliant_material(default_material);
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant.set_contact_model_parameters(model_parameters);

  const auto& tree = plant.get_rigid_body_tree();

  // RigidBodyActuators.
  DRAKE_DEMAND(tree.actuators.size() == 2u);

  auto scene_graph = builder.AddSystem<geometry::SceneGraph<double>>();
  scene_graph->set_name("scene_graph");

  auto rbt_gs_bridge = builder.AddSystem<systems::RigidBodyPlantBridge<double>>(
      &tree, scene_graph);
  builder.Connect(plant.state_output_port(),
                  rbt_gs_bridge->rigid_body_plant_state_input_port());
  builder.Connect(
      rbt_gs_bridge->geometry_pose_output_port(),
      scene_graph->get_source_pose_port(rbt_gs_bridge->source_id()));

  // Pusher
  VectorXd push_value(1);      // Single actuator.
  push_value << FLAGS_push;
  ConstantVectorSource<double>& push_source =
      *builder.template AddSystem<ConstantVectorSource<double>>(push_value);
  push_source.set_name("push_source");

  // NOTE: This is *very* fragile.  It is not obvious that input port 0 is
  // the actuator on the first brick loaded and input port 1 is likewise the
  // actuator for the second brick.
  builder.Connect(push_source.get_output_port(), plant.get_input_port(0));
  builder.Connect(push_source.get_output_port(), plant.get_input_port(1));

  // Last thing before building the diagram; configure the system for
  // visualization.
  DrakeLcm lcm;
  geometry::ConnectVisualization(*scene_graph, &builder, &lcm);
  auto diagram = builder.Build();

  // Load message must be sent before creating a Context (Simulator
  // creates one).
  geometry::DispatchLoadMessage(*scene_graph, &lcm);

  // Create simulator.
  Simulator<double> simulator(*diagram);
  Context<double>& context = simulator.get_mutable_context();
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*diagram,
                                                            FLAGS_timestep,
                                                            &context);
  // Set initial state.
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &context);
  // 6 1-dof joints * 2 = 6 * (x, ẋ) * 2
  const int kStateSize = 24;
  VectorX<double> initial_state(kStateSize);
  initial_state << 0, -0.5, 0, 0, 0, 0,  // brick 1 position
                   0, 0.5, 0, 0, 0, 0,   // brick 2 position
                   0, 0, 0, 0, 0, 0,     // brick 1 velocity
                   FLAGS_v, 0, 0, 0, 0, 0;     // brick 2 velocity
  plant.set_state_vector(&plant_context, initial_state);

  simulator.StepTo(FLAGS_sim_duration);

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::main();
}
