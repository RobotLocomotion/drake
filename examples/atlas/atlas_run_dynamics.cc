#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");
DEFINE_double(penetration_allowance, 1.0E-3, "Allowable penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-3,
              "Allowable drift speed during stiction (m/s).");

DEFINE_double(
    mbp_discrete_update_period, 0.01,
    "The fixed-time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model.");
DEFINE_string(contact_approximation, "sap",
              "Discrete contact approximation. Options are: 'sap', "
              "'similar', 'lagged'");

namespace drake {
namespace examples {
namespace atlas {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using Eigen::Translation3d;
using Eigen::VectorXd;

int do_main() {
  if (FLAGS_mbp_discrete_update_period < 0) {
    throw std::runtime_error(
        "mbp_discrete_update_period must be a non-negative number.");
  }

  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_discrete_update_period;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  plant_config.discrete_contact_approximation = FLAGS_contact_approximation;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  multibody::Parser(&builder).AddModelsFromUrl(
      "package://drake_models/atlas/atlas_convex_hull.urdf");

  // Add model of the ground.
  const double static_friction = 1.0;
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  // For a time-stepping model only static friction is used.
  const multibody::CoulombFriction<double> ground_friction(static_friction,
                                                           static_friction);
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  geometry::HalfSpace(),
                                  "GroundCollisionGeometry", ground_friction);

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  // Set the speed tolerance (m/s) for the underlying Stribeck friction model
  // For two points in contact, this is the maximum allowable drift speed at the
  // edge of the friction cone, an approximation to true stiction.
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  // Sanity check model size.
  DRAKE_DEMAND(plant.num_velocities() == 36);
  DRAKE_DEMAND(plant.num_positions() == 37);

  // Verify the "pelvis" body is a floating base body and modeled with
  // quaternions dofs before moving on with that assumption.
  const drake::multibody::RigidBody<double>& pelvis =
      plant.GetBodyByName("pelvis");
  DRAKE_DEMAND(pelvis.is_floating_base_body());
  DRAKE_DEMAND(pelvis.has_quaternion_dofs());
  // Since there is a single floating body, we know that the positions for it
  // lie first in the state vector.
  DRAKE_DEMAND(pelvis.floating_positions_start() == 0);
  // Similarly for velocities. The velocities for this floating pelvis are the
  // first set of velocities and should start at the beginning of v.
  DRAKE_DEMAND(pelvis.floating_velocities_start_in_v() == 0);

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
  plant.get_actuation_input_port().FixValue(&plant_context, tau);

  // Set the pelvis frame P initial pose.
  const Translation3d X_WP(0.0, 0.0, 0.95);
  plant.SetFloatingBaseBodyPoseInWorldFrame(&plant_context, pelvis, X_WP);

  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace atlas
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "\nPassive simulation of the Atlas robot. With the default time step of "
      "\n1 ms this simulation typically runs slightly faster than real-time. "
      "\nThe time step has an effect on the joint limits stiffnesses, which "
      "\nconverge quadratically to the rigid limit as the time step is "
      "\ndecreased. Thus, decrease the time step for more accurately resolved "
      "\njoint limits. "
      "\nLaunch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::atlas::do_main();
}
