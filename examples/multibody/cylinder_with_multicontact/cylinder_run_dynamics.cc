#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/cylinder_with_multicontact/make_cylinder_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cylinder_with_multicontact {
namespace {

DEFINE_double(target_realtime_rate, 0.5,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(z0, 0.5,
              "The initial height of the cylinder, m.");

DEFINE_double(vx0, 1.0,
              "The initial x-velocity of the cylinder, m/s.");

DEFINE_double(wx0, 0.1,
              "The initial x-angular velocity of the cylinder, rad/s.");

DEFINE_double(friction_coefficient, 0.3,
              "The friction coefficient of both the cylinder and the ground.");

DEFINE_double(penetration_allowance, 1.0e-3,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");

DEFINE_double(stiction_tolerance, 1.0e-4,
              "The maximum slipping speed allowed during stiction. [m/s]");

DEFINE_double(time_step, 1.0e-3,
              "If zero, the plant is modeled as a continuous system. "
              "If positive, the period (in seconds) of the discrete updates "
              "for the plant modeled as a discrete system."
              "This parameter must be non-negative.");

using Eigen::Vector3d;
using geometry::SceneGraph;
using lcm::DrakeLcm;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::CoulombFriction;
using drake::multibody::ConnectContactResultsToDrakeVisualizer;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialVelocity;

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Plant's parameters.
  const double radius = 0.05;          // The cylinder's radius, m
  const double mass = 0.1;             // The cylinder's mass, kg
  const double g = 9.81;               // Acceleration of gravity, m/s^2
  const double length = 4.0 * radius;  // The cylinder's length, m.
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeCylinderPlant(
      radius, length, mass, coulomb_friction, -g * Vector3d::UnitZ(),
      FLAGS_time_step, &scene_graph));
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(plant.get_source_id().has_value());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  DrakeLcm lcm;
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  // This is the source of poses for the visualizer.
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  // Publish contact results for visualization.
  ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph, &lcm);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set at height z0.
  math::RigidTransformd X_WB(Vector3d(0.0, 0.0, FLAGS_z0));
  const auto& cylinder = plant.GetBodyByName("Cylinder");
  plant.SetFreeBodyPose(&plant_context, cylinder, X_WB);
  plant.SetFreeBodySpatialVelocity(
      &plant_context, cylinder,
      SpatialVelocity<double>(
          Vector3<double>(FLAGS_wx0, 0.0, 0.0),
          Vector3<double>(FLAGS_vx0, 0.0, 0.0)));

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace cylinder_with_multicontact
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo for a cylinder falling towards the ground using Drake's"
      "MultibodyPlant, with SceneGraph contact handling and visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::cylinder_with_multicontact::do_main();
}
