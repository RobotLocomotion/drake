#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 0.4,
              "Desired duration of the simulation in seconds.");
DEFINE_string(contact_model, "hydroelastic",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'HydroelasticWithFallback'.");
DEFINE_string(hydro_rep, "poly",
              "Contact-surface representation for hydroelastics. "
              "Options are: 'tri' for triangles, 'poly' for polygons. "
              "Default is 'poly'.");
DEFINE_double(hydroelastic_modulus, 3.0e4,
              "Hydroelastic modulus of both the ball, [Pa].");
DEFINE_double(resolution_hint_factor, 0.3,
              "This scaling factor, [unitless], multiplied by the radius of "
              "the ball gives the target edge length of the mesh of the ball "
              "on the surface its hydroelastic representation. The smaller "
              "number gives a finer mesh with more tetrahedral elements.");
DEFINE_double(dissipation, 3.0,
              "Hunt & Crossley dissipation, [s/m], for the ball");
DEFINE_double(friction_coefficient, 0.3,
              "coefficient for both static and dynamic friction, [unitless], "
              "of the ball.");
DEFINE_double(mbp_dt, 0.001,
              "The fixed time step period (in seconds) of discrete updates "
              "for the multibody plant modeled as a discrete system. "
              "Strictly positive.");

// Ball's initial spatial velocity.
DEFINE_double(vx, 0,
              "Ball's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Ball's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, -7.0,
              "Ball's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wy, -10.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");

// Ball's initial pose.
DEFINE_double(roll, 0.0, "Ball's initial roll in degrees.");
DEFINE_double(pitch, 0.0, "Ball's initial pitch in degrees.");
DEFINE_double(yaw, 0.0, "Ball's initial yaw in degrees.");
DEFINE_double(z0, 0.15, "Ball's initial position in the z-axis.");
DEFINE_double(x0, 0.10, "Ball's initial position in the x-axis.");

namespace drake {
namespace examples {
namespace multibody {
namespace ball_plate {
namespace {

using Eigen::Vector3d;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialVelocity;

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2

  // We allow only discrete systems.
  DRAKE_DEMAND(FLAGS_mbp_dt > 0.0);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBallPlatePlant(
      FLAGS_mbp_dt, radius, mass, FLAGS_hydroelastic_modulus, FLAGS_dissipation,
      CoulombFriction<double>{
          FLAGS_friction_coefficient /* static friction */,
          FLAGS_friction_coefficient /* dynamic friction */},
      -g * Vector3d::UnitZ(), FLAGS_resolution_hint_factor, &scene_graph));

  if (FLAGS_hydro_rep == "tri") {
    plant.set_contact_surface_representation(
        geometry::HydroelasticContactRepresentation::kTriangle);
  } else if (FLAGS_hydro_rep == "poly") {
    plant.set_contact_surface_representation(
        geometry::HydroelasticContactRepresentation::kPolygon);
  } else {
    throw std::runtime_error(
        "Invalid contact-surface representation in hydroelastics: `" +
        FLAGS_hydro_rep + "'.");
  }

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelasticsOnly);
  } else if (FLAGS_contact_model == "HydroelasticWithFallback") {
    plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
  } else if (FLAGS_contact_model == "point") {
    plant.set_contact_model(ContactModel::kPoint);
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  plant.Finalize();

  if (FLAGS_contact_model == "point" ||
      FLAGS_contact_model == "HydroelasticWithFallback") {
    // Finalize `plant` before setting the penetration allowance (meters).
    plant.set_penetration_allowance(0.001);
  }

  DRAKE_DEMAND(plant.num_velocities() == 12);
  DRAKE_DEMAND(plant.num_positions() == 14);
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(plant.get_source_id().has_value());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  // The simulation is very fast and short. We want to see every time
  // step in the simulation.
  ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph,
                                         /* lcm */ nullptr, FLAGS_mbp_dt);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the ball's initial pose.
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"),
      math::RigidTransformd{
          math::RotationMatrixd{math::RollPitchYawd(
              M_PI / 180.0 *
              Vector3<double>(FLAGS_roll, FLAGS_pitch, FLAGS_yaw))},
          Vector3d(FLAGS_x0, 0.0, FLAGS_z0)});
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("Ball"),
      SpatialVelocity<double>{Vector3d(FLAGS_wx, FLAGS_wy, FLAGS_wz),
                              Vector3d(FLAGS_vx, FLAGS_vy, FLAGS_vz)});

  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(FLAGS_simulation_time);
  systems::PrintSimulatorStatistics(*simulator);
  return 0;
}

}  // namespace
}  // namespace ball_plate
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is an example for using hydroelastic contact model with a \n"
      "non-convex collision geometry loaded from an SDFormat file of a \n"
      "dinner plate. The ball, the plate, and the floor are compliant, \n"
      "rigid, and compliant hydroelastic. The plate-ball, ball-floor, \n"
      "and plate-floor contacts are rigid-compliant, compliant-compliant, \n"
      "and rigid-compliant. Hydroelastic contact model can work with \n"
      "non-convex shapes accurately without resorting to their convex \n"
      "hulls. Launch drake-visualizer before running this example.\n");
  FLAGS_simulator_accuracy = 0.05;
  FLAGS_simulator_max_time_step = 0.1;
  FLAGS_simulator_publish_every_time_step = true;
  FLAGS_simulator_target_realtime_rate = 0.1;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::ball_plate::do_main();
}
