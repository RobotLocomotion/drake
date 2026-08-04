#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 0.4,
              "Desired duration of the simulation in seconds.");
// See MultibodyPlantConfig for the valid strings of contact_model.
DEFINE_string(contact_model, "hydroelastic",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'hydroelastic_with_fallback'.");
// See MultibodyPlantConfig for the valid strings of contact surface
// representation.
DEFINE_string(contact_surface_representation, "polygon",
              "Contact-surface representation for hydroelastics. "
              "Options are: 'triangle' or 'polygon'. Default is 'polygon'.");
DEFINE_double(hydroelastic_modulus, 3.0e4,
              "Hydroelastic modulus of the ball, [Pa].");
DEFINE_double(resolution_hint_factor, 0.3,
              "This scaling factor, [unitless], multiplied by the radius of "
              "the ball gives the target edge length of the mesh of the ball "
              "on the surface of its hydroelastic representation. The smaller "
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
              "Ball's initial angular velocity in the x-axis in degrees/s.");
DEFINE_double(wy, -10.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Ball's initial angular velocity in the z-axis in degrees/s.");

// Ball's initial pose.
DEFINE_double(z0, 0.15, "Ball's initial position in the z-axis.");
DEFINE_double(x0, 0.10, "Ball's initial position in the x-axis.");

namespace drake {
namespace examples {
namespace ball_plate {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::SpatialVelocity;
using Eigen::Vector3d;

int do_main() {
  systems::DiagramBuilder<double> builder;

  multibody::MultibodyPlantConfig config;
  // We allow only discrete systems.
  DRAKE_DEMAND(FLAGS_mbp_dt > 0.0);
  config.time_step = FLAGS_mbp_dt;
  config.penetration_allowance = 0.001;
  config.contact_model = FLAGS_contact_model;
  config.contact_surface_representation = FLAGS_contact_surface_representation;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);

  // Ball's parameters.
  const double radius = 0.05;  // m
  const double mass = 0.1;     // kg
  AddBallPlateBodies(
      radius, mass, FLAGS_hydroelastic_modulus, FLAGS_dissipation,
      CoulombFriction<double>{// static friction (unused in discrete systems)
                              FLAGS_friction_coefficient,
                              // dynamic friction
                              FLAGS_friction_coefficient},
      FLAGS_resolution_hint_factor, &plant);

  plant.Finalize();

  DRAKE_DEMAND(plant.num_velocities() == 12);
  DRAKE_DEMAND(plant.num_positions() == 14);

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  auto simulator = MakeSimulatorFromGflags(*diagram);

  // Set the ball's initial pose.
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator->get_mutable_context());
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"),
      math::RigidTransformd{Vector3d(FLAGS_x0, 0.0, FLAGS_z0)});
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("Ball"),
      SpatialVelocity<double>{
          M_PI / 180.0 * Vector3d(FLAGS_wx, FLAGS_wy, FLAGS_wz),
          Vector3d(FLAGS_vx, FLAGS_vy, FLAGS_vz)});

  simulator->AdvanceTo(FLAGS_simulation_time);
  systems::PrintSimulatorStatistics(*simulator);
  return 0;
}

}  // namespace
}  // namespace ball_plate
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(R"""(
This is an example of using the hydroelastic contact model with a non-convex
collision geometry loaded from an SDFormat file of a dinner plate. The ball,
the plate, and the floor are compliant, rigid, and compliant hydroelastic
respectively. Hence, The plate-ball, ball-floor, and plate-floor contacts are
rigid-compliant, compliant-compliant, and rigid-compliant respectively. The
hydroelastic contact model can work with non-convex shapes accurately without
resorting to their convex hulls. Launch meldis before running this example.
See the README.md file for more information.)""");
  FLAGS_simulator_target_realtime_rate = 0.1;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::ball_plate::do_main();
}
