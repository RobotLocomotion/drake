#include <chrono>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"

// Integration parameters.
DEFINE_double(simulation_time, 2.0,
              "Desired duration of the simulation in seconds.");

// Contact model parameters.
DEFINE_string(contact_model, "point",
              "Contact model. Options are: 'point', 'hydroelastic', 'hybrid'.");
DEFINE_double(hydroelastic_modulus, 5.0e4,
              "For hydroelastic (and hybrid) contact, "
              "hydroelastic modulus, [Pa].");
DEFINE_double(dissipation, 5.0,
              "For hydroelastic (and hybrid) contact, Hunt & Crossley "
              "dissipation, [s/m].");
DEFINE_double(friction_coefficient, 0.3, "friction coefficient.");
DEFINE_bool(rigid_ball, false,
            "If true, the ball is given a rigid hydroelastic representation "
            "(instead of being compliant by the default). Make sure "
            "you have the right contact model to support this representation.");
DEFINE_bool(soft_ground, false,
            "If true, the ground is given a soft hydroelastic representation "
            "(instead of the default rigid value). Make sure you have the "
            "right contact model to support this representation.");
DEFINE_bool(add_wall, false,
            "If true, adds a wall with compliant hydroelastic representation "
            "in the path of the default ball trajectory. This will cause the "
            "simulation to throw when the compliant ball hits the wall with "
            "the 'hydroelastic' model; use the 'hybrid' or 'point' contact"
            " model to simulate beyond this contact.");
DEFINE_double(
    mbp_dt, 0.0,
    "The fixed time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model.");

DEFINE_bool(visualize, true,
            "If true, the simulation will publish messages for Drake "
            "visualizer. Useful to turn off during profiling sessions.");
DEFINE_bool(vis_hydro, false,
            "If true, visualize collision geometries as their hydroelastic "
            "meshes, where possible.");

// Sphere's spatial velocity.
DEFINE_double(vx, 1.5,
              "Sphere's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Sphere's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, 0.0,
              "Sphere's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wy, -360.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");

// Sphere's pose.
DEFINE_double(roll, 0.0, "Sphere's initial roll in degrees.");
DEFINE_double(pitch, 0.0, "Sphere's initial pitch in degrees.");
DEFINE_double(yaw, 0.0, "Sphere's initial yaw in degrees.");
DEFINE_double(z0, 0.05, "Sphere's initial position in the z-axis.");

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::lcm::DrakeLcm;
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
  const double z0 = FLAGS_z0;        // Initial height.
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBouncingBallPlant(
      FLAGS_mbp_dt, radius, mass, FLAGS_hydroelastic_modulus, FLAGS_dissipation,
      coulomb_friction, -g * Vector3d::UnitZ(), FLAGS_rigid_ball,
      FLAGS_soft_ground, &scene_graph));

  if (FLAGS_add_wall) {
    geometry::Box wall{0.2, 4, 0.4};
    const RigidTransformd X_WB(Vector3d{-0.5, 0, 0});
    geometry::ProximityProperties prox_prop;
    geometry::AddContactMaterial({} /* dissipation */, {} /* point stiffness */,
                                 CoulombFriction<double>(),
                                 &prox_prop);
    geometry::AddSoftHydroelasticProperties(0.1, 1e8, &prox_prop);
    plant.RegisterCollisionGeometry(plant.world_body(), X_WB, wall,
                                    "wall_collision", std::move(prox_prop));

    geometry::IllustrationProperties illus_prop;
    illus_prop.AddProperty("phong", "diffuse", Vector4d(0.7, 0.5, 0.4, 0.5));
    plant.RegisterVisualGeometry(plant.world_body(), X_WB, wall, "wall_visual",
                                 std::move(illus_prop));
  }

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelastic);
    plant.Finalize();
  } else if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
  } else if (FLAGS_contact_model == "hybrid") {
    plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
    plant.Finalize();
    plant.set_penetration_allowance(0.001);
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(plant.get_source_id().has_value());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  if (FLAGS_visualize) {
    geometry::DrakeVisualizerParams params;
    if (FLAGS_vis_hydro) {
      params.role = geometry::Role::kProximity;
      params.show_hydroelastic = true;
    }
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                             params);
    ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph);
  }
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the sphere's initial pose.
  math::RotationMatrixd R_WB(math::RollPitchYawd(
      M_PI / 180.0 * Vector3<double>(FLAGS_roll, FLAGS_pitch, FLAGS_yaw)));
  math::RigidTransformd X_WB(R_WB, Vector3d(0.0, 0.0, z0));
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"), X_WB);

  const SpatialVelocity<double> V_WB(Vector3d(FLAGS_wx, FLAGS_wy, FLAGS_wz),
                                     Vector3d(FLAGS_vx, FLAGS_vy, FLAGS_vz));
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("Ball"), V_WB);

  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  using clock = std::chrono::steady_clock;
  const clock::time_point start = clock::now();
  simulator->AdvanceTo(FLAGS_simulation_time);
  const clock::time_point end = clock::now();
  const double wall_clock_time =
      std::chrono::duration<double>(end - start).count();
  fmt::print("Simulator::AdvanceTo() wall clock time: {:.4g} seconds.\n",
             wall_clock_time);

  systems::PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A rolling sphere demo using Drake's MultibodyPlant, "
      "with SceneGraph visualization. This demo allows to switch between "
      "different contact models and integrators to evaluate performance."
      "Launch drake-visualizer before running this example.");
  // We slow down the default realtime rate to 0.2, so that we can appreciate
  // the motion. Users can still change it on command-line, e.g.
  // --simulator_target_realtime_rate=0.5.
  FLAGS_simulator_target_realtime_rate = 0.2;
  // Simulator default parameters for this demo.
  FLAGS_simulator_accuracy = 1.0e-3;
  FLAGS_simulator_max_time_step = 1.0e-3;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bouncing_ball::do_main();
}
