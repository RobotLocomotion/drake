#include <chrono>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"
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
DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

// Contact model parameters.
DEFINE_string(contact_model, "HydroelasticWithFallback",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'HydroelasticWithFallback'.");
DEFINE_string(contact_tessellation, "triangle",
              "Tessellation of contact patches. Options are: 'triangle' and "
              "'polygon'. The 'polygon' is only available for a discrete "
              "system, see mbp_dt.");
DEFINE_double(elastic_modulus, 5.0e4,
              "For hydroelastic (and hybrid) contact, elastic modulus, [Pa].");
DEFINE_double(dissipation, 5.0,
              "For hydroelastic (and hybrid) contact, Hunt & Crossley "
              "dissipation, [s/m].");
DEFINE_double(friction_coefficient, 0.3, "friction coefficient.");
DEFINE_double(
    mbp_dt, 0.005,
    "The fixed time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model.");

DEFINE_bool(visualize, true,
            "If true, the simulation will publish messages for Drake "
            "visualizer. Useful to turn off during profiling sessions.");

// Ball's spatial velocity.
DEFINE_double(vx, 0,
              "Ball's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Ball's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, 0.0,
              "Ball's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wy, -10.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");

// Ball's pose.
DEFINE_double(roll, 0.0, "Ball's initial roll in degrees.");
DEFINE_double(pitch, 0.0, "Ball's initial pitch in degrees.");
DEFINE_double(yaw, 0.0, "Ball's initial yaw in degrees.");
DEFINE_double(z0, 0.1, "Ball's initial position in the z-axis.");
DEFINE_double(x0, 0.08, "Ball's initial position in the x-axis.");

DEFINE_double(resolution_hint_factor, 0.15, "resolution hint.");

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
  const double x0 = FLAGS_x0;
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBallPlatePlant(
      FLAGS_mbp_dt, radius, mass, FLAGS_elastic_modulus, FLAGS_dissipation,
      coulomb_friction, -g * Vector3d::UnitZ(),
      FLAGS_resolution_hint_factor, &scene_graph));

  if (FLAGS_contact_model == "hydroelastic" ||
      FLAGS_contact_model == "HydroelasticWithFallback") {
    if (FLAGS_contact_tessellation == "triangle") {
      plant.set_low_resolution_contact_surface(false);
    } else if (FLAGS_contact_tessellation == "polygon") {
      plant.set_low_resolution_contact_surface(true);
    } else {
      throw std::runtime_error("Invalid choice of contact tessellation '" +
                               FLAGS_contact_tessellation + "'.");
    }
  }

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelasticsOnly);
    plant.Finalize();
  } else if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
  } else if (FLAGS_contact_model == "HydroelasticWithFallback") {
    plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
    plant.Finalize();
    plant.set_penetration_allowance(0.001);
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  DRAKE_DEMAND(plant.num_velocities() == 12);
  DRAKE_DEMAND(plant.num_positions() == 14);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  if (FLAGS_visualize) {
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
    ConnectContactResultsToDrakeVisualizer(&builder, plant);
  }
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the ball's initial pose.
  math::RotationMatrixd R_WB(math::RollPitchYawd(
      M_PI / 180.0 * Vector3<double>(FLAGS_roll, FLAGS_pitch, FLAGS_yaw)));
  math::RigidTransformd X_WB(R_WB, Vector3d(x0, 0.0, z0));
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
      "An example of a ball rolling on a plate mesh using Drake's "
      "MultibodyPlant, with SceneGraph visualization. This example allows to "
      "switch between different contact models and integrators to evaluate "
      "performance. Launch drake-visualizer before running this example.");
  // We set the default realtime rate to 1.0, so it tries to run in real time.
  // For inspection, users can slow it down on command-line, e.g.
  // --simulator_target_realtime_rate=0.5.
  FLAGS_simulator_target_realtime_rate = 1.0;
  FLAGS_simulator_accuracy = 0.05;
  FLAGS_simulator_max_time_step = 0.1;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bouncing_ball::do_main();
}
