#include <fstream>
#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/sine.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace conveyor_belt {
namespace {

// Simulates a block on an oscillating conveyor belt (in the belt's reference
// frame) by applying a sinusoidal horizontal force to the block.

DEFINE_double(mbp_time_step, 0.0, "Time step for plant (0 => continuous).");
DEFINE_double(integrator_time_step, 0.1, "Time step for the integrator.");
DEFINE_double(simulation_time, 4.0, "Simulation duration in seconds");
DEFINE_double(static_friction, 1.0, "Coefficient of static friction.");
DEFINE_double(dynamic_friction, 0.25, "Coefficient of dynamic friciton.");
DEFINE_double(frequency, 0.5, "Oscillation frequency (Hz).");
DEFINE_double(amplitude, 10.0, "Force amplitude (N).");
DEFINE_double(stiction_tolerance, 1e-4, "Stiction velocity (m/s).");
DEFINE_bool(use_hydro, false, "Whether to use hydroelastic contact.");
DEFINE_bool(use_error_control, true,
            "Whether to use error control in the integrator.");
DEFINE_double(accuracy, 1e-5, "Target accuracy for error control.");
DEFINE_bool(hessian_reuse, false,
            "Whether to reuse Hessian in the convex integrator.");

using Eigen::MatrixXd;
using Eigen::Vector3d;
using geometry::Box;
using geometry::Meshcat;
using geometry::SceneGraphConfig;
using math::RigidTransform;
using multibody::CoulombFriction;
using systems::Context;
using systems::DiagramBuilder;
using systems::MatrixGain;
using systems::Sine;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();
  DiagramBuilder<double> builder;

  CoulombFriction<double> friction(FLAGS_static_friction,
                                   FLAGS_dynamic_friction);

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_time_step;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  SceneGraphConfig sg_config;
  if (FLAGS_use_hydro) {
    sg_config.default_proximity_properties.compliance_type = "compliant";
    sg_config.default_proximity_properties.hydroelastic_modulus = 1e8;
  }
  sg_config.default_proximity_properties.hunt_crossley_dissipation = 500;
  scene_graph.set_config(sg_config);

  // Add flat ground with friction
  RigidTransform<double> X_WG(Vector3d(0, 0, -0.05));
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, Box(100, 100, 0.1),
                                  "ground", friction);

  // Add a block
  const double LBx = 0.2;  // block dimensions
  const double LBy = 0.2;
  const double LBz = 0.1;
  const SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::SolidBoxWithMass(1.0, LBx, LBy, LBz);
  const RigidBody<double>& block = plant.AddRigidBody("block", M_BBcm_B);
  const RigidTransform<double> X_BG;
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant.RegisterVisualGeometry(block, X_BG, geometry::Box(LBx, LBy, LBz),
                               "block_visual", lightBlue);
  plant.RegisterCollisionGeometry(block, X_BG, geometry::Box(LBx, LBy, LBz),
                                  "block_collision", friction);

  if (!FLAGS_use_hydro) {
    // Add sphere collision geometries on the corners
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const double radius = 0.01;
    int i = 0;
    for (double x_sign : {-1.0, 1.0}) {
      for (double y_sign : {-1.0, 1.0}) {
        const std::string name_spherei =
            "Sphere" + std::to_string(++i) + "_Geometry";
        const double x = x_sign * LBx / 2;
        const double y = y_sign * LBy / 2;
        const double z = -LBz / 2;
        const Vector3<double> p_BoSpherei_B(x, y, z);
        const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
        plant.RegisterCollisionGeometry(block, X_BSpherei,
                                        geometry::Sphere(radius), name_spherei,
                                        friction);
        plant.RegisterVisualGeometry(
            block, X_BSpherei, geometry::Sphere(radius), name_spherei, red);
      }
    }
  }

  plant.Finalize();

  // Add an external force supplier
  auto sine = builder.AddSystem<Sine>(FLAGS_amplitude,
                                      2 * M_PI * FLAGS_frequency, 0.0, 1);
  MatrixXd D = MatrixXd::Zero(6, 1);
  D(3, 0) = 1.0;
  auto multiplier = builder.AddSystem<MatrixGain>(D);

  builder.Connect(sine->get_output_port(0), multiplier->get_input_port());
  builder.Connect(multiplier->get_output_port(),
                  plant.get_applied_generalized_force_input_port());

  // Visualizer setup
  visualization::VisualizationConfig vis_config;
  vis_config.publish_period = std::numeric_limits<double>::infinity();
  vis_config.publish_contacts = true;
  visualization::ApplyVisualizationConfig(vis_config, &builder, nullptr, &plant,
                                          &scene_graph, meshcat);

  // Compile the diagram
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions
  const Vector3<double> p_WoBo_W(0.0, 0.0, LBz / 2.0);
  const math::RigidTransform<double> X_WB(p_WoBo_W);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, block, X_WB);

  // Set up the simulator with the convex integrator
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  if (FLAGS_mbp_time_step == 0.0) {
    systems::ConvexIntegrator<double>& ci =
        simulator.reset_integrator<systems::ConvexIntegrator<double>>();
    systems::ConvexIntegratorSolverParameters params;
    params.enable_hessian_reuse = FLAGS_hessian_reuse;
    ci.set_solver_parameters(params);
    ci.set_plant(&plant);
    ci.set_maximum_step_size(FLAGS_integrator_time_step);
    ci.set_fixed_step_mode(!FLAGS_use_error_control);
    ci.set_target_accuracy(FLAGS_accuracy);
  }

  // Set a monitor to save stats to a file
  std::ofstream ofile("conveyor_belt_data.csv");
  ofile << "time,vt,f_app\n";
  ofile.close();
  const auto& plant_ref = plant;
  simulator.set_monitor([&plant_ref, &sine](const Context<double>& context) {
    const double time = context.get_time();
    const Context<double>& plant_ctx = plant_ref.GetMyContextFromRoot(context);

    // We can just read the tangential velocity from the plant state
    const double vt = plant_ref.GetVelocities(plant_ctx)(3);

    // Get the applied force on the block. We'll use this to compute the net
    // contact force, since the contact_results_output_port and
    // generalized_acceleration_output_port of the plant aren't set correctly
    // for the convex integrator.
    const Context<double>& sine_context = sine->GetMyContextFromRoot(context);
    const double f_app =
        sine->get_output_port(0).Eval<systems::BasicVector<double>>(
            sine_context)[0];

    std::ofstream outfile("conveyor_belt_data.csv", std::ios::app);
    outfile << fmt::format("{},{},{}\n", time, vt, f_app);
    outfile.close();

    return systems::EventStatus::Succeeded();
  });

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(0.0);
  simulator.Initialize();

  // Wait for meshcat to load
  std::cout << "Press [ENTER] to continue ...\n";
  getchar();

  const double recording_frames_per_second = 32;
  meshcat->StartRecording(recording_frames_per_second);
  simulator.AdvanceTo(FLAGS_simulation_time);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  PrintSimulatorStatistics(simulator);

  // Wait for meshcat to load
  std::cout << "Press [ENTER] to continue ...\n";
  getchar();

  return 0;
}

}  // namespace
}  // namespace conveyor_belt
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::conveyor_belt::do_main();
}
