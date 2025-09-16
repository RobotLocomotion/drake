#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace dynamic_friction_demo {
namespace {

// Simulates a block on an inclined plane that is slowly raised until the block
// slides off. Different choices of static and dynamic friction coefficients
// give different results.

DEFINE_double(mbp_time_step, 0.0, "Time step for plant (0 => continuous).");
DEFINE_double(simulation_time, 12.0, "Simulation duration in seconds");
DEFINE_double(static_friction, 0.6, "Coefficient of static friction.");
DEFINE_double(dynamic_friction, 0.2, "Coefficient of dynamic friciton.");
DEFINE_double(stiction_tolerance, 1e-4, "Stiction velocity (m/s).");
DEFINE_bool(use_hydro, false, "Whether to use hydroelastic contact.");
DEFINE_double(accuracy, 1e-3, "Convex integrator accuracy.");
DEFINE_double(integrator_time_step, 0.01, "Integrator time step.");
DEFINE_bool(use_error_control, true,
            "Whether to use error control in the integrator.");

using Eigen::Vector3d;
using geometry::SceneGraphConfig;
using math::RigidTransform;
using multibody::CoulombFriction;
using systems::ConstantVectorSource;
using systems::controllers::PidController;

int do_main() {
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  meshcat->SetCameraPose(Vector3d(-0.1, -0.4, 0.1), Vector3d(-0.2, 0.0, 0.1));

  systems::DiagramBuilder<double> builder;

  const CoulombFriction<double> friction(FLAGS_static_friction,
                                         FLAGS_dynamic_friction);

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_time_step;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  if (FLAGS_use_hydro) {
    SceneGraphConfig sg_config;
    sg_config.default_proximity_properties.compliance_type = "compliant";
    scene_graph.set_config(sg_config);
  }

  // Add an inlined plane with actuation
  const double LPx = 0.4064;  // 16 inches
  const double LPy = 0.3038;  // 12 inches
  const double LPz = 0.02;

  const Vector3d p0(-LPx / 2, 0.0, LPz / 2);
  const SpatialInertia<double> M_PPcm_P =
      SpatialInertia<double>::SolidBoxWithMass(1.0, LPx, LPy, LPz).Shift(-p0);
  const RigidBody<double>& plane = plant.AddRigidBody("plane", M_PPcm_P);
  const RigidTransform<double> X_PG(p0);
  const Vector4<double> planeColor(0.7, 0.5, 0.2, 1.0);
  plant.RegisterVisualGeometry(plane, X_PG, geometry::Box(LPx, LPy, LPz),
                               "plane_visual", planeColor);
  plant.RegisterCollisionGeometry(plane, X_PG, geometry::Box(LPx, LPy, LPz),
                                  "plane_collision", friction);

  // Add an actuated revolute joint on the plane
  const double damping = 100.0;
  const RevoluteJoint<double>& joint = plant.AddJoint<RevoluteJoint>(
      "plane_joint", plant.world_body(), RigidTransform<double>(), plane,
      std::optional<RigidTransform<double>>{}, Vector3d::UnitY(), damping);
  plant.AddJointActuator("actuator", joint);

  // Add a box on the plane
  const double LBx = 0.08;
  const double LBy = 0.08;
  const double LBz = 0.06;
  const SpatialInertia<double> M_BBcm_B =
      SpatialInertia<double>::SolidBoxWithMass(0.096, LBx, LBy, LBz);
  const RigidBody<double>& block = plant.AddRigidBody("block", M_BBcm_B);
  const RigidTransform<double> X_BG;
  const Vector4<double> boxColor(0.9, 0.7, 0.4, 1.0);
  plant.RegisterVisualGeometry(block, X_BG, geometry::Box(LBx, LBy, LBz),
                               "block_visual", boxColor);
  plant.RegisterCollisionGeometry(block, X_BG, geometry::Box(LBx, LBy, LBz),
                                  "block_collision", friction);

  // Add ground collision
  plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransform<double>(Vector3d(0, 0, -5)),
      geometry::Box(20, 20, 10.0), "ground_collision", friction);

  if (!FLAGS_use_hydro) {
    // Add sphere collision geometries on the corners
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const double radius = 0.001;
    int i = 0;
    for (double x_sign : {-1.0, 1.0}) {
      for (double y_sign : {-1.0, 1.0}) {
        for (double z_sign : {-1.0, 1.0}) {
          const std::string name_spherei =
              "Sphere" + std::to_string(++i) + "_Geometry";
          const double x = x_sign * LBx / 2;
          const double y = y_sign * LBy / 2;
          const double z = z_sign * LBz / 2;
          const Vector3<double> p_BoSpherei_B(x, y, z);
          const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
          plant.RegisterCollisionGeometry(block, X_BSpherei,
                                          geometry::Sphere(radius),
                                          name_spherei, friction);
          plant.RegisterVisualGeometry(
              block, X_BSpherei, geometry::Sphere(radius), name_spherei, red);
        }
      }
    }
  }

  plant.Finalize();

  // Add a controller to set the plane angle
  Vector1d kp(50.0);
  Vector1d ki(0.0);
  Vector1d kd(100.0);
  Eigen::MatrixXd Px = Eigen::MatrixXd::Zero(2, 15);
  Px(0, 0) = 1.0;
  Px(1, 8) = 1.0;
  auto ctrl = builder.AddSystem<PidController>(Px, kp, ki, kd);

  const Eigen::Vector2d x_des(0.7, 0.0);
  auto x_des_sender = builder.AddSystem<ConstantVectorSource>(x_des);

  builder.Connect(ctrl->get_output_port(), plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  ctrl->get_input_port_estimated_state());
  builder.Connect(x_des_sender->get_output_port(),
                  ctrl->get_input_port_desired_state());

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
  const Vector3<double> p_WoBo_W(-0.31, 0.0, LBz / 2 + LPz + 0.002);
  const math::RigidTransform<double> X_WB(p_WoBo_W);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, block, X_WB);

  // Set up the simulator with the convex integrator
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  if (FLAGS_mbp_time_step == 0.0) {
    systems::ConvexIntegrator<double>& ci =
        simulator.reset_integrator<systems::ConvexIntegrator<double>>();
    ci.set_plant(&plant);
    ci.set_maximum_step_size(FLAGS_integrator_time_step);
    ci.set_fixed_step_mode(!FLAGS_use_error_control);
    ci.set_target_accuracy(FLAGS_accuracy);
  }
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);
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
}  // namespace dynamic_friction_demo
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::dynamic_friction_demo::do_main();
}
