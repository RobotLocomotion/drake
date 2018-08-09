#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/multibody/cylinder_with_multicontact/make_cylinder_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/random_rotation.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

DEFINE_double(target_realtime_rate, 0.5,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(z0, 0.5,
              "The initial height, in z, of the ball, m.");

DEFINE_double(vx0, 1.0,
              "The initial x-velocity of the ball, m/s.");

DEFINE_double(wx0, 0.1,
              "The initial x-angular velocity of the ball, rad/s.");

DEFINE_double(friction_coefficient, 0.5,
              "The friction coefficient of both the sphere and the ground.");

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

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::ContactResults;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::multibody_plant::PointPairContactInfo;
using drake::multibody::MultibodyTree;
using drake::multibody::QuaternionFloatingMobilizer;
using drake::multibody::SpatialVelocity;
using drake::systems::AbstractValue;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::RungeKutta2Integrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;


int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeCylinderPlant(
      radius, mass, coulomb_friction, -g * Vector3d::UnitZ(), FLAGS_time_step,
      &scene_graph));
  const MultibodyTree<double>& model = plant.model();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set at height z0 with random orientation.
  std::mt19937 generator(41);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  model.SetDefaultContext(&plant_context);
  //Matrix3d R_WB = math::UniformlyRandomRotationMatrix(&generator).matrix();
  Isometry3d X_WB = Isometry3d::Identity();
  //X_WB.linear() = R_WB;
  X_WB.translation() = Vector3d(0.0, 0.0, FLAGS_z0);
  const auto& cylinder = model.GetBodyByName("Cylinder");
  model.SetFreeBodyPoseOrThrow(
      cylinder, X_WB, &plant_context);
  model.SetFreeBodySpatialVelocityOrThrow(
      cylinder,
      SpatialVelocity<double>(
          Vector3<double>(FLAGS_wx0, 0.0, 0.0),
          Vector3<double>(FLAGS_vx0, 0.0, 0.0)), &plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyTree,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::bouncing_ball::do_main();
}
