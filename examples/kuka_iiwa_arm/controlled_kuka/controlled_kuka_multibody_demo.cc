/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a MultibodyPlant simulation to follow an arbitrarily designed
/// plan. The generated plan takes the arm from the zero configuration to reach
/// to a position in space and then repeat this reaching task with a different
/// joint configuration constraint.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/controlled_kuka_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using trajectories::PiecewisePolynomial;

const char kSdfPath[] =
    "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf";

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the kuka robot model.
  MultibodyPlant<double>& kuka_plant = *builder.AddSystem<MultibodyPlant>();
  AddModelFromSdfFile(FindResourceOrThrow(kSdfPath), &kuka_plant, &scene_graph);

  // Add gravity to the model.
  kuka_plant.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  kuka_plant.Finalize(&scene_graph);
  DRAKE_THROW_UNLESS(kuka_plant.num_positions() == 7);
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!kuka_plant.get_source_id());

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

  builder.Connect(
      kuka_plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(kuka_plant.get_source_id().value()));
  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(kSdfPath), multibody::joints::kFixed,
      nullptr, tree.get());

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller = builder.AddSystem<
      systems::controllers::InverseDynamicsController>(
      std::move(tree), iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  // Wire up Kuka plant to controller.
  builder.Connect(kuka_plant.get_continuous_state_output_port(),
                  controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port_control(),
                  kuka_plant.get_actuation_input_port());

  // Wire up output from planned trajectory to controller.
  PiecewisePolynomial<double> traj = MakeControlledKukaPlan();
  auto traj_src =
      builder.AddSystem<systems::TrajectorySource<double>>(
          traj, 1 /* outputs q + v */);
  traj_src->set_name("trajectory_source");

  builder.Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);

  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
