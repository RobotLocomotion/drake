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
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using trajectories::PiecewisePolynomial;

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_no_collision.urdf";

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the kuka robot model.
  MultibodyPlant<double>& kuka_plant = *builder.AddSystem<MultibodyPlant>();
  Parser(&kuka_plant, &scene_graph).AddModelFromFile(
      FindResourceOrThrow(kUrdfPath));
  kuka_plant.WeldFrames(kuka_plant.world_frame(),
                        kuka_plant.GetFrameByName("base"));

  // Add gravity to the model.
  kuka_plant.AddForceElement<UniformGravityFieldElement>();

  // Now the model is complete.
  kuka_plant.Finalize();
  DRAKE_THROW_UNLESS(kuka_plant.num_positions() == 7);
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!kuka_plant.get_source_id());

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller = builder.AddSystem<
      systems::controllers::InverseDynamicsController>(
      kuka_plant,
      iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  // Wire up Kuka plant to controller.
  builder.Connect(kuka_plant.get_state_output_port(),
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

  builder.Connect(
      kuka_plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(kuka_plant.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.get_mutable_integrator().set_target_accuracy(1e-3);

  simulator.AdvanceTo(FLAGS_simulation_sec);
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
