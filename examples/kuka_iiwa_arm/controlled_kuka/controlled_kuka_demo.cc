/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/controlled_kuka_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::util::SimDiagramBuilder;
using trajectories::PiecewisePolynomial;

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  CreateTreedFromFixedModelAtPose(FindResourceOrThrow(kUrdfPath), tree.get());

  PiecewisePolynomial<double> traj = MakeControlledKukaPlan();

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  // Adds a plant
  auto plant = builder.AddPlant(std::move(tree));
  builder.AddVisualizer(&lcm);

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  auto controller = builder.AddController<
      systems::controllers::rbt::InverseDynamicsController<double>>(
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
      plant->get_rigid_body_tree().Clone(), iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  // Adds a trajectory source for desired state.
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();
  auto traj_src =
      diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
          traj, 1 /* outputs q + v */);
  traj_src->set_name("trajectory_source");

  diagram_builder->Connect(traj_src->get_output_port(),
                           controller->get_input_port_desired_state());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

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
