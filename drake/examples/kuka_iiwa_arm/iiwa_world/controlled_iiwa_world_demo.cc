/// @file
///
/// This demo sets up an controlled KUKA iiwa robot within a simulation
/// mounted upon a table and with some simple objects (3 cylinders) placed
/// in its vicinity. The iiwa arm follows a plan that results in collision
/// with each of these objects in a fixed sequence leading to some of the
/// cylinders being knocked off the table.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/systems/analysis/simulator.h"

DEFINE_double(simulation_sec, 4.5, "Number of seconds to simulate.");

using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {

using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;

namespace examples {
namespace kuka_iiwa_arm {

namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto world_sim_tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world. kRobotName alone is declared independently
  // since it is needed again later.
  const string kRobotName =
      "/examples/kuka_iiwa_arm/urdf/"
      "iiwa14_simplified_collision.urdf";

  world_sim_tree_builder->StoreModel("iiwa", kRobotName);
  world_sim_tree_builder->StoreModel("table",
                                     "/examples/kuka_iiwa_arm/models/table/"
                                     "extra_heavy_duty_table_surface_only_"
                                     "collision.sdf");
  world_sim_tree_builder->StoreModel(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");

  world_sim_tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d::Zero() /* xyz */,
      Eigen::Vector3d::Zero() /* rpy */);
  world_sim_tree_builder->AddGround();

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;
  const double kTableTopXInWorld = -0.243716;
  const double kTableTopYInWorld = -0.625087;

  const double kTableSurfaceSizeX = 0.7112;
  const double kTableSurfaceSizeY = 0.762;

  const double kGapToEdge = 0.05;

  // The positions of the iiwa robot and the 3 cylinders are distributed
  // over the surface of the heavy duty table. Only the positions are set; the
  // default orientations are used in each case. While 2 of the cylinders
  // are located at table corners, the third cylinder is placed exactly
  // in between them.
  const Eigen::Vector3d kRobotBase(kTableTopXInWorld, kTableTopYInWorld,
                                   kTableTopZInWorld);

  const Eigen::Vector3d kCylinderCorner1Base(
      kTableTopXInWorld + 0.5 * kTableSurfaceSizeX - kGapToEdge,
      kTableTopYInWorld + 0.5 * kTableSurfaceSizeY - kGapToEdge,
      kTableTopZInWorld + 0.1);

  const Eigen::Vector3d kCylinderCorner2Base(
      kTableTopXInWorld + 0.5 * kTableSurfaceSizeX - kGapToEdge,
      kTableTopYInWorld - 0.5 * kTableSurfaceSizeY + kGapToEdge,
      kTableTopZInWorld + 0.1);

  // Adding each model to the Tree builder.
  int robot_model_instance =
      world_sim_tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  world_sim_tree_builder->AddFloatingModelInstance("cylinder",
                                                   kCylinderCorner1Base);

  world_sim_tree_builder->AddFloatingModelInstance("cylinder",
                                                   kCylinderCorner2Base);

  world_sim_tree_builder->AddFloatingModelInstance(
      "cylinder", 0.5 * (kCylinderCorner1Base + kCylinderCorner2Base));
  vector<Eigen::Vector3d> target_position_vector;

  // The following desired trajectory was hand crafted in order for the
  // KUKA iiwa arm to knock over each of the cylinders in sequence.
  // The desired positions coincide with the upper edge of each of the
  // cylinders.
  target_position_vector.push_back(
      0.5 * (kCylinderCorner2Base + kCylinderCorner1Base) +
      Eigen::Vector3d(-0.05, 0, 0.5));
  target_position_vector.push_back(kCylinderCorner2Base +
                                   Eigen::Vector3d(-0.05, 0, 0.05));
  target_position_vector.push_back(kCylinderCorner1Base +
                                   Eigen::Vector3d(-0.05, 0, 0.05));
  target_position_vector.push_back(
      0.5 * (kCylinderCorner2Base + kCylinderCorner1Base) +
      Eigen::Vector3d(-0.05, 0, 0.05));
  target_position_vector.push_back(
      0.5 * (kCylinderCorner2Base + kCylinderCorner1Base) +
      Eigen::Vector3d(-0.05, 0, 0.5));

  // Way point time vectors were hand crafted to obtain a nice demo.
  vector<double> target_time_vector = {1.0, 1.75, 2.25, 3.0, 3.75};

  // Initializes a robot tree for use in the inverse kinematics and
  // in the gravity compensation control.
  RigidBodyTreed robot_tree;
  CreateTreedFromFixedModelAtPose(kRobotName, &robot_tree, kRobotBase);

  unique_ptr<PiecewisePolynomialTrajectory> polynomial_trajectory =
      SimpleCartesianWayPointPlanner(robot_tree, "iiwa_link_ee",
                                     target_position_vector,
                                     target_time_vector);

  lcm::DrakeLcm lcm;
  auto demo_plant = std::make_unique<PositionControlledPlantWithRobot<double>>(
      world_sim_tree_builder->Build(), std::move(polynomial_trajectory),
      robot_model_instance, robot_tree, 3000 /* penetration_stiffness */,
      10.0 /* penetration_damping */, 10.0 /* contact friction */, &lcm);

  auto simulator = std::make_unique<systems::Simulator<double>>(*demo_plant);

  auto context = simulator->get_mutable_context();
  demo_plant->SetDefaultState(*context, context->get_mutable_state());

  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  simulator->StepTo(FLAGS_simulation_sec);

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
