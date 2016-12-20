/// @file
///
/// This demo sets up an uncontrolled KUKA iiwa robot within a simulation
/// mounted upon a table and with some simple objects (2 cylinders and
/// 1 cuboid) placed in its vicinity. The uncontrolled iiwa arm collapses
/// under the influence of gravity and results in the objects being scattered
/// due to collisions with the robot arm and among themselves.

#include <gflags/gflags.h>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto iiwa_world = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.

  iiwa_world->StoreModel(
      "iiwa",
      "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");

  iiwa_world->StoreModel(
      "table", "/examples/kuka_iiwa_arm/models/table/"
               "extra_heavy_duty_table_surface_only_collision.sdf");
  iiwa_world->StoreModel(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  iiwa_world->StoreModel(
      "cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");

  iiwa_world->AddFixedModelInstance("table", Eigen::Vector3d::Zero() /* xyz */,
                                    Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddGround();

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // The positions of the iiwa robot, two cylinders and the cuboid are
  // distributed over the surface of the heavy duty table. Only the positions
  // are set; the default orientations are used in each case.
  const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);
  const Eigen::Vector3d kBoxBase(-0.53, -0.35, kTableTopZInWorld + 0.15);
  const Eigen::Vector3d kCylinder1Base(-0.5, -0.51, kTableTopZInWorld + 0.1);
  const Eigen::Vector3d kCylinder2Base(-0.32, -0.325, kTableTopZInWorld + 0.1);

  iiwa_world->AddFixedModelInstance("iiwa", kRobotBase);
  iiwa_world->AddFloatingModelInstance("cylinder", kCylinder1Base);
  iiwa_world->AddFloatingModelInstance("cylinder", kCylinder2Base);
  iiwa_world->AddFloatingModelInstance("cuboid", kBoxBase);

  lcm::DrakeLcm lcm;

  auto visualized_plant = std::make_unique<VisualizedPlant<double>>(
      iiwa_world->Build(), 4500 /* penetration_stiffness */,
      1.0 /* penetration_damping */, 1.0 /* contact friction */, &lcm);

  auto demo_plant = std::make_unique<PassiveVisualizedPlant<double>>(
      std::move(visualized_plant));

  auto simulator = std::make_unique<systems::Simulator<double>>(*demo_plant);

  systems::Context<double>* context = simulator->get_mutable_context();
  demo_plant->SetDefaultState(*context, context->get_mutable_state());

  simulator->Initialize();
  simulator->set_target_realtime_rate(1.0);

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
