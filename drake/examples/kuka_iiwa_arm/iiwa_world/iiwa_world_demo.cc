/// @file
///
/// This demo sets up an uncontrolled KUKA iiwa robot within a simulation
/// mounted upon a table and with some simple objects (2 cylinders and
/// 1 cuboid) placed in its vicinity. The uncontrolled iiwa arm collapses
/// under the influence of gravity and results in the objects being scattered
/// due to collisions with the robot arm and among themselves.

#include <gflags/gflags.h>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_sim_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto iiwa_world = std::make_unique<IiwaWorldSimBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  iiwa_world->StoreModel("iiwa", "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");
  iiwa_world->StoreModel(
      "table",
      "/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  iiwa_world->StoreModel(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  iiwa_world->StoreModel(
      "cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");

  iiwa_world->AddFixedModelInstance("table", Eigen::Vector3d::Zero() /* xyz */,
                                    Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddGround();

  iiwa_world->SetPenetrationContactParameters(4500 /* penetration_stiffness */,
                                              1.0 /* penetration_damping */,
                                              1.0 /* contact friction */);

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

  // Sets up a builder for the demo.
  std::unique_ptr<drake::systems::DiagramBuilder<double>> demo_builder{
      std::make_unique<drake::systems::DiagramBuilder<double>>()};

  auto iiwa_plant_diagram =
      demo_builder->template AddSystem(iiwa_world->Build());

  // Instantiates a constant source that outputs a vector of zeros.
  VectorX<double> constant_value(iiwa_world->GetPlantInputSize());
  constant_value.setZero();

  auto const_source_ =
      demo_builder->template AddSystem<systems::ConstantVectorSource<double>>(
          constant_value);

  // Cascades the constant source to the iiwa plant diagram. This effectively
  // results in the robot being uncontrolled.
  demo_builder->Cascade(*const_source_, *iiwa_plant_diagram);

  auto demo_diagram = demo_builder->Build();

  auto simulator = std::make_unique<systems::Simulator<double>>(*demo_diagram);

  iiwa_world->SetZeroConfiguration(simulator.get(), demo_diagram.get());

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
