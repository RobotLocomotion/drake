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
  auto iiwa_world = std::make_unique<IiwaWorldSimBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  iiwa_world->AddObjectUrdf("iiwa", "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");
  iiwa_world->AddObjectUrdf(
      "table",
      "/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  iiwa_world->AddObjectUrdf(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  iiwa_world->AddObjectUrdf(
      "cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");

  iiwa_world->AddObjectFixedToWorld("table", Eigen::Vector3d::Zero() /* xyz */,
                                    Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddGround();

  iiwa_world->SetPenetrationContactParameters(4500 /* penetration_stiffness */,
                                              1.0 /* penetration_damping */,
                                              1.0 /* contact friction */);

  // The z coordinate of the top of the table in the world frame.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // The positions of the iiwa robot, two cylinders and the cuboid was fixed
  // in a manner that they are distributed over the surface of the heavy duty
  // table. Only the positions are set, as the default orientations are used
  // in each case.
  const Eigen::Vector3d kRobotBase(-0.25, -0.75, kTableTopZInWorld);
  const Eigen::Vector3d kBoxBase(-0.45, -0.4, kTableTopZInWorld + 0.15);
  const Eigen::Vector3d kCylinder1Base(-0.5, -0.60, kTableTopZInWorld + 0.1);
  const Eigen::Vector3d kCylinder2Base(-0.05, -0.75, kTableTopZInWorld + 0.1);

  iiwa_world->AddObjectFixedToWorld("iiwa", kRobotBase,
                                    Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddObjectFloatingInWorld("cylinder", kCylinder1Base,
                                       Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddObjectFloatingInWorld("cylinder", kCylinder2Base,
                                       Eigen::Vector3d::Zero() /* rpy */);
  iiwa_world->AddObjectFloatingInWorld("cuboid", kBoxBase,
                                       Eigen::Vector3d::Zero() /* rpy */);

  // Setup builder for the demo
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

  iiwa_world->SetZeroConfiguration(simulator.get(), demo_diagram.get(),
                                   iiwa_plant_diagram);

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
