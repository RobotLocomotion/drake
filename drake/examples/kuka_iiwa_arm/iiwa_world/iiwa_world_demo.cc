#include <iostream>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_sim_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int main(int argc, char* argv[]) {
  auto iiwa_world = std::make_unique<IiwaWorldSimBuilder<double>>();

  // Adding URDFs
  iiwa_world->AddObjectUrdf("iiwa", "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");
  iiwa_world->AddObjectUrdf(
      "table",
      "/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  iiwa_world->AddObjectUrdf(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  iiwa_world->AddObjectUrdf(
      "cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");

  iiwa_world->AddObjectFixedToWorld(Eigen::Vector3d::Zero() /* xyz */,
                                    Eigen::Vector3d::Zero() /* rpy */, "table");
  iiwa_world->AddGroundToTree();

  iiwa_world->SetPenetrationContactParameters(4500 /* penetration_stiffness */,
                                              1.0 /* penetration_damping */,
                                              1.0 /* contact friction */);
  double table_top_z_in_world = 0.736 + 0.057 / 2;
  Eigen::Vector3d robot_base(-0.25, -0.75, table_top_z_in_world);
  iiwa_world->AddObjectFixedToWorld(robot_base,
                                    Eigen::Vector3d::Zero() /* rpy */, "iiwa");
  Eigen::Vector3d box_base(-0.45, -0.4, table_top_z_in_world + 0.15);
  Eigen::Vector3d cylinder_1_base(-0.5, -0.60, table_top_z_in_world + 0.1);
  Eigen::Vector3d cylinder_2_base(-0.05, -0.75, table_top_z_in_world + 0.1);

  iiwa_world->AddObjectFloatingToWorld(
      cylinder_1_base, Eigen::Vector3d::Zero() /* rpy */, "cylinder");
  iiwa_world->AddObjectFloatingToWorld(
      cylinder_2_base, Eigen::Vector3d::Zero() /* rpy */, "cylinder");
  iiwa_world->AddObjectFloatingToWorld(
      box_base, Eigen::Vector3d::Zero() /* rpy */, "cuboid");

  auto diagram = iiwa_world->Build();
  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);
  iiwa_world->SetZeroConfiguration(simulator.get(), diagram.get());

  simulator->Initialize();
  simulator->StepTo(3.5 /* final time */);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
