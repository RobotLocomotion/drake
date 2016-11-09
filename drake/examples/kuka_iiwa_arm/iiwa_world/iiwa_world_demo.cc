#include <iostream>
#include <memory>

#include <gflags/gflags.h>

//#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int main(int argc, char* argv[]) {

  auto simulator = std::make_unique<IiwaWorldSimulator<double>>();

  simulator->AddObjectFixedToWorld(Eigen::Vector3d::Zero() /* xyz */,
                                   Eigen::Vector3d::Zero() /* rpy */,
                                   "table");
  simulator->AddGroundToTree();

  double table_top_z_in_world = 0.736 + 0.057/2;

  Eigen::Vector3d robot_base(-0.25, -0.75, table_top_z_in_world);

  simulator->AddObjectFixedToWorld(robot_base,
                                   Eigen::Vector3d::Zero() /* rpy */,
                                   "iiwa");

//  simulator->AddObjectFixedToTable(robot_base,
//                                  Eigen::Vector3d::Zero() /* rpy */,
//                                  "iiwa");

  Eigen::Vector3d box_base(-0.45, -0.4, table_top_z_in_world + 0.15);
  Eigen::Vector3d cylinder_1_base(-0.5, -0.60, table_top_z_in_world + 0.1);
  Eigen::Vector3d cylinder_2_base(-0.05, -0.75, table_top_z_in_world + 0.1);
//
//    simulator->AddObjectFixedToWorld(cylinder_1_base,
//                                  Eigen::Vector3d::Zero() /* rpy */,
//                                  "cylinder");

  simulator->AddObjectFloatingToWorld(cylinder_1_base,
                                  Eigen::Vector3d::Zero() /* rpy */,
                                  "cylinder");
  simulator->AddObjectFloatingToWorld(cylinder_2_base,
                                      Eigen::Vector3d::Zero() /* rpy */,
                                      "cylinder");
  simulator->AddObjectFloatingToWorld(box_base,
                                  Eigen::Vector3d::Zero() /* rpy */,
                                  "cuboid");
  simulator->Build();

  while (true) {
    simulator->StepBy(0.01);
  }

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
