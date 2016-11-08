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
  Eigen::Vector3d robot_base(-0.2, -0.2, 0.736);


  simulator->AddObjectFixedToWorld(Eigen::Vector3d::Zero() /* xyz */,
                                   Eigen::Vector3d::Zero() /* rpy */,
                                   "table");

  simulator->AddObjectFixedToWorld(robot_base,
                                   Eigen::Vector3d::Zero() /* rpy */,
                                   "iiwa");

//  simulator->AddObjectFixedToTable(robot_base,
//                                  Eigen::Vector3d::Zero() /* rpy */,
//                                  "iiwa");


  Eigen::Vector3d cylinder_base(0.2, 0.2, 0);
  simulator->AddObjectFixedToWorld(cylinder_base,
                                  Eigen::Vector3d::Zero() /* rpy */,
                                  "cylinder");
  Eigen::Vector3d box_base(0.4, 0.2, 0);
  simulator->AddObjectFixedToWorld(box_base,
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
