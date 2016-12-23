#include <gflags/gflags.h>

#include "drake/automotive/car_sim_lcm_common.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

DEFINE_bool(with_speed_bump, false,
    "Whether to include a speed bump in front of the vehicle.");

using std::make_unique;
using std::move;

namespace drake {

using lcm::DrakeLcm;
using parsers::sdf::AddModelInstancesFromSdfFile;
using systems::Diagram;
using systems::Simulator;

namespace automotive {
namespace {

// Verifies that the order of rigid body names and actuator names within the
// provided tree are as expected.
void VerifyCarSimLcmTree(const RigidBodyTreed& tree, int expected_num_bodies) {
  DRAKE_DEMAND(tree.get_num_bodies() == expected_num_bodies);

  std::map<std::string, int> name_to_idx =
      tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("base_x"));
  DRAKE_DEMAND(name_to_idx["base_x"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_y"));
  DRAKE_DEMAND(name_to_idx["base_y"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_z"));
  DRAKE_DEMAND(name_to_idx["base_z"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qw"));
  DRAKE_DEMAND(name_to_idx["base_qw"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qx"));
  DRAKE_DEMAND(name_to_idx["base_qx"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qy"));
  DRAKE_DEMAND(name_to_idx["base_qy"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("base_qz"));
  DRAKE_DEMAND(name_to_idx["base_qz"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("steering"));
  DRAKE_DEMAND(name_to_idx["steering"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("left_pin"));
  DRAKE_DEMAND(name_to_idx["left_pin"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("left_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["left_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("axle_tie_rod_arm"));
  DRAKE_DEMAND(name_to_idx["axle_tie_rod_arm"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("right_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["right_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("rear_left_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["rear_left_wheel_joint"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("rear_right_wheel_joint"));
  DRAKE_DEMAND(name_to_idx["rear_right_wheel_joint"] == joint_idx++);

  DRAKE_DEMAND(tree.actuators.size() == 3);
  DRAKE_DEMAND(tree.actuators.at(0).name_ == "steering");
  DRAKE_DEMAND(tree.actuators.at(1).name_ == "left_wheel_joint");
  DRAKE_DEMAND(tree.actuators.at(2).name_ == "right_wheel_joint");
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto rigid_body_tree = make_unique<RigidBodyTreed>();
  AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
      multibody::joints::kQuaternion, nullptr /* weld to frame */,
      rigid_body_tree.get());
  multibody::AddFlatTerrainToWorld(rigid_body_tree.get());
  if (FLAGS_with_speed_bump) {
    AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/automotive/models/speed_bump/speed_bump.sdf",
      multibody::joints::kFixed, nullptr /* weld to frame */,
      rigid_body_tree.get());
    VerifyCarSimLcmTree(*rigid_body_tree, 19);
  } else {
    VerifyCarSimLcmTree(*rigid_body_tree, 18);
  }

  lcm::DrakeLcm lcm;
  DrivingCommandTranslator driving_command_translator;
  std::unique_ptr<systems::Diagram<double>> diagram =
      CreatCarSimLcmDiagram(driving_command_translator, move(rigid_body_tree),
                            &lcm);
  lcm.StartReceiveThread();
  Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::automotive::main(argc, argv);
}
