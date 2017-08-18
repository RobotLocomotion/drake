#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace pr2 {

void DoMain() { 
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(FindResourceOrThrow(
      "drake/examples/pr2/models/pr2_description/urdf/"
      "pr2_simplified.urdf"),
      multibody::joints::kFixed, nullptr, &tree);

  DRAKE_DEMAND(28 == tree.get_num_positions());
  const int num_actuators = 28;

  std::cout << "didin't crash1" << "\n";

  const manipulation::RobotStateLcmMessageTranslator translator(tree);

  std::cout << "didin't crash2" << "\n";

  Eigen::VectorXd initial_pr2_state(num_actuators*2);
  initial_pr2_state << 0, 0, 0, 0.3, 0, 0, -1.14, 1.11, -1.40, -2.11,
      -1.33, -1.12, 2.19, 0.2, 0.2, 0.2, 0.2, 2.1, 1.29, 0 - 0.15, 0, -0.1, 0,
      0.2, 0.2, 0.2, 0.2, VectorX<double>::Zero(num_actuators);

  VectorX<double> q = Eigen::VectorBlock<VectorX<double>, num_actuators>(initial_pr2_state, 0);
  VectorX<double> v = Eigen::VectorBlock<VectorX<double>, num_actuators>(initial_pr2_state, num_actuators);

  std::vector<VectorX<double>> q_sequence;
  q_sequence.push_back(q);
  while(q[6] + 0.15 < 0 || q[7] - 0.15 > 0 || q[8] + 0.15 < 0 || q[9] + 0.15 < 0 || q[10] + 0.15 < 0 || q[11] + 0.15 < 0 || q[12] - 0.15 > 0){
    if(q[6] + 0.15 < 0){
      q[6] += 0.1;
    }
    if(q[7] - 0.15 > 0){
      q[7] -= 0.1;
    }
    if(q[8] + 0.15 < 0){
      q[8] += 0.1;
    }
    if(q[9] + 0.15 < 0){
      q[9] += 0.1;
    }
    if(q[10] + 0.15 < 0){
      q[10] += 0.1;
    }
    if(q[11] + 0.15 < 0){
      q[11] += 0.1;
    }
    if(q[12] - 0.15 > 0){
      q[12] -= 0.1;
    }
    q_sequence.push_back(q);
  }
  
  robotlocomotion::robot_plan_t msg{};
  msg.utime = time(NULL);
  msg.num_states = q_sequence.size();
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);

  for(int index = 0; index < (int)q_sequence.size(); index ++){
    translator.InitializeMessage(&(msg.plan[index]));
    translator.EncodeMessageKinematics(q_sequence[index], v, &(msg.plan[index]));
    msg.plan[index].utime = 1e6;
  }

  lcm::LCM lcm;
  lcm.publish("PR2_PLAN", &msg);
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::pr2::DoMain();
  return 0;
}
