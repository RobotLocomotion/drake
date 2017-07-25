/**
 * @brief This is a demo program that sends a full body manipulation plan
 * encoded as a robotlocomotion::robot_plan_t to ValkyrieController. Upon
 * receiving the plan, the Valkyrie robot should return to the nominal
 * configuration except the right shoulder pitch joint.
 */
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

using std::default_random_engine;

namespace drake {
namespace {

void send_manip_message() {
  RigidBodyTree<double> robot;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      multibody::joints::kRollPitchYaw, &robot);

  DRAKE_DEMAND(examples::valkyrie::kRPYValkyrieDof ==
               robot.get_num_positions());
  VectorX<double> q = examples::valkyrie::RPYValkyrieFixedPointState().head(
      examples::valkyrie::kRPYValkyrieDof);
  VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());

  const manipulation::RobotStateLcmMessageTranslator translator(robot);

  // There needs to be at least 1 knot point, the controller will insert its
  // current desired q to the beginning to make the desired trajectories.
  //
  // Some notes about the message:
  // 1. The first timestamp needs to be bigger than 0.
  // 2. msg.utime needs to be different for different messages. The controller
  // ignores messages with the same utime.
  // 3. Assumes the message is packaged with a RPY floating joint.
  // 4. Assumes that the given knot points are stable given the current actual
  // contact points.
  // 5. Pelvis z and orientation + torso orientation are tracked in Cartesian
  // mode (These can be changed by the gains in the configuration file). CoM
  // in controlled by a LQR like controller, where the desired is given by the
  // knot points specified here.
  robotlocomotion::robot_plan_t msg{};
  msg.utime = time(NULL);
  msg.num_states = 1;
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);

  q[10] -= 0.5;  // right shoulder pitch
  translator.InitializeMessage(&(msg.plan[0]));
  translator.EncodeMessageKinematics(q, v, &(msg.plan[0]));
  msg.plan[0].utime = 1e6;

  lcm::LCM lcm;
  lcm.publish("VALKYRIE_MANIP_PLAN", &msg);

  sleep(1);
}

}  // namespace
}  // namespace drake

int main() {
  drake::send_manip_message();
  return 0;
}
