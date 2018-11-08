#include "drake/manipulation/util/simple_tree_visualizer.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace manipulation {

SimpleTreeVisualizer::SimpleTreeVisualizer(const RigidBodyTreed& tree,
                                           lcm::DrakeLcmInterface* lcm)
    : tree_(tree),
      state_dimension_(tree_.get_num_positions() + tree_.get_num_velocities()),
      draw_message_translator_(tree_),
      lcm_(lcm) {
  // Loads the robot.
  const lcmt_viewer_load_robot load_message(
      multibody::CreateLoadRobotMessage<double>(tree_));

  Publish(lcm_, "DRAKE_VIEWER_LOAD_ROBOT", load_message);
}

void SimpleTreeVisualizer::visualize(const VectorX<double>& position_vector) {
  DRAKE_DEMAND(position_vector.size() == tree_.get_num_positions());

  std::vector<uint8_t> message_bytes;
  constexpr double kTime = 0;
  draw_message_translator_.Serialize(
      kTime, systems::BasicVector<double>{position_vector}, &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size(), kTime);
}

}  // namespace manipulation
}  // namespace drake
