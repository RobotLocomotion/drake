#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator.h"

#include <cstdint>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

using std::runtime_error;

ViewerDrawTranslator::ViewerDrawTranslator(
    const RigidBodyTree& tree) :
    LcmAndVectorBaseTranslator(tree.get_number_of_bodies() * kNumStatesPerBody),
    tree_(tree) {
  initialize_draw_message();
}

// TODO(liang.fok) Implement this method.
void ViewerDrawTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  DRAKE_ABORT_MSG("Unable to convert from a drake::lcmt_viewer_draw message to "
                  "a VectorBase object that contains the RigidBodyTree's "
                  "generalized state.");
}

void ViewerDrawTranslator::Serialize(double time,
    const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  std::cout << "************ViewerDrawTranslator::Serialize:" << std::endl
            << "  - vector_base.size() = " << vector_base.size() << std::endl
            << "  - get_vector_size() = " << get_vector_size() << std::endl;
  DRAKE_ASSERT(vector_base.size() == get_vector_size());
  DRAKE_ASSERT(lcm_message_bytes != nullptr);

  // Creates a copy of the partially-initialized lcmt_viewer_draw message.
  // This is necessary since this method is declared const.
  drake::lcmt_viewer_draw message = draw_msg_;

  // Updates the timestamp in the message.
  message.timestamp = static_cast<int64_t>(time * 1000);

  // Obtains the generalized positions from vector_base.
  const Eigen::VectorXd q = vector_base.CopyToVector().head(
      tree_.number_of_positions());

  // Calls RigidBodyTree::doKinematics() using the generalized position vector.
  // This computes the poses of each body.
  KinematicsCache<double> cache = tree_.doKinematics(q);

  // Saves the poses of each body in the lcmt_viewer_draw message.
  for (size_t i = 0; i < tree_.bodies.size(); ++i) {
    auto transform = tree_.relativeTransform(cache, 0, i);
    auto quat = drake::math::rotmat2quat(transform.linear());
    std::vector<float>& position = message.position[i];

    auto translation = transform.translation();
    for (int j = 0; j < 3; ++j) {
      position[j] = static_cast<float>(translation(j));
    }
    std::vector<float>& quaternion = message.quaternion[i];
    for (int j = 0; j < 4; ++j) {
      quaternion[j] = static_cast<float>(quat(j));
    }
  }

  // Serializes the message into an array of bytes.
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void ViewerDrawTranslator::initialize_draw_message() {
  draw_msg_.num_links = tree_.bodies.size();
  std::vector<float> position = {0, 0, 0};
  std::vector<float> quaternion = {0, 0, 0, 1};
  for (const auto& body : tree_.bodies) {
    draw_msg_.link_name.push_back(body->get_name());
    draw_msg_.robot_num.push_back(body->get_model_instance_id());
    draw_msg_.position.push_back(position);
    draw_msg_.quaternion.push_back(quaternion);
  }
}

}  // namespace systems
}  // namespace drake
