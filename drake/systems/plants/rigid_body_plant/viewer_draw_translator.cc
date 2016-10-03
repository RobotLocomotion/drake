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
    LcmAndVectorBaseTranslator(
        tree.get_num_positions() + tree.get_num_velocities()),
    tree_(tree) {
  // Initializes the draw message.
  draw_message_.num_links = tree_.bodies.size();
  std::vector<float> position = {0, 0, 0};
  std::vector<float> quaternion = {0, 0, 0, 1};
  for (const auto& body : tree_.bodies) {
    draw_message_.link_name.push_back(body->get_name());
    draw_message_.robot_num.push_back(body->get_model_instance_id());
    draw_message_.position.push_back(position);
    draw_message_.quaternion.push_back(quaternion);
  }
}

void ViewerDrawTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  DRAKE_ABORT_MSG(
    "The translator that converts from a drake::lcmt_viewer_draw message to "
    "a VectorBase object that contains the RigidBodyTree's state vector has not"
    "been implemented yet.");
}

void ViewerDrawTranslator::Serialize(double time,
    const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  DRAKE_DEMAND(vector_base.size() == get_vector_size());
  DRAKE_DEMAND(lcm_message_bytes != nullptr);

  // Creates a copy of the partially-initialized lcmt_viewer_draw message.
  // This is necessary since this method is declared const.
  drake::lcmt_viewer_draw message = draw_message_;

  // Updates the timestamp in the message.
  message.timestamp = static_cast<int64_t>(time * 1000);

  // Obtains the generalized positions from vector_base.
  const Eigen::VectorXd q = vector_base.CopyToVector().head(
      tree_.get_num_positions());

  // Computes the poses of each body.
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

}  // namespace systems
}  // namespace drake
