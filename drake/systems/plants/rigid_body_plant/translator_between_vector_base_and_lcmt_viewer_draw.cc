// NOLINT(readability/line_length)
#include "drake/systems/plants/rigid_body_plant/translator_between_vector_base_and_lcmt_viewer_draw.h"

#include <cstdint>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

using std::runtime_error;

// Defines the number of states per body. There are seven states: three position
// states (x, y, and z) and four orientation states (w, x, y, z).
const int kNumStatesPerBody = 7;

// NOLINT(readability/line_length)
TranslatorBetweenVectorBaseAndLcmtViewerDraw::TranslatorBetweenVectorBaseAndLcmtViewerDraw(
    const RigidBodyTree& tree) :
    LcmAndVectorBaseTranslator(tree.number_of_positions() +
                               tree.number_of_velocities()),
    tree_(tree) {
  initialize_draw_message();
}

void TranslatorBetweenVectorBaseAndLcmtViewerDraw::TranslateLcmToVectorBase(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  DRAKE_DEMAND(vector_base);

  // Decodes the LCM message using data from the receive buffer.
  drake::lcmt_viewer_draw message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw runtime_error(
      "drake::systems::plants::rigid_body_plant::"
      "TranslatorBetweenVectorBaseAndLcmtViewerDraw: "
      "TranslateLcmToBasicVector: ERROR: Failed to decode LCM message, the "
      "status is " + std::to_string(status) + ".");
  }

  // Verifies that the size of the LCM message matches the size of the basic
  // vector. Throws an exception if the sizes do not match.
  if (message.num_links * kNumStatesPerBody != vector_base->size()) {
    throw runtime_error(
      "drake::systems::plants::rigid_body_plant::"
      "TranslatorBetweenVectorBaseAndLcmtViewerDraw: "
      "TranslateLcmToBasicVector: ERROR: Size of LCM message (" +
      std::to_string(message.num_links * kNumStatesPerBody) +
      ") is not equal to the size of the vector vector (" +
      std::to_string(vector_base->size()) + ").");
  }

  // Saves the values from the LCM message into the basic vector.
  // Assumes that the order of the values in both vectors are identical.
  // for (int ii = 0; ii < message.dim; ++ii) {
  //   vector_base->SetAtIndex(ii, message.val[ii]);
  // }
}

void TranslatorBetweenVectorBaseAndLcmtViewerDraw::TranslateVectorBaseToLcm(
    const VectorBase<double>& vector_base,
    double time,
    std::vector<uint8_t>* lcm_message_bytes) const {
  DRAKE_ASSERT(vector_base.size() == get_vector_size());
  DRAKE_ASSERT(lcm_message_bytes != nullptr);

  // Create a copy of the partially-initialized draw message.
  // This is necessary since this method is declared const.
  drake::lcmt_viewer_draw message = draw_msg_;

  // Instantiates and initializes a LCM message capturing the state of
  // parameter vector_base.
  message.timestamp = time;

  const Eigen::VectorXd q = vector_base.CopyToVector().head(
      tree_.number_of_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

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

  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void TranslatorBetweenVectorBaseAndLcmtViewerDraw::initialize_draw_message() {
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
