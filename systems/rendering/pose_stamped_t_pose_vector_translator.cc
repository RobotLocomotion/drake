#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include <string>

#include "robotlocomotion/pose_stamped_t.hpp"

#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace rendering {

namespace {
const int kSecToMicrosec = 1000000;
}

PoseStampedTPoseVectorTranslator::PoseStampedTPoseVectorTranslator(
    const std::string& frame_name)
    : lcm::LcmAndVectorBaseTranslator(PoseVector<double>::kSize),
      frame_name_(frame_name) {
}

void PoseStampedTPoseVectorTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    VectorBase<double>* vector_base) const {
  robotlocomotion::pose_stamped_t pose_msg;
  pose_msg.decode(lcm_message_bytes, 0, lcm_message_length);

  Eigen::Translation<double, 3> t(
      pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);

  const auto& q = pose_msg.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();

  auto pose_vector = dynamic_cast<PoseVector<double>*>(vector_base);
  pose_vector->set_translation(t);
  pose_vector->set_rotation(quat);
}

void PoseStampedTPoseVectorTranslator::Serialize(
    double time, const VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  robotlocomotion::pose_stamped_t pose_msg;
  pose_msg.header.utime = static_cast<int64_t>(time * kSecToMicrosec);
  pose_msg.header.frame_name = frame_name_;
  // TODO(kunimatsu-tri) Implement seq count in LcmPublisherSystem.
  pose_msg.header.seq = 0;

  auto pose_vector = dynamic_cast<const PoseVector<double>*>(&vector_base);
  const auto t = pose_vector->get_translation();
  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();

  const auto quat = pose_vector->get_rotation();
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  const int encoded_size = pose_msg.getEncodedSize();
  lcm_message_bytes->resize(encoded_size);
  pose_msg.encode(lcm_message_bytes->data(), 0, encoded_size);
}

std::unique_ptr<BasicVector<double>>
PoseStampedTPoseVectorTranslator::AllocateOutputVector() const {
  return std::make_unique<rendering::PoseVector<double>>();
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake
