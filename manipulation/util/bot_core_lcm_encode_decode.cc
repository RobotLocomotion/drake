#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"

#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"

using Eigen::Isometry3d;

void EncodeVector3d(const Eigen::Ref<const Eigen::Vector3d>& vec,
                    // NOLINTNEXTLINE(runtime/references)
                    bot_core::vector_3d_t& msg) {
  msg.x = vec[0];
  msg.y = vec[1];
  msg.z = vec[2];
}

Eigen::Vector3d DecodeVector3d(const bot_core::vector_3d_t& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

void EncodeQuaternion(const Eigen::Ref<const Eigen::Vector4d>& vec,
                      // NOLINTNEXTLINE(runtime/references)
                      bot_core::quaternion_t& msg) {
  msg.w = vec[0];
  msg.x = vec[1];
  msg.y = vec[2];
  msg.z = vec[3];
}

Eigen::Vector4d DecodeQuaternion(const bot_core::quaternion_t& msg) {
  return Eigen::Vector4d(msg.w, msg.x, msg.y, msg.z);
}

void EncodePose(const Eigen::Isometry3d& pose,
                // NOLINTNEXTLINE(runtime/references)
                bot_core::position_3d_t& msg) {
  auto rotation = drake::math::rotmat2quat(pose.linear());
  EncodeQuaternion(rotation, msg.rotation);
  EncodeVector3d(pose.translation(), msg.translation);
}

Eigen::Isometry3d DecodePose(const bot_core::position_3d_t& msg) {
  Isometry3d ret;
  ret.translation() = DecodeVector3d(msg.translation);
  auto quaternion = DecodeQuaternion(msg.rotation);
  ret.linear() = drake::math::quat2rotmat(quaternion);
  ret.makeAffine();
  return ret;
}

void EncodeTwist(const Eigen::Ref<const drake::TwistVector<double>>& twist,
                 // NOLINTNEXTLINE(runtime/references)
                 bot_core::twist_t& msg) {
  EncodeVector3d(twist.head<3>(), msg.angular_velocity);
  EncodeVector3d(twist.tail<3>(), msg.linear_velocity);
}

drake::TwistVector<double> DecodeTwist(const bot_core::twist_t& msg) {
  drake::TwistVector<double> ret;
  ret.head<3>() = DecodeVector3d(msg.angular_velocity);
  ret.tail<3>() = DecodeVector3d(msg.linear_velocity);
  return ret;
}
