#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"

#include "drake/math/quaternion.h"
#include "drake/math/rigid_transform.h"

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

void EncodeQuaternion(const Eigen::Quaterniond& q,
                      // NOLINTNEXTLINE(runtime/references)
                      bot_core::quaternion_t& msg) {
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
}

Eigen::Vector4d DecodeQuaternion(const bot_core::quaternion_t& msg) {
  return Eigen::Vector4d(msg.w, msg.x, msg.y, msg.z);
}

void EncodePose(const Eigen::Isometry3d& pose,
                // NOLINTNEXTLINE(runtime/references)
                bot_core::position_3d_t& msg) {
  const Eigen::Quaterniond q =
      drake::math::RotationMatrix<double>::ToQuaternion(pose.linear());
  EncodeQuaternion(q, msg.rotation);
  EncodeVector3d(pose.translation(), msg.translation);
}

Eigen::Isometry3d DecodePose(const bot_core::position_3d_t& msg) {
  const Eigen::Vector3d position = DecodeVector3d(msg.translation);
  const Eigen::Vector4d wxyz = DecodeQuaternion(msg.rotation);
  const Eigen::Quaterniond quat(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  const drake::math::RigidTransformd X(quat, position);
  return X.GetAsIsometry3();
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
