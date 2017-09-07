#include "drake/manipulation/util/perception_utils.h"

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace manipulation {
namespace util {

using Eigen::Quaterniond;

Isometry3<double> VectorToIsometry3d(const VectorX<double>& pose_vector) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  pose.linear() = Quaterniond(pose_vector(3), pose_vector(4), pose_vector(5),
                              pose_vector(6))
                      .toRotationMatrix();
  pose.translation() = pose_vector.head<3>();
  pose.makeAffine();
  return pose;
}

VectorX<double> Isometry3dToVector(const Isometry3<double>& pose) {
  VectorX<double> pose_vector = VectorX<double>::Zero(7);
  pose_vector.head<3>() = pose.translation();
  Quaterniond return_quat = Quaterniond(pose.linear());
  return_quat = math::QuaternionToCanonicalForm(return_quat);

  pose_vector.tail<4>() = (VectorX<double>(4) << return_quat.w(),
                           return_quat.x(), return_quat.y(), return_quat.z())
                              .finished();

  return pose_vector;
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
