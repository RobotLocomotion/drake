#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

using std::move;

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id, int size)
    : source_id_(source_id), ids_(size), data_(size), next_pose_(0) {}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::clear() {
  next_pose_ = 0;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::set_value(
    FrameId id, const KinematicsValue& value) {
  if (count() >= size()) {
    throw std::runtime_error(
        "Trying to report kinematics data for more frames "
        "than the vector was sized for; did you forget to call clear()?");
  }
  ids_[next_pose_] = id;
  data_[next_pose_] = value;
  ++next_pose_;
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<Isometry3<double>>;
template class FrameKinematicsVector<Isometry3<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake
