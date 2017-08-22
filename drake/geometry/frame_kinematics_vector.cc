#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>

namespace drake {
namespace geometry {

using std::move;

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id)
    : std::vector<KinematicsValue>(), source_id_(source_id) {}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id, const std::vector<KinematicsValue>& values)
    : std::vector<KinematicsValue>(values), source_id_(source_id) {}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id, std::vector<KinematicsValue>&& values)
    : std::vector<KinematicsValue>(move(values)), source_id_(source_id) {}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<Isometry3<double>>;

}  // namespace geometry
}  // namespace drake
