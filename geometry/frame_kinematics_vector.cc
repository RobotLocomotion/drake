#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

using std::make_pair;
using std::move;

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id, const std::vector<FrameId>& ids)
    : source_id_(source_id), values_(0) {
  for (FrameId id : ids) {
    bool is_unique = values_.insert(make_pair(id, FlaggedValue())).second;
    if (!is_unique) {
      throw std::runtime_error(
          fmt::format("At least one frame id appears multiple times: {}", id));
    }
  }
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::clear() {
  ++version_;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::set_value(
    FrameId id, const KinematicsValue& value) {
  using std::to_string;

  if (values_.count(id) != 1) {
    throw std::runtime_error(
        "Trying to set a kinematics value for a frame id that does not belong "
            "to the kinematics vector: "  + to_string(id));
  }
  if (values_[id].version == version_) {
    throw std::runtime_error(
        "Trying to set kinematics value for the same id (" + to_string(id)
        + ") multiple times. Did you forget to call clear()?");
  }
  values_[id].version = version_;
  values_[id].value = value;
}

template <typename KinematicsValue>
const KinematicsValue& FrameKinematicsVector<KinematicsValue>::value(
    FrameId id) const {
  using std::to_string;
  if (values_.count(id) != 1) {
    throw std::runtime_error("Can't acquire value for id " + to_string(id) +
                             ". It is not part of the kinematics data id set.");
  }
  return values_.at(id).value;
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<Isometry3<double>>;
template class FrameKinematicsVector<Isometry3<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake
