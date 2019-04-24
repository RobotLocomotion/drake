#include "drake/geometry/frame_kinematics_vector.h"

#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace geometry {

namespace {
template <typename T>
void InitializeKinematicsValue(Isometry3<T>* value) {
  value->setIdentity();
}
}  // namespace

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector()
    : FrameKinematicsVector<KinematicsValue>({}, {}) {}
#pragma GCC diagnostic pop

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    SourceId source_id, const std::vector<FrameId>& ids)
    : source_id_(source_id), values_(0) {
  KinematicsValue default_value;
  InitializeKinematicsValue(&default_value);
  for (FrameId id : ids) {
    bool is_unique = values_.emplace(id, default_value).second;
    if (!is_unique) {
      throw std::runtime_error(
          fmt::format("At least one frame id appears multiple times: {}", id));
    }
  }
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  values_ = init;
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>&
FrameKinematicsVector<KinematicsValue>::operator=(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  values_ = init;
  return *this;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::clear() {
  values_.clear();
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::set_value(
    FrameId id, const KinematicsValue& value) {
  values_[id] = value;
}

template <typename KinematicsValue>
const KinematicsValue& FrameKinematicsVector<KinematicsValue>::value(
    FrameId id) const {
  using std::to_string;
  auto iter = values_.find(id);
  if (iter == values_.end()) {
    throw std::runtime_error("No such FrameId " + to_string(id) + ".");
  }
  return iter->second;
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<Isometry3<double>>;
template class FrameKinematicsVector<Isometry3<AutoDiffXd>>;
template class FrameKinematicsVector<Isometry3<symbolic::Expression>>;

}  // namespace geometry
}  // namespace drake
