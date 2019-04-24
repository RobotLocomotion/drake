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
  optional<KinematicsValue> default_value{KinematicsValue{}};
  InitializeKinematicsValue(&default_value.value());
  for (FrameId id : ids) {
    bool is_unique = values_.emplace(id, default_value).second;
    if (!is_unique) {
      throw std::runtime_error(
          fmt::format("At least one frame id appears multiple times: {}", id));
    }
    ++size_;
  }
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  values_.insert(init.begin(), init.end());
  size_ = init.size();
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>&
FrameKinematicsVector<KinematicsValue>::operator=(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  clear();
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
  return *this;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::clear() {
  for (auto& item : values_) {
    item.second = nullopt;
  }
  size_ = 0;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::set_value(
    FrameId id, const KinematicsValue& value) {
  auto& map_value = values_[id];
  if (!map_value.has_value()) { ++size_; }
  map_value = value;
}

template <typename KinematicsValue>
const KinematicsValue& FrameKinematicsVector<KinematicsValue>::value(
    FrameId id) const {
  using std::to_string;
  auto iter = values_.find(id);
  if (iter != values_.end()) {
    const optional<KinematicsValue>& map_value = iter->second;
    if (map_value.has_value()) {
      return *map_value;
    }
  }
  throw std::runtime_error("No such FrameId " + to_string(id) + ".");
}

template <typename KinematicsValue>
bool FrameKinematicsVector<KinematicsValue>::has_id(FrameId id) const {
  auto iter = values_.find(id);
  return (iter != values_.end()) && iter->second.has_value();
}

template <typename KinematicsValue>
std::vector<FrameId>
FrameKinematicsVector<KinematicsValue>::frame_ids() const {
  std::vector<FrameId> result;
  result.reserve(size_);
  for (const auto& item : values_) {
    if (item.second.has_value()) {
      result.emplace_back(item.first);
    }
  }
  return result;
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<Isometry3<double>>;
template class FrameKinematicsVector<Isometry3<AutoDiffXd>>;
template class FrameKinematicsVector<Isometry3<symbolic::Expression>>;

}  // namespace geometry
}  // namespace drake
