#include "drake/geometry/frame_kinematics_vector.h"

#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

namespace {
template <typename T>
void InitializeKinematicsValue(math::RigidTransform<T>* value) {
  value->SetIdentity();
}
}  // namespace

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector() {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>::FrameKinematicsVector(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  values_.insert(init.begin(), init.end());
  size_ = init.size();
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename KinematicsValue>
FrameKinematicsVector<KinematicsValue>&
FrameKinematicsVector<KinematicsValue>::operator=(
    std::initializer_list<std::pair<const FrameId, KinematicsValue>> init) {
  // N.B. We can't use unordered_map::insert in our operator= implementation
  // because it does not overwrite pre-existing keys.  (Our clear() doesn't
  // remove the keys, it only nulls the values.)
  clear();
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
  return *this;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::clear() {
  for (auto& item : values_) {
    item.second = std::nullopt;
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
    const std::optional<KinematicsValue>& map_value = iter->second;
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
  DRAKE_ASSERT(static_cast<int>(result.size()) == size_);
  return result;
}

template <typename KinematicsValue>
void FrameKinematicsVector<KinematicsValue>::CheckInvariants() const {
  int num_nonnull = 0;
  for (const auto& item : values_) {
    if (item.second.has_value()) {
      ++num_nonnull;
    }
  }
  DRAKE_DEMAND(num_nonnull == size_);
}

// Explicitly instantiates on the most common scalar types.
template class FrameKinematicsVector<math::RigidTransform<double>>;
template class FrameKinematicsVector<math::RigidTransform<AutoDiffXd>>;
template class FrameKinematicsVector<
    math::RigidTransform<symbolic::Expression>>;

}  // namespace geometry
}  // namespace drake
