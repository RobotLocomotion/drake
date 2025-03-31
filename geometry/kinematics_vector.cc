#include "drake/geometry/kinematics_vector.h"

#include <functional>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace geometry {

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector() {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init) {
  values_.insert(init.begin(), init.end());
  size_ = init.size();
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init) {
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

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::~KinematicsVector() = default;

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::clear() {
  for (auto& item : values_) {
    item.second = std::nullopt;
  }
  size_ = 0;
}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::set_value(
    Id id, const KinematicsValue& value) {
  auto& map_value = values_[id];
  if (!map_value.has_value()) {
    ++size_;
  }
  map_value = value;
}

template <typename Id, typename KinematicsValue>
const KinematicsValue& KinematicsVector<Id, KinematicsValue>::value(
    Id id) const {
  using std::to_string;
  auto iter = values_.find(id);
  if (iter != values_.end()) {
    const std::optional<KinematicsValue>& map_value = iter->second;
    if (map_value.has_value()) {
      return *map_value;
    }
  }
  throw std::runtime_error(fmt::format(
      "No such {}: {}.",
      NiceTypeName::RemoveNamespaces(NiceTypeName::Get<Id>()), to_string(id)));
}

template <typename Id, typename KinematicsValue>
bool KinematicsVector<Id, KinematicsValue>::has_id(Id id) const {
  auto iter = values_.find(id);
  return (iter != values_.end()) && iter->second.has_value();
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::ids() const {
  std::vector<Id> result;
  result.reserve(size_);
  for (const auto& item : values_) {
    if (item.second.has_value()) {
      result.emplace_back(item.first);
    }
  }
  DRAKE_ASSERT(static_cast<int>(result.size()) == size_);
  return result;
}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::CheckInvariants() const {
  int num_nonnull = 0;
  for (const auto& item : values_) {
    if (item.second.has_value()) {
      ++num_nonnull;
    }
  }
  DRAKE_DEMAND(num_nonnull == size_);
}

namespace {
// Tests for the supported kinematics value types for whether they have NaN
// values.

template <typename Derived>
bool KinematicsIsFinite(const Eigen::MatrixBase<Derived>& m) {
  using std::isfinite;
  return all_of(m, [](const auto& t) {
    return isfinite(t);
  });
}

template <typename T>
bool KinematicsIsFinite(const math::RigidTransform<T>& X) {
  return KinematicsIsFinite(X.translation()) &&
         KinematicsIsFinite(X.rotation().matrix());
}

}  // namespace

template <typename Id, typename KinematicsValue>
bool KinematicsVector<Id, KinematicsValue>::IsFinite() const {
  // For T = Expression, we simply don't bother checking.
  if constexpr (std::is_same_v<KinematicsValue,
                               math::RigidTransform<symbolic::Expression>> ||
                std::is_same_v<KinematicsValue,
                               VectorX<symbolic::Expression>>) {
    return true;
  } else {
    for (const auto& [_, maybe_value] : values_) {
      if (!maybe_value.has_value()) continue;
      if (!KinematicsIsFinite(*maybe_value)) return false;
    }
    return true;
  }
}

// Explicitly instantiates on the most common scalar types.
using math::RigidTransform;
using symbolic::Expression;
template class KinematicsVector<FrameId, RigidTransform<double>>;
template class KinematicsVector<FrameId, RigidTransform<AutoDiffXd>>;
template class KinematicsVector<FrameId, RigidTransform<Expression>>;
template class KinematicsVector<GeometryId, VectorX<double>>;
template class KinematicsVector<GeometryId, VectorX<AutoDiffXd>>;
template class KinematicsVector<GeometryId, VectorX<Expression>>;

}  // namespace geometry
}  // namespace drake
