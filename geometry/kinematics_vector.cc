#include "drake/geometry/kinematics_vector.h"

#include <algorithm>

#include "absl/container/flat_hash_map.h"

#include "drake/common/autodiff.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

template <class Id, class KinematicsValue>
class __attribute__((visibility("hidden")))
KinematicsVector<Id, KinematicsValue>::Impl {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Impl);

  Impl() = default;
  ~Impl() = default;

  void clear() {
    /* We don't use map_.clear() to ensure the memory isn't released. */
    map_.erase(map_.begin(), map_.end());
  }

  void set_value(Id id, const KinematicsValue& value) {
    map_.insert_or_assign(id, value);
  }

  int size() const { return map_.size(); }

  const KinematicsValue& value(const Id& id) const {
    const auto& it = map_.find(id);
    if (it != map_.end()) return it->second;

    throw std::runtime_error(
        fmt::format("No such {}: {}.",
                    NiceTypeName::RemoveNamespaces(NiceTypeName::Get<Id>()),
                    to_string(id)));
  }

  bool has_id(const Id& id) const { return map_.contains(id); }

  std::vector<Id> GetAllIds() const {
    std::vector<Id> results(map_.size());
    int index = 0;
    for (const auto& it : map_) {
      results[index++] = it.first;
    }
    // Sort the results to ensure it's deterministic.
    std::sort(results.begin(), results.end());
    return results;
  }

 private:
  absl::flat_hash_map<Id, KinematicsValue> map_;
};

template <typename Id, typename KinematicsValue>
typename KinematicsVector<Id, KinematicsValue>::Impl&
KinematicsVector<Id, KinematicsValue>::impl() {
  DRAKE_ASSERT(pimpl_ != nullptr);
  return *static_cast<Impl*>(pimpl_.get());
}

template <typename Id, typename KinematicsValue>
const typename KinematicsVector<Id, KinematicsValue>::Impl&
KinematicsVector<Id, KinematicsValue>::impl() const {
  DRAKE_ASSERT(pimpl_ != nullptr);
  return *static_cast<Impl*>(pimpl_.get());
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector()
    : pimpl_(std::make_shared<Impl>()) {}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init)
    : KinematicsVector() {
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init) {
  clear();
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    const KinematicsVector<Id, KinematicsValue>& other)
    : KinematicsVector() {
  *this = other;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    KinematicsVector<Id, KinematicsValue>&& other)
    : KinematicsVector() {
  *this = std::move(other);
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    const KinematicsVector<Id, KinematicsValue>& other) {
  pimpl_ = std::make_shared<Impl>(other.impl());
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    KinematicsVector<Id, KinematicsValue>&& other) {
  std::swap(pimpl_, other.pimpl_);
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::~KinematicsVector() = default;

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::clear() {
  impl().clear();
}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::set_value(
    Id id, const KinematicsValue& value) {
  impl().set_value(id, value);
}

template <typename Id, typename KinematicsValue>
int KinematicsVector<Id, KinematicsValue>::size() const {
  return impl().size();
}

template <typename Id, typename KinematicsValue>
const KinematicsValue& KinematicsVector<Id, KinematicsValue>::value(
    Id id) const {
  return impl().value(id);
}

template <typename Id, typename KinematicsValue>
bool KinematicsVector<Id, KinematicsValue>::has_id(Id id) const {
  return impl().has_id(id);
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::frame_ids() const {
  return GetAllIds();
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::GetAllIds() const {
  return impl().GetAllIds();
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
