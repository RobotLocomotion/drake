#include "drake/geometry/kinematics_vector.h"

#include "absl/container/flat_hash_map.h"

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

template <class Id, class KinematicsValue>
class KinematicsVector<Id, KinematicsValue>::Impl {
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
    if (!has_id(id)) {
      throw std::runtime_error("No such Id " + to_string(id) + ".");
    }
    return map_.at(id);
  }

  bool has_id(const Id& id) const { return map_.find(id) != map_.end(); }

  std::vector<Id> ids() const {
    std::vector<Id> results(map_.size());
    int index = 0;
    for (const auto& it : map_) {
      results[index++] = it.first;
    }
    return results;
  }

 private:
  absl::flat_hash_map<Id, KinematicsValue> map_;
};

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector()
    : pimpl_(std::make_unique<Impl>()) {}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init)
    : pimpl_(std::make_unique<Impl>()) {
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
    : pimpl_(std::make_unique<Impl>()) {
  *this = other;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    KinematicsVector<Id, KinematicsValue>&& other)
    : pimpl_(std::make_unique<Impl>()) {
  *this = other;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    const KinematicsVector<Id, KinematicsValue>& other) {
  this->pimpl_ = std::make_unique<Impl>(*other.pimpl_);
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    KinematicsVector<Id, KinematicsValue>&& other) {
  this->pimpl_.swap(other.pimpl_);
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::~KinematicsVector() {}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::clear() {
  pimpl_->clear();
}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::set_value(
    Id id, const KinematicsValue& value) {
  pimpl_->set_value(id, value);
}

template <typename Id, typename KinematicsValue>
int KinematicsVector<Id, KinematicsValue>::size() const {
  return pimpl_->size();
}

template <typename Id, typename KinematicsValue>
const KinematicsValue& KinematicsVector<Id, KinematicsValue>::value(
    Id id) const {
  return pimpl_->value(id);
}

template <typename Id, typename KinematicsValue>
bool KinematicsVector<Id, KinematicsValue>::has_id(Id id) const {
  return pimpl_->has_id(id);
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::frame_ids() const {
  return ids();
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::ids() const {
  return pimpl_->ids();
}

// Explicitly instantiates on the most common scalar types.
template class KinematicsVector<FrameId, math::RigidTransform<double>>;
template class KinematicsVector<FrameId, math::RigidTransform<AutoDiffXd>>;
template class KinematicsVector<FrameId,
                                math::RigidTransform<symbolic::Expression>>;
template class KinematicsVector<GeometryId, VectorX<double>>;
template class KinematicsVector<GeometryId, VectorX<AutoDiffXd>>;
template class KinematicsVector<GeometryId, VectorX<symbolic::Expression>>;

}  // namespace geometry
}  // namespace drake
