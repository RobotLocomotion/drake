#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The DifferenceState is a container for the numerical state values that are
/// updated discontinuously on time or state based triggers. It may own its
/// underlying data, for use with leaf Systems, or not, for use with Diagrams.
///
/// DifferenceState is an ordered collection of vectors xd = [xd0, xd1...].
/// Requesting a specific index from this collection is the most granular way
/// to retrieve difference state from the Context, and thus is the unit of
/// cache invalidation. System authors are encouraged to partition their
/// DifferenceState such that each cacheable computation within the System may
/// depend on only the elements of DifferenceState that it needs.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DifferenceState {
 public:
  /// Constructs an empty difference state.
  DifferenceState() {}

  /// Constructs a difference state that does not own the underlying @p data.
  /// The data must outlive this DifferenceState.
  explicit DifferenceState(const std::vector<BasicVector<T>*>& data)
      : data_(data) {}

  /// Constructs a difference state that owns the underlying @p data.
  explicit DifferenceState(std::vector<std::unique_ptr<BasicVector<T>>>&& data)
      : owned_data_(std::move(data)) {
    // Initialize the unowned pointers.
    for (auto& datum : owned_data_) {
      data_.push_back(datum.get());
    }
  }

  virtual ~DifferenceState() {}

  int size() const {
    return static_cast<int>(data_.size());
  }

  const BasicVector<T>* get_difference_state(int index) const {
    DRAKE_ASSERT(index >= 0 && index < size());
    return data_[index];
  }

  BasicVector<T>* get_mutable_difference_state(int index) {
    DRAKE_ASSERT(index >= 0 && index < size());
    return data_[index];
  }

  /// Writes the values from @p other into this DifferenceState, possibly
  /// writing through to unowned data. Aborts if the dimensions don't match.
  void CopyFrom(const DifferenceState<T>& other) {
    DRAKE_DEMAND(size() == other.size());
    for (int i = 0; i < size(); i++) {
      DRAKE_DEMAND(other.get_difference_state(i) != nullptr);
      DRAKE_DEMAND(data_[i] != nullptr);
      data_[i]->set_value(other.get_difference_state(i)->get_value());
    }
  }

  /// Returns a deep copy of all the data in this DifferenceState. The clone
  /// will own its own data. This is true regardless of whether the state being
  /// cloned had ownership of its data or not.
  std::unique_ptr<DifferenceState> Clone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
    cloned_data.reserve(data_.size());
    for (const BasicVector<T>* datum : data_) {
      cloned_data.push_back(datum->Clone());
    }
    return std::make_unique<DifferenceState>(std::move(cloned_data));
  }

  // DifferenceState is not copyable or moveable.
  DifferenceState(const DifferenceState& other) = delete;
  DifferenceState& operator=(const DifferenceState& other) = delete;
  DifferenceState(DifferenceState&& other) = delete;
  DifferenceState& operator=(DifferenceState&& other) = delete;

 private:
  // Pointers to the data comprising the state. If the data is owned, these
  // pointers are equal to the pointers in owned_data_.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the state. The only purpose of these
  // pointers is to maintain ownership. They may be populated at construction
  // time, and are never accessed thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;
};

}  // namespace systems
}  // namespace drake
