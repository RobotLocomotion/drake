#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The DiscreteState is a container for the numerical state values that are
/// updated discontinuously on time or state based triggers. It may own its
/// underlying data, for use with leaf Systems, or not, for use with Diagrams.
///
/// DiscreteState is an ordered collection of vectors xd = [xd0, xd1...].
/// Requesting a specific index from this collection is the most granular way
/// to retrieve discrete state from the Context, and thus is the unit of
/// cache invalidation. System authors are encouraged to partition their
/// DiscreteState such that each cacheable computation within the System may
/// depend on only the elements of DiscreteState that it needs.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DiscreteState {
 public:
  /// Constructs an empty discrete state.
  DiscreteState() {}

  /// Constructs a discrete state that does not own the underlying @p data.
  /// The data must outlive this DiscreteState.
  explicit DiscreteState(const std::vector<BasicVector<T>*>& data)
      : data_(data) {}

  /// Constructs a discrete state that owns the underlying @p data.
  explicit DiscreteState(std::vector<std::unique_ptr<BasicVector<T>>>&& data)
      : owned_data_(std::move(data)) {
    // Initialize the unowned pointers.
    for (auto& datum : owned_data_) {
      data_.push_back(datum.get());
    }
  }

  /// Constructs a discrete state that owns a single @p datum vector.
  explicit DiscreteState(std::unique_ptr<BasicVector<T>> datum) {
    data_.push_back(datum.get());
    owned_data_.push_back(std::move(datum));
  }

  virtual ~DiscreteState() {}

  int size() const {
    return static_cast<int>(data_.size());
  }

  const std::vector<BasicVector<T>*>& get_data() const {
    return data_;
  }

  const BasicVector<T>* get_discrete_state(int index) const {
    DRAKE_ASSERT(index >= 0 && index < size());
    return data_[index];
  }

  BasicVector<T>* get_mutable_discrete_state(int index) {
    DRAKE_ASSERT(index >= 0 && index < size());
    return data_[index];
  }

  /// Writes the values from @p other into this DiscreteState, possibly
  /// writing through to unowned data. Asserts if the dimensions don't match.
  void CopyFrom(const DiscreteState<T>& other) {
    SetFromGeneric(other);
  }

  /// Resets the values in this DiscreteState from the values in @p other,
  /// possibly writing through to unowned data. Asserts if the dimensions don't
  /// match.
  void SetFrom(const DiscreteState<double>& other) {
    SetFromGeneric(other);
  }

  /// Returns a deep copy of all the data in this DiscreteState. The clone
  /// will own its own data. This is true regardless of whether the state being
  /// cloned had ownership of its data or not.
  std::unique_ptr<DiscreteState> Clone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
    cloned_data.reserve(data_.size());
    for (const BasicVector<T>* datum : data_) {
      cloned_data.push_back(datum->Clone());
    }
    return std::make_unique<DiscreteState>(std::move(cloned_data));
  }

  // DiscreteState is not copyable or moveable.
  DiscreteState(const DiscreteState& other) = delete;
  DiscreteState& operator=(const DiscreteState& other) = delete;
  DiscreteState(DiscreteState&& other) = delete;
  DiscreteState& operator=(DiscreteState&& other) = delete;

 private:
  // Pointers to the data comprising the state. If the data is owned, these
  // pointers are equal to the pointers in owned_data_.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the state. The only purpose of these
  // pointers is to maintain ownership. They may be populated at construction
  // time, and are never accessed thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;

  template <typename U>
  void SetFromGeneric(const DiscreteState<U>& other) {
    DRAKE_ASSERT(size() == other.size());
    for (int i = 0; i < size(); i++) {
      DRAKE_ASSERT(other.get_discrete_state(i) != nullptr);
      DRAKE_ASSERT(data_[i] != nullptr);
      data_[i]->set_value(
          other.get_discrete_state(i)->get_value().template cast<T>());
    }
  }
};

}  // namespace systems
}  // namespace drake
