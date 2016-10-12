#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The DifferenceState is a container for the numerical state values that are
/// updated discontinuously on time or state based triggers. It may or may not
/// own the underlying data, and therefore is suitable for both leaf Systems
/// and diagrams.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class DifferenceState {
 public:
  /// Constructs an empty difference state.
  DifferenceState() {}

  /// Constructs a difference state that does not own the underlying @p data.
  /// The data must outlive this DifferenceState.
  explicit DifferenceState(std::vector<BasicVector<T>*> data)
      : data_(data) {}

  /// Constructs a difference state that owns the underlying @p data.
  explicit DifferenceState(std::vector<std::unique_ptr<BasicVector<T>>> data)
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

  /// Returns a deep copy of all the data in this DifferenceState. Even if this
  /// state's data is unowned, the clone's will be owned.
  std::unique_ptr<DifferenceState> Clone() const {
    std::vector<std::unique_ptr<BasicVector<T>>> cloned_data;
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
  // Pointers to the data comprising the state.
  std::vector<BasicVector<T>*> data_;
  // Owned pointers to the data comprising the state.
  // Possibly written at construction, and not read thereafter.
  std::vector<std::unique_ptr<BasicVector<T>>> owned_data_;
};

}  // namespace systems
}  // namespace drake
