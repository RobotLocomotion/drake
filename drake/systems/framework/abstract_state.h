#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The AbstractState is a container for the non-numerical state values that are
/// updated discontinuously on time or state based triggers. It may or may not
/// own the underlying data, and therefore is suitable for both leaf Systems
/// and diagrams.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
class AbstractState {
 public:
  // Constructs an empty abstract state.
  AbstractState();

  /// Constructs a abstract state that owns the underlying data.
  explicit AbstractState(std::vector<std::unique_ptr<AbstractValue>>&& data);

  /// Constructs a abstract state that does not own the underlying data.
  explicit AbstractState(const std::vector<AbstractValue*>& data);

  virtual ~AbstractState();

  /// Returns the number of elements of abstract state.
  int size() const;

  /// Returns the element of abstract state at the given @p index, or aborts if
  /// the index is out-of-bounds.
  const AbstractValue& get_abstract_state(int index) const;

  /// Returns the element of abstract state at the given @p index, or aborts if
  /// the index is out-of-bounds.
  AbstractValue& get_mutable_abstract_state(int index);

  /// Copies all of the abstract state in @p other into this state. Asserts that
  /// two states are not equal in size. Throws if any of the elements are of
  /// incompatible type.
  void CopyFrom(const AbstractState& other);

  /// Returns a deep copy of all the data in this AbstractState. The clone
  /// will own its own data. This is true regardless of whether the state being
  /// cloned had ownership of its data or not.
  std::unique_ptr<AbstractState> Clone() const;

  // AbstractState is not copyable or moveable.
  AbstractState(const AbstractState& other) = delete;
  AbstractState& operator=(const AbstractState& other) = delete;
  AbstractState(AbstractState&& other) = delete;
  AbstractState& operator=(AbstractState&& other) = delete;

 private:
  // Pointers to the data comprising the state. If the data is owned, these
  // pointers are equal to the pointers in owned_data_.
  std::vector<AbstractValue*> data_;
  // Owned pointers to the data comprising the state. The only purpose of these
  // pointers is to maintain ownership. They may be populated at construction
  // time, and are never accessed thereafter.
  std::vector<std::unique_ptr<AbstractValue>> owned_data_;
};

}  // namespace systems
}  // namespace drake
