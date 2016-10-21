#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The ModalState is a container for the non-numerical state values that are
/// updated discontinuously on time or state based triggers. It may or may not
/// own the underlying data, and therefore is suitable for both leaf Systems
/// and diagrams.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
class DRAKE_EXPORT ModalState {
 public:
  // Constructs an empty modal state.
  ModalState();

  /// Constructs a modal state that owns the underlying data.
  explicit ModalState(std::vector<std::unique_ptr<AbstractValue>>&& data);

  /// Constructs a modal state that does not own the underlying data.
  explicit ModalState(const std::vector<AbstractValue*>& data);

  virtual ~ModalState();

  /// Returns the number of elements of modal state.
  int size() const;

  /// Returns the element of modal state at the given @p index, or aborts if
  /// the index is out-of-bounds.
  const AbstractValue& get_modal_state(int index) const;

  /// Returns the element of modal state at the given @p index, or aborts if
  /// the index is out-of-bounds.
  AbstractValue& get_mutable_modal_state(int index);

  /// Copies all of the modal state in @p other into this state. Aborts if the
  /// two states are not equal in size. Throws if any of the elements are of
  /// incompatible type.
  void CopyFrom(const ModalState& other);

  /// Returns a deep copy of all the data in this DifferenceState. The clone
  /// will own its own data. This is true regardless of whether the state being
  /// cloned had ownership of its data or not.
  std::unique_ptr<ModalState> Clone() const;

  // ModalState is not copyable or moveable.
  ModalState(const ModalState& other) = delete;
  ModalState& operator=(const ModalState& other) = delete;
  ModalState(ModalState&& other) = delete;
  ModalState& operator=(ModalState&& other) = delete;

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
