#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// AbstractValues is a container for non-numerical state and parameters.
/// It may or may not own the underlying data, and therefore is suitable
/// for both leaf Systems and diagrams.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
class AbstractValues {
 public:
  // AbstractState is not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractValues)

  // Constructs an empty AbstractValues.
  AbstractValues();

  /// Constructs an AbstractValues that owns the underlying data.
  explicit AbstractValues(std::vector<std::unique_ptr<AbstractValue>>&& data);

  /// Constructs an AbstractValues that does not own the underlying data.
  explicit AbstractValues(const std::vector<AbstractValue*>& data);

  /// Constructs an AbstractValues that owns a single @p datum.
  explicit AbstractValues(std::unique_ptr<AbstractValue> datum);

  virtual ~AbstractValues();

  /// Returns the number of elements of AbstractValues.
  int size() const;

  /// Returns the element of AbstractValues at the given @p index, or aborts if
  /// the index is out-of-bounds.
  const AbstractValue& get_value(int index) const;

  /// Returns the element of AbstractValues at the given @p index, or aborts if
  /// the index is out-of-bounds.
  AbstractValue& get_mutable_value(int index);

  /// Copies all of the AbstractValues in @p other into this. Asserts if the
  /// two are not equal in size. Throws if any of the elements are of
  /// incompatible type.
  void CopyFrom(const AbstractValues& other);

  /// Returns a deep copy of all the data in this AbstractValues. The clone
  /// will own its own data. This is true regardless of whether the data being
  /// cloned had ownership of its data or not.
  std::unique_ptr<AbstractValues> Clone() const;

 private:
  // Pointers to the data. If the data is owned, these pointers are equal to
  // the pointers in owned_data_.
  std::vector<AbstractValue*> data_;
  // Owned pointers to the data. The only purpose of these pointers is to
  // maintain ownership. They may be populated at construction time, and are
  // never accessed thereafter.
  std::vector<std::unique_ptr<AbstractValue>> owned_data_;
};

}  // namespace systems
}  // namespace drake
