#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace detail {

/// Represents models for a sequence of AbstractValues (usually a sequence of
/// either input or output ports).  The models are the "prototype" design
/// pattern (https://en.wikipedia.org/wiki/Prototype_pattern).  When creating
/// elements in new Context, these models' values are cloned to establish the
/// subtype and default values of the, e.g. input port.
///
/// Conceptually, the %ModelValues form an infinite sequence.  The value at a
/// given index is allowed to be absent, in which case the CloneModel method
/// will return nullptr.  Adding new values must be monotonic; new values must
/// use strictly larger indices; the model value for a given index cannot be
/// reset.
class ModelValues {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelValues);
  ModelValues() = default;

  /// Returns one greater than the largest index passed to AddModel or
  /// AddVectorModel.
  int size() const;

  /// Sets @p model_value to be the model value for @p index.
  ///
  /// @pre index >= size()
  void AddModel(int index, std::unique_ptr<AbstractValue> model_value);

  /// Wraps @p model_vector in a VectorValue<T> and sets that to be the model
  /// value for @p index.
  ///
  /// @pre index >= size()
  template <typename T>
  void AddVectorModel(int index, std::unique_ptr<BasicVector<T>> model_vector);

  /// Returns a clone of the model value at @p index, which may be nullptr.
  std::unique_ptr<AbstractValue> CloneModel(int index) const;

  /// Returns a clone of the vector within the model value at @p index, which
  /// may be nullptr.
  ///
  /// @throw exception if the index has a model but the model's type does not
  /// match the given @p T
  template <typename T>
  std::unique_ptr<BasicVector<T>> CloneVectorModel(int index) const;

 private:
  // Elements here are allowed to be nullptr.
  std::vector<std::unique_ptr<const AbstractValue>> values_;
};

template <typename T>
void ModelValues::AddVectorModel(
    int index, std::unique_ptr<BasicVector<T>> model_vector) {
  AddModel(index, std::make_unique<VectorValue<T>>(std::move(model_vector)));
}

template <typename T>
std::unique_ptr<BasicVector<T>>
ModelValues::CloneVectorModel(int index) const {
  std::unique_ptr<AbstractValue> abstract_result = CloneModel(index);
  if (abstract_result.get() == nullptr) {
    return nullptr;
  }
  auto vector = abstract_result->GetMutableValueOrThrow<BasicVector<T>*>();
  if (vector == nullptr) {
    return nullptr;
  }
  return vector->Clone();
}

}  // namespace detail
}  // namespace systems
}  // namespace drake
