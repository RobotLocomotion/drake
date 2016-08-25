#pragma once

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

/// BasicStateVector is a concrete class template that implements
/// StateVector in a convenient manner for LeafSystem blocks,
/// by owning and wrapping a VectorBase<T>.
///
/// It will often be convenient to inherit from BasicStateVector, and add
/// additional semantics specific to the LeafSystem. Such child classes must
/// override DoClone with an implementation that returns their concrete type.
///
/// TODO(david-german-tri): Collapse BasicStateVector into BasicVector.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class BasicStateVector : public BasicVector<T> {
 public:
  /// Constructs a BasicStateVector that owns a generic BasicVector of the
  /// specified @p size.
  explicit BasicStateVector(int size)
      : BasicVector<T>(size) {}

  // Constructs a BasicStateVector with the specified @p data.
  explicit BasicStateVector(const VectorX<T>& data)
      : BasicVector<T>(data) {}

  static std::unique_ptr<BasicStateVector<T>> Make(
      const std::initializer_list<T>& data) {
    auto vec = std::make_unique<BasicStateVector<T>>(data.size());
    int i = 0;
    for (const T& datum : data) {
      vec->SetAtIndex(i++, datum);
    }
    return vec;
  }

  // Assignment of BasicStateVectors could change size, so we forbid it.
  BasicStateVector& operator=(const BasicStateVector& other) = delete;

  // BasicStateVector objects are not moveable.
  BasicStateVector(BasicStateVector&& other) = delete;
  BasicStateVector& operator=(BasicStateVector&& other) = delete;

 protected:
  explicit BasicStateVector(const BasicStateVector& other)
      : BasicVector<T>(other) {}

  BasicStateVector<T>* DoClone() const override {
    return new BasicStateVector<T>(*this);
  }
};

}  // namespace systems
}  // namespace drake
