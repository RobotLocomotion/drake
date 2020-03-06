#pragma once

#include <cstdint>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// Subvector is a concrete class template that implements
/// VectorBase by providing a sliced view of a VectorBase.
///
/// @tparam_default_scalar
template <typename T>
class Subvector : public VectorBase<T> {
 public:
  // Subvector objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subvector)

  /// Constructs a subvector of vector that consists of num_elements starting
  /// at first_element.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  Subvector(VectorBase<T>* vector, int first_element, int num_elements)
      : vector_(vector),
        first_element_(first_element),
        num_elements_(num_elements) {
    if (vector_ == nullptr) {
      throw std::logic_error("Cannot create Subvector of a nullptr vector.");
    }
    if (first_element_ + num_elements_ > vector_->size()) {
      throw std::out_of_range("Subvector out of bounds.");
    }
  }

  /// Constructs an empty subvector.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  explicit Subvector(VectorBase<T>* vector) : Subvector(vector, 0, 0) {}

  int size() const override { return num_elements_; }

 protected:
  const T& DoGetAtIndex(int index) const override {
    DRAKE_THROW_UNLESS(index < size());
    return (*vector_)[first_element_ + index];
  }

  T& DoGetAtIndex(int index) override {
    DRAKE_THROW_UNLESS(index < size());
    return (*vector_)[first_element_ + index];
  }

 private:
  VectorBase<T>* vector_{nullptr};
  int first_element_{0};
  int num_elements_{0};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Subvector)
