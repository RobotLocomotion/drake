#pragma once

#include <Eigen/Dense>

#include <stdexcept>

#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateSubvector is a concrete class template that implements
/// StateVectorInterface by providing a sliced view of a StateVectorInterface.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateSubvector : public StateVectorInterface<T> {
 public:
  /// Constructs a subvector of vector that consists of num_elements starting
  /// at first_element.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  StateSubvector(StateVectorInterface<T>* vector, size_t first_element,
                 size_t num_elements)
      : vector_(vector),
        first_element_(first_element),
        num_elements_(num_elements) {
    if (vector_ == nullptr) {
      throw std::runtime_error(
          "Cannot create StateSubvector of a nullptr vector.");
    }
    if (first_element_ + num_elements_ > vector_->size()) {
      throw std::runtime_error("StateSubvector out of bounds.");
    }
  }

  /// Constructs an empty subvector.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  explicit StateSubvector(StateVectorInterface<T>* vector)
      : StateSubvector(vector, 0, 0) {}

  virtual size_t size() const { return num_elements_; }

  virtual const T GetAtIndex(size_t index) const {
    if (index >= size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state subvector of size " +
                               std::to_string(size()));
    }
    return vector_->GetAtIndex(first_element_ + index);
  }

  virtual void SetAtIndex(size_t index, const T& value) {
    if (index >= size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state subvector of size " +
                               std::to_string(size()));
    }
    vector_->SetAtIndex(first_element_ + index, value);
  }

  virtual void SetFromVector(const Eigen::VectorBlock<VectorX<T>>& value) {
    for (int i = 0; i < value.rows(); ++i) {
      SetAtIndex(i, value[i]);
    }
  }

 private:
  // StateSubvector objects are neither copyable nor moveable.
  StateSubvector(const StateSubvector& other) = delete;
  StateSubvector& operator=(const StateSubvector& other) = delete;
  StateSubvector(StateSubvector&& other) = delete;
  StateSubvector& operator=(StateSubvector&& other) = delete;

  StateVectorInterface<T>* vector_ = nullptr;
  size_t first_element_ = 0;
  size_t num_elements_ = 0;
};

}  // namespace systems
}  // namespace drake
