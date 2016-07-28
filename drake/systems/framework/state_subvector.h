#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <stdexcept>

#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateSubvector is a concrete class template that implements
/// StateVector by providing a sliced view of a StateVector.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateSubvector : public StateVector<T> {
 public:
  /// Constructs a subvector of vector that consists of num_elements starting
  /// at first_element.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  StateSubvector(StateVector<T>* vector, int first_element,
                 int num_elements)
      : vector_(vector),
        first_element_(first_element),
        num_elements_(num_elements) {
    if (vector_ == nullptr) {
      throw std::logic_error(
          "Cannot create StateSubvector of a nullptr vector.");
    }
    if (first_element_ + num_elements_ > vector_->size()) {
      throw std::out_of_range("StateSubvector out of bounds.");
    }
  }

  /// Constructs an empty subvector.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  explicit StateSubvector(StateVector<T>* vector)
      : StateSubvector(vector, 0, 0) {}

  int size() const override { return num_elements_; }

  const T GetAtIndex(int index) const override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state subvector of size " +
                              std::to_string(size()));
    }
    return vector_->GetAtIndex(first_element_ + index);
  }

  void SetAtIndex(int index, const T& value) override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state subvector of size " +
                              std::to_string(size()));
    }
    vector_->SetAtIndex(first_element_ + index, value);
  }

 private:
  // StateSubvector objects are neither copyable nor moveable.
  StateSubvector(const StateSubvector& other) = delete;
  StateSubvector& operator=(const StateSubvector& other) = delete;
  StateSubvector(StateSubvector&& other) = delete;
  StateSubvector& operator=(StateSubvector&& other) = delete;

  StateVector<T>* vector_{nullptr};
  int first_element_{0};
  int num_elements_{0};
};

}  // namespace systems
}  // namespace drake
