#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateVectorInterface is an interface template for vector quantities within
/// the state of a System.  Both leaf Systems and composite Systems (Diagrams)
/// have state that satisfies StateVectorInterface.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateVectorInterface {
 public:
  virtual ~StateVectorInterface() {}

  /// Returns the number of elements in the vector.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual size_t get_size() const = 0;

  /// Returns the element at the given index in the vector. Throws
  /// std::runtime_error if the index is >= get_size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual T GetAtIndex(size_t index) const = 0;

  /// Returns the VectorInterface for the state, or nullptr if this state is
  /// not a leaf state (and thus does not have a contiguous vector form).
  /// The caller does not own the returned pointer. The returned pointer is
  /// guaranteed to be valid for so long as the StateVectorInterface that
  /// produced it is valid.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual const VectorInterface<T>* GetVector() const = 0;

  /// Replaces the state at the given index with the value. Throws
  /// std::runtime_error if the index is >= get_size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual void SetAtIndex(size_t index, const T& value) = 0;

  /// Replaces the entire state with the contents of value. Throws
  /// std::runtime_error if value is not a column vector with get_size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::VectorBlock<VectorX<T>>& value) = 0;

 protected:
  StateVectorInterface() {}

 private:
  // StateVectorInterface objects are neither copyable nor moveable.
  StateVectorInterface(const StateVectorInterface& other) = delete;
  StateVectorInterface& operator=(const StateVectorInterface& other) = delete;
  StateVectorInterface(StateVectorInterface&& other) = delete;
  StateVectorInterface& operator=(StateVectorInterface&& other) = delete;
};

/// SystemStateVector is a concrete class template that implements
/// StateVectorInterface in a convenient manner for leaf Systems, by owning
/// and wrapping a VectorInterface<T>.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class SystemStateVector : public StateVectorInterface<T> {
 public:
  explicit SystemStateVector(std::unique_ptr<VectorInterface<T>> vector)
      : vector_(std::move(vector)) {}

  size_t get_size() const override { return vector_->get_value().rows(); }

  T GetAtIndex(size_t index) const override {
    if (index >= get_size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state vector of size " +
                               std::to_string(get_size()));
    }
    return vector_->get_value()[index];
  }

  const VectorInterface<T>* GetVector() const override { return vector_.get(); }

  void SetAtIndex(size_t index, const T& value) override {
    if (index >= get_size()) {
      throw std::runtime_error("Index " + std::to_string(index) +
                               "out of bounds for state vector of size " +
                               std::to_string(get_size()));
    }
    vector_->get_mutable_value()[index] = value;
  }

  void SetFromVector(const Eigen::VectorBlock<VectorX<T>>& value) override {
    vector_->set_value(value);
  }

 private:
  // SystemStateVector objects are neither copyable nor moveable.
  SystemStateVector(const SystemStateVector& other) = delete;
  SystemStateVector& operator=(const SystemStateVector& other) = delete;
  SystemStateVector(SystemStateVector&& other) = delete;
  SystemStateVector& operator=(SystemStateVector&& other) = delete;

  std::unique_ptr<VectorInterface<T>> vector_;
};

/// The ContinuousState is a container for all the State variables that are
/// unique to continuous Systems, i.e. Systems that satisfy
/// ContinuousSystemInterface and have defined dynamics at all times.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
struct ContinuousState {
  /// Generalized coordinates representing System configuration, conventionally
  /// denoted `q`. These are second-order state variables.
  std::unique_ptr<SystemStateVector<T>> generalized_position;

  /// Generalized speeds representing System velocity. Conventionally denoted
  /// `v`. These are first-order state variables that the System can linearly
  /// map to time derivatives `qdot` of `q` above.
  std::unique_ptr<SystemStateVector<T>> generalized_velocities;

  /// Additional continuous, first-order state variables not representing
  /// multibody system motion.  Conventionally denoted `z`.
  std::unique_ptr<SystemStateVector<T>> other_continuous_state;
};

/// The State is a container for all the data comprising the complete state of
/// a particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
struct State {
  ContinuousState<T> continuous_state;
};

}  // namespace systems
}  // namesapce drake
