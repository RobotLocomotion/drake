#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {

/// LeafStateVector is an abstract class template that implements
/// StateVector for leaf Systems, i.e. Systems that are not Diagrams.
///
/// It introduces the additional Non-Virtual Interface method Clone(), with
/// corresponding virtual implementation DoClone().  Children of LeafStateVector
/// should override DoClone to return their concrete type.
template <typename T>
class LeafStateVector : public StateVector<T> {
 public:
  ~LeafStateVector() override {}

  /// Copies the entire state to a new LeafStateVector.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<LeafStateVector<T>> Clone() const {
    return std::unique_ptr<LeafStateVector<T>>(DoClone());
  }

 protected:
  LeafStateVector() {}

 private:
  /// Returns a new LeafStateVector containing a copy of the entire state.
  /// Caller must take ownership.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates only the O(N) memory that it returns.
  virtual LeafStateVector<T>* DoClone() const = 0;

  // LeafStateVector objects are neither copyable nor moveable.
  LeafStateVector(const LeafStateVector& other) = delete;
  LeafStateVector& operator=(const LeafStateVector& other) = delete;
  LeafStateVector(LeafStateVector&& other) = delete;
  LeafStateVector& operator=(LeafStateVector&& other) = delete;
};

}  // namespace systems
}  // namespace drake
