#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// BasicStateAndOutputVector is a concrete class template that implements
/// StateVector in a convenient manner for LeafSystem blocks, and implements
/// VectorBase so that it may also be used as an output.
///
/// TODO(david-german-tri): Collapse BasicStateAndOutputVector into BasicVector.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class BasicStateAndOutputVector : public BasicStateVector<T> {
 public:
  /// Constructs a BasicStateAndOutputVector of the specified @p size.
  explicit BasicStateAndOutputVector(int size) : BasicStateVector<T>(size) {}

  // Constructs a BasicStateVector with the specified @p data.
  explicit BasicStateAndOutputVector(const VectorX<T>& data)
      : BasicStateVector<T>(data) {}

  // Disable assignment and move, for consistency with parent class.
  BasicStateAndOutputVector& operator=(const BasicStateAndOutputVector&) =
      delete;
  BasicStateAndOutputVector(BasicStateAndOutputVector&&) = delete;
  BasicStateAndOutputVector& operator=(BasicStateAndOutputVector&&) = delete;

 protected:
  explicit BasicStateAndOutputVector(const BasicStateAndOutputVector& other)
      : BasicStateVector<T>(other) {}

  BasicStateAndOutputVector<T>* DoClone() const override {
    return new BasicStateAndOutputVector<T>(*this);
  }
};

}  // namespace systems
}  // namespace drake
