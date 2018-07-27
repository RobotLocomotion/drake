#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/dense_output.h"

namespace drake {
namespace systems {

/// A DenseOutput class interface extension to deal with scalar ODE
/// solutions. A ScalarDenseOutput instance is also a DenseOutput
/// instance with single element vector values (i.e. size() == 1).
/// As such, its value can evaluated in both scalar and vectorial
/// form (via EvaluateScalar() and Evaluate(), respectively).
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class ScalarDenseOutput : public DenseOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarDenseOutput)

  virtual ~ScalarDenseOutput() = default;

  /// Evaluates output at the given time @p t.
  /// @param t Time at which to evaluate output.
  /// @returns Output scalar value.
  /// @pre Output is not empty i.e. is_empty() is false.
  /// @throws std::logic_error if any of the preconditions is not met.
  /// @throws std::runtime_error if the output is not defined for the
  ///                            given @p t.
  T EvaluateScalar(const T& t) const {
    if (this->is_empty()) {
      throw std::logic_error("Empty dense output cannot be evaluated.");
    }
    if (t < this->start_time() || t > this->end_time()) {
      throw std::runtime_error("Dense output is not defined for given time.");
    }
    return this->DoEvaluateScalar(t);
  }

 protected:
  ScalarDenseOutput() = default;

 private:
  // @see EvaluateScalar(const T&)
  virtual T DoEvaluateScalar(const T& t) const = 0;

  VectorX<T> DoEvaluate(const T& t) const override {
    return VectorX<T>::Constant(1, this->DoEvaluateScalar(t));
  }

  int do_size() const override {
    return 1;
  }
};

}  // namespace systems
}  // namespace drake
