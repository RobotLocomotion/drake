#pragma once

#include "drake/common/default_scalars.h"
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
/// @tparam_default_scalar
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
  /// @throws std::runtime_error if given @p t is not within output's domain
  ///                            i.e. @p t ∉ [start_time(), end_time()].
  T EvaluateScalar(const T& t) const {
    this->ThrowIfOutputIsEmpty(__func__);
    this->ThrowIfTimeIsInvalid(__func__, t);
    return this->DoEvaluateScalar(t);
  }

 protected:
  ScalarDenseOutput() = default;

  VectorX<T> DoEvaluate(const T& t) const override {
    return VectorX<T>::Constant(1, this->DoEvaluateScalar(t));
  }

  int do_size() const override {
    return 1;
  }

  // @see EvaluateScalar(const T&)
  virtual T DoEvaluateScalar(const T& t) const = 0;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ScalarDenseOutput)
