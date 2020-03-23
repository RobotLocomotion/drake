#pragma once

#include <memory>
#include <utility>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/dense_output.h"
#include "drake/systems/analysis/scalar_dense_output.h"

namespace drake {
namespace systems {

/// A ScalarDenseOutput class implementation that wraps a
/// DenseOutput class instance and behaves as a view to one of
/// its elements.
///
/// @tparam_default_scalar
template <typename T>
class ScalarViewDenseOutput : public ScalarDenseOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarViewDenseOutput)

  /// Constructs a view of another DenseOutput instance.
  /// @param base_output Base dense output to operate with.
  /// @param n The nth scalar element (0-indexed) of the output value
  ///          to view.
  /// @throws std::runtime_error if @p base_output is nullptr.
  /// @throws std::runtime_error if given @p n does not refer to a valid
  ///                            base output dimension
  ///                            i.e. @p n ∉ [0, `base_output`->size()).
  explicit ScalarViewDenseOutput(
      std::unique_ptr<DenseOutput<T>> base_output, int n)
      : base_output_(std::move(base_output)), n_(n) {
    if (base_output_ == nullptr) {
      throw std::runtime_error("Base dense output to view is null.");
    }
    if (n < 0 || base_output_->size() <= n) {
      throw std::runtime_error(fmt::format(
          "Index {} out of base dense output [0, {}) range.",
          n, base_output_->size()));
    }
  }

  /// Returns the base dense output upon which the
  /// view operates.
  const DenseOutput<T>* get_base_output() const {
    return base_output_.get();
  }

 protected:
  T DoEvaluateScalar(const T& t) const override {
    return base_output_->EvaluateNth(t, n_);
  }

  bool do_is_empty() const override {
    return base_output_->is_empty();
  }

  const T& do_start_time() const override {
    return base_output_->start_time();
  }

  const T& do_end_time() const override {
    return base_output_->end_time();
  }

  // The base (vector) dense output being wrapped.
  const std::unique_ptr<DenseOutput<T>> base_output_;
  // The nth scalar element (0-indexed) of the base
  // (vector) dense output value to view.
  const int n_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ScalarViewDenseOutput)
