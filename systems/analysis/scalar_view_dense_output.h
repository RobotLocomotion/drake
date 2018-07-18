#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/dense_output.h"
#include "drake/systems/analysis/scalar_dense_output.h"

namespace drake {
namespace systems {

/// A ScalarDenseOutput class implementation that wraps a
/// DenseOutput class instance and behaves as a view to one of
/// its dimensions.
///
/// @tparam T A valid Eigen scalar.
template <typename T>
class ScalarViewDenseOutput : public ScalarDenseOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarViewDenseOutput)

  /// Constructs a view of another DenseOutput instance.
  /// @param base_output Base dense output to operate with.
  /// @param dimension The dimension to view.
  /// @throw std::logic_error if specified @p dimension is not a
  ///                         valid dimension of @p base_output.
  explicit ScalarViewDenseOutput(
      std::unique_ptr<DenseOutput<T>> base_output, int dimension)
      : base_output_(std::move(base_output)),
        dimension_(dimension) {
    if (dimension < 0 || dimension >= base_output_->get_dimensions()) {
      throw std::logic_error("Dimension out of range for the "
                             "given base dense output.");
    }
  }

  /// Returns the base dense output upon which the
  /// view operates.
  const DenseOutput<T>* get_base_output() const {
    return base_output_.get();
  }

 private:
  T DoEvaluate(const T& t) const override {
    return base_output_->Evaluate(t, dimension_);
  }

  bool do_is_empty() const override {
    return base_output_->is_empty();
  }

  const T& do_get_start_time() const override {
    return base_output_->get_start_time();
  }

  const T& do_get_end_time() const override {
    return base_output_->get_end_time();
  }

  // The base (vector) dense output being wrapped.
  const std::unique_ptr<DenseOutput<T>> base_output_;
  // The dimension of interest for the view.
  const int dimension_;
};

}  // namespace systems
}  // namespace drake
