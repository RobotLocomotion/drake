#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/difference_state.h"
#include "drake/systems/framework/modal_state.h"

namespace drake {
namespace systems {

template <typename T>
class Parameters {
 public:
  /// Constructs an empty Parameters.
  Parameters() : Parameters({}, {}) {}

  /// Constructs Parameters both @p numeric and @p modal.
  Parameters(std::vector<std::unique_ptr<BasicVector<T>>>&& numeric,
             std::vector<std::unique_ptr<AbstractValue>>&& modal)
      : numeric_parameters_(
            std::make_unique<DifferenceState<T>>(std::move(numeric))),
        modal_parameters_(
            std::make_unique<ModalState>(std::move(modal))) {}

  /// Constructs Parameters that are purely @p numeric.
  explicit Parameters(std::vector<std::unique_ptr<BasicVector<T>>>&& numeric)
      : Parameters(std::move(numeric), {}) {}

  /// Constructs Parameters that are purely @p modal.
  explicit Parameters(std::vector<std::unique_ptr<AbstractValue>>&& modal)
      : Parameters({}, std::move(modal)) {}

  /// Constructs Parameters in the common case where the parameters consist of
  /// exactly one numeric vector.
  explicit Parameters(std::unique_ptr<BasicVector<T>> vec)
      : numeric_parameters_(
          std::make_unique<DifferenceState<T>>(std::move(vec))),
        modal_parameters_(std::make_unique<ModalState>()) {}

  virtual ~Parameters() {}

  int num_numeric_parameters() const {
    return numeric_parameters_->size();
  }

  const BasicVector<T>* get_numeric_parameter(int index) const {
    return numeric_parameters_->get_difference_state(index);
  }

  BasicVector<T>* get_mutable_numeric_parameter(int index) {
    return numeric_parameters_->get_mutable_difference_state(index);
  }

  const AbstractValue& get_modal_parameter(int index) const {
    return modal_parameters_->get_modal_state(index);
  }

  AbstractValue& get_mutable_modal_parameter(int index) {
    return modal_parameters_->get_mutable_modal_state(index);
  }

  std::unique_ptr<Parameters<T>> Clone() {
    auto clone = std::make_unique<Parameters<T>>();
    clone->numeric_parameters_ = numeric_parameters_->Clone();
    clone->modal_parameters_ = modal_parameters_->Clone();
    return clone;
  }

  // Parameters are not copyable or moveable.
  Parameters(const Parameters& other) = delete;
  Parameters& operator=(const Parameters& other) = delete;
  Parameters(Parameters&& other) = delete;
  Parameters& operator=(Parameters&& other) = delete;

 private:
  // TODO(david-german-tri): Consider finding a more general name for ModalState
  // and DifferenceState, since the abstraction is actually used in both State
  // and Parameters.
  std::unique_ptr<DifferenceState<T>> numeric_parameters_;
  std::unique_ptr<ModalState> modal_parameters_;
};

}  // namespace systems
}  // namespace drake
