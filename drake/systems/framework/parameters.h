#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/abstract_state.h"
#include "drake/systems/framework/discrete_state.h"

namespace drake {
namespace systems {

/// Parameters is a container for variables that parameterize a System so
/// that it can represent a family of related models. Parameters are
/// members of the Context.  Parameters are not Inputs because they do not
/// flow from upstream Systems, and they are not State because the System
/// does not define update functions for them.  If Parameters are modified,
/// they are modified by application-specific logic, extrinsic to the
/// System framework and to the flow of simulation time.
///
/// The Parameters include both vector-valued and abstract-valued elements.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class Parameters {
 public:
  /// Constructs an empty Parameters.
  Parameters() : Parameters({}, {}) {}

  /// Constructs Parameters both @p numeric and @p abstract.
  Parameters(std::vector<std::unique_ptr<BasicVector<T>>>&& numeric,
             std::vector<std::unique_ptr<AbstractValue>>&& abstract)
      : numeric_parameters_(
            std::make_unique<DiscreteState<T>>(std::move(numeric))),
        abstract_parameters_(
            std::make_unique<AbstractState>(std::move(abstract))) {}

  /// Constructs Parameters that are purely @p numeric.
  explicit Parameters(std::vector<std::unique_ptr<BasicVector<T>>>&& numeric)
      : Parameters(std::move(numeric), {}) {}

  /// Constructs Parameters that are purely @p abstract.
  explicit Parameters(std::vector<std::unique_ptr<AbstractValue>>&& abstract)
      : Parameters({}, std::move(abstract)) {}

  /// Constructs Parameters in the common case where the parameters consist of
  /// exactly one numeric vector.
  explicit Parameters(std::unique_ptr<BasicVector<T>> vec)
      : numeric_parameters_(
          std::make_unique<DiscreteState<T>>(std::move(vec))),
        abstract_parameters_(std::make_unique<AbstractState>()) {}

  virtual ~Parameters() {}

  int num_numeric_parameters() const {
    return numeric_parameters_->size();
  }

  /// Returns the vector-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  const BasicVector<T>* get_numeric_parameter(int index) const {
    return numeric_parameters_->get_discrete_state(index);
  }

  /// Returns the vector-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  BasicVector<T>* get_mutable_numeric_parameter(int index) {
    return numeric_parameters_->get_mutable_discrete_state(index);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  const AbstractValue& get_abstract_parameter(int index) const {
    return abstract_parameters_->get_abstract_state(index);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  AbstractValue& get_mutable_abstract_parameter(int index) {
    return abstract_parameters_->get_mutable_abstract_state(index);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds, and throws if the parameter is not of type V.
  template <typename V>
  const V& get_abstract_parameter(int index) const {
    return get_abstract_parameter(index).template GetValue<V>();
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds, and throws if the parameter is not of type V.
  template <typename V>
  V& get_mutable_abstract_parameter(int index) {
    return get_mutable_abstract_parameter(index).template GetMutableValue<V>();
  }

  /// Returns a deep copy of the Parameters.
  std::unique_ptr<Parameters<T>> Clone() {
    auto clone = std::make_unique<Parameters<T>>();
    clone->numeric_parameters_ = numeric_parameters_->Clone();
    clone->abstract_parameters_ = abstract_parameters_->Clone();
    return clone;
  }

  // Parameters are not copyable or moveable.
  Parameters(const Parameters& other) = delete;
  Parameters& operator=(const Parameters& other) = delete;
  Parameters(Parameters&& other) = delete;
  Parameters& operator=(Parameters&& other) = delete;

 private:
  // TODO(david-german-tri): Consider renaming AbstractState and DiscreteState
  // to NumericValues and AbstractValues, since the abstraction is actually
  // used in both State and Parameters.
  std::unique_ptr<DiscreteState<T>> numeric_parameters_;
  std::unique_ptr<AbstractState> abstract_parameters_;
};

}  // namespace systems
}  // namespace drake
