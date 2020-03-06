#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/discrete_values.h"

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
/// @tparam_default_scalar
template <typename T>
class Parameters {
 public:
  // Parameters are not copyable or moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Parameters)

  /// Constructs an empty Parameters.
  Parameters() : Parameters({}, {}) {}

  /// Constructs Parameters both @p numeric and @p abstract.
  Parameters(std::vector<std::unique_ptr<BasicVector<T>>>&& numeric,
             std::vector<std::unique_ptr<AbstractValue>>&& abstract)
      : numeric_parameters_(
            std::make_unique<DiscreteValues<T>>(std::move(numeric))),
        abstract_parameters_(
            std::make_unique<AbstractValues>(std::move(abstract))) {}

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
            std::make_unique<DiscreteValues<T>>(std::move(vec))),
        abstract_parameters_(std::make_unique<AbstractValues>()) {}

  /// Constructs Parameters in the common case where the parameters consist of
  /// exactly one abstract value.
  explicit Parameters(std::unique_ptr<AbstractValue> value)
      : numeric_parameters_(std::make_unique<DiscreteValues<T>>()),
        abstract_parameters_(
            std::make_unique<AbstractValues>(std::move(value))) {}

  virtual ~Parameters() {}

  int num_numeric_parameter_groups() const {
    return numeric_parameters_->num_groups();
  }

  int num_abstract_parameters() const {
    return abstract_parameters_->size();
  }

  /// Returns the vector-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  const BasicVector<T>& get_numeric_parameter(int index) const {
    return numeric_parameters_->get_vector(index);
  }

  /// Returns the vector-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  BasicVector<T>& get_mutable_numeric_parameter(int index) {
    return numeric_parameters_->get_mutable_vector(index);
  }

  const DiscreteValues<T>& get_numeric_parameters() const {
    return *numeric_parameters_;
  }

  void set_numeric_parameters(
      std::unique_ptr<DiscreteValues<T>> numeric_params) {
    DRAKE_DEMAND(numeric_params != nullptr);
    numeric_parameters_ = std::move(numeric_params);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  const AbstractValue& get_abstract_parameter(int index) const {
    return abstract_parameters_->get_value(index);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds.
  AbstractValue& get_mutable_abstract_parameter(int index) {
    return abstract_parameters_->get_mutable_value(index);
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds, and throws if the parameter is not of type V.
  template <typename V>
  const V& get_abstract_parameter(int index) const {
    return get_abstract_parameter(index).template get_value<V>();
  }

  /// Returns the abstract-valued parameter at @p index. Asserts if the index
  /// is out of bounds, and throws if the parameter is not of type V.
  template <typename V>
  V& get_mutable_abstract_parameter(int index) {
    return get_mutable_abstract_parameter(index).
        template get_mutable_value<V>();
  }

  const AbstractValues& get_abstract_parameters() const {
    return *abstract_parameters_;
  }

  void set_abstract_parameters(
      std::unique_ptr<AbstractValues> abstract_params) {
    DRAKE_DEMAND(abstract_params != nullptr);
    abstract_parameters_ = std::move(abstract_params);
  }

  /// Returns a deep copy of the Parameters.
  std::unique_ptr<Parameters<T>> Clone() const {
    auto clone = std::make_unique<Parameters<T>>();
    clone->set_numeric_parameters(numeric_parameters_->Clone());
    clone->set_abstract_parameters(abstract_parameters_->Clone());
    return clone;
  }

  /// Initializes this state from `other`.
  template <typename U>
  void SetFrom(const Parameters<U>& other) {
    numeric_parameters_->SetFrom(other.get_numeric_parameters());
    abstract_parameters_->SetFrom(other.get_abstract_parameters());
  }

 private:
  std::unique_ptr<DiscreteValues<T>> numeric_parameters_;
  std::unique_ptr<AbstractValues> abstract_parameters_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Parameters)
