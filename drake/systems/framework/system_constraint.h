#pragma once

#include <functional>
#include <string>
#include <utility>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace systems {

template <typename T>
class Context;

using SystemConstraintIndex = TypeSafeIndex<class SystemConstraintTag>;

/// A SystemConstraint is a generic base-class for constraints on Systems.
///
/// A SystemConstraint is a means to inform our algorithms *about*
/// the implemented system behavior -- declaring the constraint does not
/// *cause* the system behavior to change.  It is meant to improve analysis
/// by telling our algorithms that "all valid solutions of this dynamical
/// system will satisfy the following (in)equalities".  Examples could
/// include conserved quantities or joint limits on a mechanism.
///
/// This class is intentionally compatible with, but (so far) independent from
/// solvers::Constraint. This is primarily because there is no notion of
/// decision variables in the system classes (yet); rather each individual
/// algorithm (e.g. trajectory optimization, or system identification)
/// constructs decision variables for the particular mathematical program that
/// is being formulated, and must bind the system constraint to those variables
/// (e.g. by populating the Context with the decision variables and calling
/// Calc).
///
/// @see LeafSystem<T>::DeclareConstraint for use cases.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
template <typename T>
class SystemConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemConstraint)

  /// This is the signature of a stateless function that evaluates the value of
  /// the constraint function f:
  ///   value = f(context),
  /// where value has the dimension specified in the constructor.
  // TODO(russt): replace the argument VectorX<T>* with an Eigen::Ref* using
  // whatever magic jwnimmer and soonho figured out a few weeks back.
  using CalcCallback =
      std::function<void(const Context<T>& context, VectorX<T>* value)>;

  /// Constructs the SystemConstraint.
  ///
  /// @param count the number of constraints (size of the value vector).
  /// @param is_equality_constraint true implies that the constraint is of the
  /// form f(x)=0, false implies an inequality constraint of the form f(x)≥0.
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(CalcCallback calc_function, int count,
                   bool is_equality_constraint, const std::string& description)
      : calc_function_(std::move(calc_function)),
        count_(count),
        is_equality_constraint_(is_equality_constraint),
        description_(description) {
    DRAKE_DEMAND(count_ >= 0);
  }
  virtual ~SystemConstraint() = default;

  /// Evaluates the function pointer passed in through the constructor,
  /// writing the output to @p value.  @p value will be (non-conservatively)
  /// resized to match the constraint function output.
  void Calc(const Context<T>& context, VectorX<T>* value) const {
    value->resize(count_);
    calc_function_(context, value);
    DRAKE_DEMAND(value->size() == count_);
  }

  /// Evaluates the function pointer, and check if all of the outputs
  /// are within the desired bounds.
  // TODO(russt): Resolve names differences across the codebase. The vector
  // gen scripts call this IsValid, but Constraint calls it CheckSatisfied.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value, bool>::type CheckSatisfied(
      const Context<T1>& context, double tol = 1E-6) const {
    DRAKE_DEMAND(tol >= 0.0);
    VectorX<T> value(count_);
    Calc(context, &value);
    if (is_equality_constraint_) {
      return (value.template lpNorm<Eigen::Infinity>() <= tol);
    } else {
      return (value.array() >= -tol).all();
    }
  }

  // Accessor methods.
  int size() const { return count_; }
  bool is_equality_constraint() const { return is_equality_constraint_; }
  const std::string& description() const { return description_; }

 private:
  const CalcCallback calc_function_;
  const int count_{0};
  const bool is_equality_constraint_{false};
  const std::string description_;
};

}  // namespace systems
}  // namespace drake
