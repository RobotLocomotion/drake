#pragma once

#include <functional>
#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
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
/// Note: Equality constraints are implemented by setting lower_bound ==
/// upper_bound.
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
  ///   value = f(context)
  /// where value has the same number of elements as lower_bound and
  /// upper_bound.
  // TODO(russt): replace the argument VectorX<T>* with an Eigen::Ref* using
  // whatever magic jwnimmer and soonho figured out a few weeks back.
  using CalcCallback =
      std::function<void(const Context<T>& context, VectorX<T>* value)>;

  /// Constructs the SystemConstraint.
  SystemConstraint(CalcCallback calc_function,
                   const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                   const Eigen::Ref<const Eigen::VectorXd>& upper_bound,
                   const std::string& description)
      : calc_function_(calc_function),
        lower_bound_(lower_bound),
        upper_bound_(upper_bound),
        description_(description) {
    DRAKE_DEMAND(lower_bound.rows() == upper_bound.rows());
    DRAKE_ASSERT((lower_bound.array() <= upper_bound.array()).all());
  }
  virtual ~SystemConstraint() = default;

  /// Evaluates the function pointer passed in through the constructor,
  /// writing the output to @p value.  @p value will be (non-conservatively)
  /// resized to match the constraint function output.
  void Calc(const Context<T>& context, VectorX<T>* value) const {
    value->resize(lower_bound_.rows());
    calc_function_(context, value);
    DRAKE_DEMAND(value->size() == lower_bound_.rows());
  }

  /// Evaluates the function pointer, and check if all of the outputs
  /// are within the desired bounds.
  // TODO(russt): Resolve names differences across the codebase. The vector
  // gen scripts call this IsValid, but Constraint calls it CheckSatisfied.
  bool CheckSatisfied(const Context<T>& context,
                      const double tol = 1E-6) const {
    DRAKE_DEMAND(tol >= 0.0);
    VectorX<T> value(lower_bound_.rows());
    Calc(context, &value);
    return (value.array() >= lower_bound_.array() - tol).all() &&
           (value.array() <= upper_bound_.array() + tol).all();
  }

  // Accessor methods.
  int size() const { return lower_bound_.rows(); }
  const Eigen::VectorXd& lower_bound() const { return lower_bound_; }
  const Eigen::VectorXd& upper_bound() const { return upper_bound_; }
  const std::string& description() const { return description_; }

 private:
  const CalcCallback calc_function_;
  const Eigen::VectorXd lower_bound_;
  const Eigen::VectorXd upper_bound_;
  const std::string description_;
};

}  // namespace systems
}  // namespace drake
