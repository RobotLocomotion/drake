#pragma once

#include <functional>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

template <typename T>
class Context;

using SystemConstraintIndex = TypeSafeIndex<class SystemConstraintTag>;

enum class SystemConstraintType {
  kEquality = 0,  ///< The constraint is of the form f(x)=0.
  kInequality =
      1,  ///< The constraint is of the form lower_bound <= f(x) <= upper_bound.
};

/// A SystemConstraint is a generic base-class for constraints on Systems.
///
/// A SystemConstraint is a means to inform our algorithms *about*
/// the implemented system behavior -- declaring the constraint does not
/// *cause* the system behavior to change.  It is meant to improve analysis
/// by telling our algorithms that "all valid solutions of this dynamical
/// system will satisfy the following (in)equalities".  Examples could
/// include conserved quantities or joint limits on a mechanism.
///
/// TODO(hongkai.dai): this class can be used to generate solvers::Constraint.
///
/// This class is intentionally similar to, but (so far) independent from
/// solvers::Constraint. This is primarily because there is no notion of
/// decision variables in the system classes (yet); rather each individual
/// algorithm (e.g. trajectory optimization, or system identification)
/// constructs decision variables for the particular mathematical program that
/// is being formulated, and must bind the system constraint to those variables
/// (e.g. by populating the Context with the decision variables and calling
/// Calc).
///
/// @see LeafSystem<T>::DeclareEqualityConstraint and
///      LeafSystem<T>::DeclareInequalityConstraint for use cases.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
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

  /// Constructs a SystemConstraint with equality constraint f(x) = 0.
  ///
  /// @param count the number of constraints (size of the value vector).
  /// @param type the SystemConstraintType.
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(CalcCallback calc_function, int count,
                   const std::string& description)
      : calc_function_(std::move(calc_function)),
        count_(count),
        lower_bound_(Eigen::VectorXd::Zero(count_)),
        upper_bound_(Eigen::VectorXd::Zero(count_)),
        type_(SystemConstraintType::kEquality),
        description_(description) {
    DRAKE_DEMAND(count_ >= 0);
  }

  /// Constructs a SystemConstraint with inequality constraint lower_bound <=
  /// f(x) <= upper_bound
  SystemConstraint(CalcCallback calc_function,
                   const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                   const Eigen::Ref<const Eigen::VectorXd>& upper_bound,
                   const std::string& description)
      : calc_function_(std::move(calc_function)),
        count_(lower_bound.rows()),
        lower_bound_(lower_bound),
        upper_bound_(upper_bound),
        type_(SystemConstraintType::kInequality),
        description_(description) {
    DRAKE_DEMAND(lower_bound.rows() == upper_bound.rows());
    DRAKE_DEMAND((lower_bound.array() <= upper_bound.array()).all());
  }

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
  boolean<T> CheckSatisfied(const Context<T>& context, double tol) const {
    DRAKE_DEMAND(tol >= 0.0);
    VectorX<T> value(count_);
    Calc(context, &value);
    // Special-case (tol == 0.0) cases both so that the symbolic form is
    // elegant, and so that double evaluation is as fast as possible.
    if (type_ == SystemConstraintType::kEquality) {
      if (tol == 0.0) {
        return all(value.array() == 0.0);
      } else {
        return all(value.cwiseAbs().array() <= tol);
      }
    } else {
      DRAKE_ASSERT(type_ == SystemConstraintType::kInequality);
      if (tol == 0.0) {
        return all(value.array() >= lower_bound_.array()) &&
               all(value.array() <= upper_bound_.array());
      } else {
        return all((value - lower_bound_).array() >= -tol) &&
               all((upper_bound_ - value).array() >= -tol);
      }
    }
  }

  // Accessor methods.
  int size() const { return count_; }
  SystemConstraintType type() const { return type_; }
  bool is_equality_constraint() const {
    return (type_ == SystemConstraintType::kEquality);
  }
  const Eigen::VectorXd& lower_bound() const { return lower_bound_; }
  const Eigen::VectorXd& upper_bound() const { return upper_bound_; }
  const std::string& description() const { return description_; }

 private:
  const CalcCallback calc_function_;
  const int count_{0};
  const Eigen::VectorXd lower_bound_;
  const Eigen::VectorXd upper_bound_;
  const SystemConstraintType type_;
  const std::string description_;
};

}  // namespace systems
}  // namespace drake
