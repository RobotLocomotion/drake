#pragma once

#include <functional>
#include <limits>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

template <typename T>
class Context;

using SystemConstraintIndex = TypeSafeIndex<class SystemConstraintTag>;

/// The form of a SystemConstraint.
enum class SystemConstraintType {
  kEquality = 0,  ///< The constraint is of the form f(x)=0.
  kInequality =
      1,  ///< The constraint is of the form lower_bound <= f(x) <= upper_bound.
};

/// The bounds of a SystemConstraint.  This also encompasses the form of the
/// constraint: equality constraints occur when both the lower and upper bounds
/// are all zeros.
class SystemConstraintBounds final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemConstraintBounds)

  /// Creates constraint of type SystemConstraintType::kEquality, with the
  /// given size for `f(x)`.
  static SystemConstraintBounds Equality(int size) {
    DRAKE_THROW_UNLESS(size >= 0);
    return SystemConstraintBounds(size);
  }

  /// Creates a constraint with the given upper and lower bounds for `f(x)`.
  /// The type() of this constraint will be kInequality, except in the unusual
  /// case where both lower and upper are all zeros (in which case it is
  /// kEquality).  It is not currently allowed to set lower == upper (creating
  /// an equality constraint in the form f(x) = b), except when b == 0.  Using
  /// a non-zero b might be allowed in the future.
  SystemConstraintBounds(
      const Eigen::Ref<const Eigen::VectorXd>& lower,
      const Eigen::Ref<const Eigen::VectorXd>& upper);

  /// Creates an inequality constraint with the given lower bounds for `f(x)`.
  /// The upper bounds are all positive infinity.
  SystemConstraintBounds(
      const Eigen::Ref<const Eigen::VectorXd>& lower,
      stx::nullopt_t);

  /// Creates an inequality constraint with the given upper bounds for `f(x)`.
  /// The lower bounds are all negative infinity.
  SystemConstraintBounds(
      stx::nullopt_t,
      const Eigen::Ref<const Eigen::VectorXd>& upper);

  int size() const { return size_; }
  SystemConstraintType type() const { return type_; }
  const Eigen::VectorXd& lower() const { return lower_; }
  const Eigen::VectorXd& upper() const { return upper_; }

 private:
  explicit SystemConstraintBounds(int size);

  int size_{};
  SystemConstraintType type_{};
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;
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

  /// Constructs a SystemConstraint.  Depending on the `bounds` it could be an
  /// equality constraint f(x) = 0, or an inequality constraint lower_bound <=
  /// f(x) <= upper_bound.
  ///
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(CalcCallback calc_function, SystemConstraintBounds bounds,
                   std::string description)
      : calc_function_(std::move(calc_function)),
        bounds_(std::move(bounds)),
        description_(std::move(description)) {
  }

  /// Evaluates the function pointer passed in through the constructor,
  /// writing the output to @p value.  @p value will be (non-conservatively)
  /// resized to match the constraint function output.
  void Calc(const Context<T>& context, VectorX<T>* value) const {
    value->resize(size());
    calc_function_(context, value);
    DRAKE_DEMAND(value->size() == size());
  }

  /// Evaluates the function pointer, and check if all of the outputs
  /// are within the desired bounds.
  boolean<T> CheckSatisfied(const Context<T>& context, double tol) const {
    DRAKE_DEMAND(tol >= 0.0);
    VectorX<T> value(size());
    Calc(context, &value);
    // Special-case (tol == 0.0) cases both so that the symbolic form is
    // elegant, and so that double evaluation is as fast as possible.
    if (type() == SystemConstraintType::kEquality) {
      if (tol == 0.0) {
        return all(value.array() == 0.0);
      } else {
        return all(value.cwiseAbs().array() <= tol);
      }
    } else {
      DRAKE_ASSERT(type() == SystemConstraintType::kInequality);
      // TODO(hongkai.dai): ignore the bounds that are infinite.
      if (tol == 0.0) {
        return all(value.array() >= lower_bound().array()) &&
               all(value.array() <= upper_bound().array());
      } else {
        return all((value - lower_bound()).array() >= -tol) &&
               all((upper_bound() - value).array() >= -tol);
      }
    }
  }

  // Accessor methods.
  const SystemConstraintBounds& bounds() const { return bounds_; }
  int size() const { return bounds_.size(); }
  SystemConstraintType type() const { return bounds_.type(); }
  bool is_equality_constraint() const {
    return (bounds_.type() == SystemConstraintType::kEquality);
  }
  const Eigen::VectorXd& lower_bound() const { return bounds_.lower(); }
  const Eigen::VectorXd& upper_bound() const { return bounds_.upper(); }
  const std::string& description() const { return description_; }

 private:
  const CalcCallback calc_function_;
  const SystemConstraintBounds bounds_;
  const std::string description_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemConstraint)
