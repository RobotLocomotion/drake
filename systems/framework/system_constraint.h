#pragma once

#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

// Break the System <=> SystemConstraint physical dependency cycle.
// SystemConstraint is decorated with a back-pointer to its owning System,
// but that pointer is never dereferenced within this component.
template <typename T>
class System;

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

  /// Creates constraint bounds with zero size.
  SystemConstraintBounds() : SystemConstraintBounds(0) {}

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
      std::nullopt_t);

  /// Creates an inequality constraint with the given upper bounds for `f(x)`.
  /// The lower bounds are all negative infinity.
  SystemConstraintBounds(
      std::nullopt_t,
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

/// This is the signature of a stateless function that evaluates the value of
/// the constraint function f:
///   value = f(context)
///
/// Note that in the std::function signature, the computed value is an output
/// parameter, not a return value.
///
/// See also SystemConstraintCalc, which offers the System reference.
template <typename T>
using ContextConstraintCalc =
    std::function<void(const Context<T>&, VectorX<T>* value)>;

/// This is the signature of a stateless function that evaluates the value of
/// the constraint function f:
///   value = f(system, context)
///
/// Note that in the std::function signature, the computed value is an output
/// parameter, not a return value.
///
/// Instances of this function type are expected to work with *any* instance of
/// the class of System they are designed for.  Specifically, they should not
/// capture pointers into an instance of a System, OutputPort, etc.  Instead,
/// they should only use the System reference that is passed into this functor.
///
/// See also ContextConstraintCalc, which omits the System reference.  A value
/// of type ContextConstraintCalc is allowed to assume it's only ever applied
/// to a specific System object.
template <typename T>
using SystemConstraintCalc =
    std::function<void(const System<T>&, const Context<T>&, VectorX<T>* value)>;

/// A SystemConstraint is a generic base-class for constraints on Systems.
///
/// A SystemConstraint is a means to inform our algorithms *about*
/// the implemented system behavior -- declaring the constraint does not
/// *cause* the system behavior to change.  It is meant to improve analysis
/// by telling our algorithms that "all valid solutions of this dynamical
/// system will satisfy the following (in)equalities".  Examples could
/// include conserved quantities or joint limits on a mechanism.
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
/// We can convert a SystemConstraint to a solvers::Constraint by using
/// SystemConstraintWrapper or SystemConstraintAdapter.
///
/// @see LeafSystem<T>::DeclareEqualityConstraint and
///      LeafSystem<T>::DeclareInequalityConstraint for use cases.
/// @tparam_default_scalar
template <typename T>
class SystemConstraint final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemConstraint)

  /// (Advanced) Constructs a default (zero-sized) SystemConstraint.
  ///
  /// Most users should call a LeafSystem method like DeclareEqualityConstraint
  /// to create (and add) constraints, not call this constructor directly.
  ///
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(const System<T>* system,
                   std::string description)
    : SystemConstraint<T>(
        system, &NoopSystemConstraintCalc, SystemConstraintBounds{},
        std::move(description)) {}

  /// (Advanced) Constructs a SystemConstraint.  Depending on the `bounds` it
  /// could be an equality constraint f(x) = 0, or an inequality constraint
  /// lower_bound <= f(x) <= upper_bound.
  ///
  /// Most users should call a LeafSystem method like DeclareEqualityConstraint
  /// to create (and add) constraints, not call this constructor directly.
  ///
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(const System<T>* system,
                   ContextConstraintCalc<T> calc_function,
                   SystemConstraintBounds bounds,
                   std::string description)
      : system_(system),
        system_calc_function_{},
        context_calc_function_(std::move(calc_function)),
        bounds_(std::move(bounds)),
        description_(std::move(description)) {
    DRAKE_DEMAND(system != nullptr);
  }

  /// (Advanced) Constructs a SystemConstraint.  Depending on the `bounds` it
  /// could be an equality constraint f(x) = 0, or an inequality constraint
  /// lower_bound <= f(x) <= upper_bound.
  ///
  /// Most users should call a LeafSystem method like DeclareEqualityConstraint
  /// to create (and add) constraints, not call this constructor directly.
  ///
  /// @param description a human-readable description useful for debugging.
  SystemConstraint(const System<T>* system,
                   SystemConstraintCalc<T> calc_function,
                   SystemConstraintBounds bounds,
                   std::string description)
      : system_(system),
        system_calc_function_(std::move(calc_function)),
        context_calc_function_{},
        bounds_(std::move(bounds)),
        description_(std::move(description)) {
    DRAKE_DEMAND(system != nullptr);
  }

  /// Evaluates the function pointer passed in through the constructor,
  /// writing the output to @p value.  @p value will be (non-conservatively)
  /// resized to match the constraint function output.
  void Calc(const Context<T>& context, VectorX<T>* value) const {
    MaybeValidateSystemIdsMatch(context);
    value->resize(size());
    if (context_calc_function_) {
      context_calc_function_(context, value);
    } else {
      system_calc_function_(*system_, context, value);
    }
    DRAKE_DEMAND(value->size() == size());
  }

  /// Evaluates the function pointer, and check if all of the outputs
  /// are within the desired bounds.
  boolean<T> CheckSatisfied(const Context<T>& context, double tol) const {
    MaybeValidateSystemIdsMatch(context);
    DRAKE_DEMAND(tol >= 0.0);
    VectorX<T> value(size());
    Calc(context, &value);
    // Special-case (tol == 0.0) cases both so that the symbolic form is
    // elegant, and so that double evaluation is as fast as possible.
    if (type() == SystemConstraintType::kEquality) {
      if (tol == 0.0) {
        return drake::all(value.array() == 0.0);
      } else {
        return drake::all(value.cwiseAbs().array() <= tol);
      }
    } else {
      DRAKE_ASSERT(type() == SystemConstraintType::kInequality);
      // TODO(hongkai.dai): ignore the bounds that are infinite.
      if (tol == 0.0) {
        return drake::all(value.array() >= lower_bound().array()) &&
               drake::all(value.array() <= upper_bound().array());
      } else {
        return drake::all((value - lower_bound()).array() >= -tol) &&
               drake::all((upper_bound() - value).array() >= -tol);
      }
    }
  }

  /// Returns a reference to the System that owns this constraint.  Note that
  /// for a constraint on a diagram this will be the diagram itself, never a
  /// leaf system whose constraint was re-expressed.
  const System<T>& get_system() const {
    return *system_;
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

  /// @name System compatibility
  /// See @ref system_compatibility.
  //@{
  /// (Internal use only) Gets the id of the subsystem associated with this
  /// object, if one has been set.
  const std::optional<internal::SystemId>& get_system_id() const {
    return system_id_;
  }

  /// (Internal use only) Records the id of the subsystem associated with this
  /// object.
  void set_system_id(internal::SystemId id) { system_id_ = id; }
  //@}

 private:
  static void NoopSystemConstraintCalc(
      const System<T>&, const Context<T>&, VectorX<T>*) {}

  // If this object has a system id, check that it matches the id of the
  // context parameter.
  void MaybeValidateSystemIdsMatch(const Context<T>& context) const {
    DRAKE_DEMAND(!system_id_.has_value() ||
                 *system_id_ == context.get_system_id());
  }

  const System<T>* const system_;
  const SystemConstraintCalc<T> system_calc_function_;
  const ContextConstraintCalc<T> context_calc_function_;
  const SystemConstraintBounds bounds_;
  const std::string description_;

  // The id of the subsystem associated with this object.
  std::optional<internal::SystemId> system_id_;
};

/// An "external" constraint on a System.  This class is intended for use by
/// applications that are examining a System by adding additional constraints
/// based on their particular situation (e.g., that a velocity state element
/// has an upper bound); it is not intended for declaring intrinsic constraints
/// that some particular System subclass might always impose on itself (e.g.,
/// that a mass parameter is non-negative).
class ExternalSystemConstraint final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExternalSystemConstraint)

  /// Creates an empty constraint.
  ExternalSystemConstraint()
      : ExternalSystemConstraint("empty", {}, {}) {}

  /// Creates a constraint with the given arguments.
  /// The calc functions (other than calc_double) may be omitted.
  ExternalSystemConstraint(
      std::string description,
      SystemConstraintBounds bounds,
      SystemConstraintCalc<double> calc_double,
      SystemConstraintCalc<AutoDiffXd> calc_autodiffxd = {},
      SystemConstraintCalc<symbolic::Expression> calc_expression = {})
      : description_(std::move(description)),
        bounds_(std::move(bounds)),
        calc_double_(std::move(calc_double)),
        calc_autodiffxd_(std::move(calc_autodiffxd)),
        calc_expression_(std::move(calc_expression)) {}

  /// Creates a constraint based on generic lambda.  This constraint will
  /// supply Calc functions for Drake's default scalar types.
  template <typename GenericSystemConstraintCalc>
  static ExternalSystemConstraint MakeForAllScalars(
      std::string description,
      SystemConstraintBounds bounds,
      GenericSystemConstraintCalc calc) {
    return ExternalSystemConstraint(
        std::move(description),
        std::move(bounds),
        calc, calc, calc);
  }

  /// Creates a constraint based on generic lambda.  This constraint will
  /// supply Calc functions for Drake's non-symbolic default scalar types.
  template <typename GenericSystemConstraintCalc>
  static ExternalSystemConstraint MakeForNonsymbolicScalars(
      std::string description,
      SystemConstraintBounds bounds,
      GenericSystemConstraintCalc calc) {
    return ExternalSystemConstraint(
        std::move(description),
        std::move(bounds),
        calc, calc, {});
  }

  /// Returns a human-readable description of this constraint.
  const std::string& description() const { return description_; }

  /// Returns the bounds of this constraint (and whether it is an equality or
  /// inequality constraint.)
  const SystemConstraintBounds& bounds() const { return bounds_; }

  /// Retrieves the evaluation function `value = f(system, context)` for this
  /// constraint.  The result may be a default-constructed (missing) function,
  /// if the scalar type T is not supported by this constraint instance.
  ///
  /// @tparam T denotes the scalar type of the System<T>.
  template <typename T>
  const SystemConstraintCalc<T>& get_calc() const {
    return do_get_calc<T>();
  }

 private:
  // This is the generic fallback implementation for unknown scalars.  Below,
  // we specialize this template function for the scalars that we know about
  // (i.e., for when we can return references to our calc_foo_ member fields).
  template <typename T>
  const SystemConstraintCalc<T>& do_get_calc() const {
    static const never_destroyed<SystemConstraintCalc<T>> empty;
    return empty.access();
  }

  std::string description_;
  SystemConstraintBounds bounds_;
  SystemConstraintCalc<double> calc_double_;
  SystemConstraintCalc<AutoDiffXd> calc_autodiffxd_;
  SystemConstraintCalc<symbolic::Expression> calc_expression_;
};

template <> inline
const SystemConstraintCalc<double>&
ExternalSystemConstraint::do_get_calc() const {
  return calc_double_;
}

template <> inline
const SystemConstraintCalc<AutoDiffXd>&
ExternalSystemConstraint::do_get_calc() const {
  return calc_autodiffxd_;
}

template <> inline
const SystemConstraintCalc<symbolic::Expression>&
ExternalSystemConstraint::do_get_calc() const {
  return calc_expression_;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemConstraint)
