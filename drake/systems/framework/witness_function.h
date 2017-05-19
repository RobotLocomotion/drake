#pragma once

#include <limits>
#include <string>
#include <utility>

#include "drake/systems/framework/discrete_event.h"

namespace drake {
namespace systems {

/// Abstract class that describes a function that is able to help determine
/// the time and state at which a simulation should be halted, which may be
/// done for any number of purposes, including publishing or state
/// reinitialization (i.e., event handling).
///
/// For the ensuing discussion, consider two times (`t₀` and `t₁ > t₀`) and
/// states corresponding to those times (`x₀` and `x₁`). A witness function,
/// `w(t, x)`, "triggers" only when it crosses zero at a time `t*` where
/// t₀ < t* ≤ t₁`. Note the half-open interval. For a concrete example of a
/// witness function, consider the "signed distance" (i.e., Euclidean distance
/// when bodies are disjoint and minimum translational distance when bodies
/// intersect) between two rigid bodies; this witness function can be used to
/// determine both the time of impact for rigid bodies and their states at that
/// time of impact.
///
/// Precision in the definition of the witness function is necessary, because we
/// want the witness function to trigger only once if, for example,
/// `w(t₀, x₀) ≠ 0`, `w(t₁, x₁) = 0`, and `w(t₂, x₂) ≠ 0`, for some t₂ > t₁.
/// In other words, if the witness function is evaluated over the intervals
/// [t₀, t₁] and [t₁, t₂], meaning that the zero occurs precisely at an interval
/// endpoint, the witness function should trigger once. Similarly, the witness
/// function should trigger exactly once if `w(t₀, x₀) ≠ 0`, `w(t*, x*) = 0`,
/// and `w(t₁, x₁) = 0`. We can define the trigger condition formally over
/// interval `[t₀, t₁]` as:<pre>
/// T(w, t₀, x₀, t₁) =  1   if w(t₀, x₀) ≠ 0 and w(t₀, x₀)⋅w(t₁, x₁) ≤ 0
///                     0   if w(t₀, x₀) = 0 or  w(t₀, x₀)⋅w(t₁, x₁) > 0
/// </pre>
/// where `x(tₑ)` for some `tₑ ≥ t₀` is the solution to the ODE or DAE initial
/// value problem `ẋ = f(t, x)` for `x(t₀) = x₀`. The trigger function can be
/// further modified, if desired, to incorporate the constraint that the witness
/// function should trigger only when crossing from positive values to negative
/// values, or vice versa.
///
/// A good witness function should not cross zero repeatedly over a small
/// interval of time or over small changes in state; when a witness function has
/// been "bracketed" over an interval of time (i.e., it changes sign), that
/// witness function will ideally cross zero only once in that interval.
///
/// Since it is generally more intuitive to specify the time tolerance for which
/// a witness should be considered to be triggered than it is to specify the
/// tolerance for the witness function output (which would need to be
/// specialized for each witness function), this is the only floating point
/// parameter that needs to be tuned. The disadvantage of this scheme is that it
/// always requires the length of the interval to be reduced to the time
/// tolerance *and that each function evaluation (which requires numerical
/// integration) is extraordinarily expensive*. Assuming the (slow) bisection
/// algorithm is used, the number of integrations necessary to cut the interval
/// from a length of ℓ to a length of ε will be log₂(ℓ / ε). In other words,
/// the scheme does not benefit from a fortuitous guess close to the root.
template <class T>
class WitnessFunction {
 public:
  virtual ~WitnessFunction() {}

  enum class TriggerType {
    /// This witness function will never be triggered.
    kNone,

    /// Witness function triggers when the function crosses zero after an
    /// initial positive evaluation.
    kPositiveThenNegative,

    /// Witness function triggers when the function crosses zero after an
    /// initial negative evaluation.
    kNegativeThenPositive,

    /// Witness function triggers *any time* the function crosses zero.
    /// Convenience definition for equivalence to bitwise OR of
    /// kPositiveThenNegative and kNegativeThenPositive.
    kCrossesZero,
  };

  /// Gets the name of this witness function (used primarily for logging and
  /// debugging).
  const std::string& get_name() const { return name_; }

  /// Sets the name of this witness function.
  void set_name(const std::string& name) { name_ = name; }

  /// Derived classes will override this function to get the type of event
  /// that the witness function will trigger.
  virtual typename DiscreteEvent<T>::ActionType get_action_type() const = 0;

  /// Derived classes will override this function to get the witness function
  /// trigger type.
  virtual TriggerType get_trigger_type() const = 0;

  /// Derived classes will override this function to evaluate the witness
  /// function at the given context.
  virtual T Evaluate(const Context<T>& context) = 0;

  /// Derived classes can override this function to specify the absolute time
  /// tolerance with which to isolate the first witness trigger. Default
  /// implementation returns 10 * machine epsilon.
  virtual T get_time_isolation_tolerance() const { return
      T(10 * std::numeric_limits<double>::epsilon()); }

  /// Checks whether the witness function should trigger using given
  /// values at w0 and wf. Note that this function is not specific to a
  /// particular witness function.
  bool should_trigger(const T& w0, const T& wf) const {
    TriggerType ttype = get_trigger_type();

    switch (ttype) {
      case TriggerType::kNone:
        return false;

      case TriggerType::kPositiveThenNegative:
        return (w0 > 0 && wf <= 0);

      case TriggerType::kNegativeThenPositive:
        return (w0 < 0 && wf >= 0);

      case TriggerType::kCrossesZero:
        return ((w0 > 0 && wf <= 0) ||
                (w0 < 0 && wf >= 0));

      default:
        DRAKE_ABORT();
    }
  }

 protected:
  // The name of this witness function.
  std::string name_;
};

}  // namespace systems
}  // namespace drake
