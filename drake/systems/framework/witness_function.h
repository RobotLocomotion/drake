#pragma once

#include <utility>
#include <string>

#include "drake/systems/framework/discrete_event.h"

namespace drake {
namespace systems {

/// Abstract class that describes a function that is able to help determine
/// the time and state at which a simulation should be halted, which may be
/// done for any number of purposes, including publishing or state
/// reinitialization (i.e., event handling).
///
/// A witness functions "triggers" when it crosses zero. For example, the
/// "signed distance" (i.e., Euclidean distance when bodies are disjoint and
/// minimum translational distance when bodies intersect) between two rigid
/// bodies can be used as a witness function to determine both the time of
/// impact for rigid bodies (and their states at that time of impact). A good
/// witness function should cross zero decisively- it should not linger around
/// zero- and should not cross zero repeatedly over a small interval of time
/// or over small changes in state; when a witness function has been "bracketed"
/// over an interval of time (i.e., it changes sign), that witness function will
/// ideally cross zero only once in that interval.
template <class T>
class WitnessFunction {
 public:
  virtual ~WitnessFunction() {}
  explicit WitnessFunction(const System<T>* system) { system_ = system; }

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

  /// Gets the reference to the system corresponding to this witness function.
  const System<T>& get_system() const { return *system_; }

  /// Gets a mutable pointer to the system corresponding to this witness
  /// function.
  System<T>* get_mutable_system() { return system_; }

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

  /// Derived classes will override this function gets the time tolerance with
  /// which to isolate the first witness trigger.
  virtual T get_time_isolation_tolerance() const = 0;

  /// Derived classes will override this function to evaluate the witness
  /// function at the given context.
  virtual T Evaluate(const Context<T>& context) = 0;

  /// Derived classes will override this function to return the positive "dead"
  /// band. This value should generally be non-negative.
  virtual T get_positive_dead_band() const = 0;

  /// Derived classes will override this function to return the negative "dead"
  /// band. This value should generally be non-positive.
  virtual T get_negative_dead_band() const = 0;

  /// Derived classes can override this function to get the time that the
  /// witness function should trigger, given two times and two witness function
  /// evaluations. This default implementation returns the time with the witness
  /// function evaluation that is closer to zero. This function allows a
  /// witness function to specify how the input that "zeros" the witness
  /// function over a small interval is selected. For example, if the witness
  /// function corresponds to a signed distance function, selecting the time
  /// corresponding to the positive value (presumably
  /// @p time_and_witness_value0) would allow the integration to stop before
  /// interpenetration occurs.
  virtual T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                                const std::pair<T, T>& time_and_witness_valuef)
                                const {
    using std::abs;
    const T& value0 = time_and_witness_value0.second;
    const T& valuef = time_and_witness_valuef.second;
    const T& time0 = time_and_witness_value0.first;
    const T& timef = time_and_witness_valuef.first;
    return (abs(value0) < abs(valuef)) ? time0 : timef;
  }

  /// Gets the time that the witness function should trigger, given two times
  /// and two function evaluations. Calls do_get_trigger_time().
  /// @throws std::logic_error if the second time is less than the first time.
  T get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                     const std::pair<T, T>& time_and_witness_valuef) const {
    if (time_and_witness_value0.first > time_and_witness_valuef.first) {
      throw std::logic_error("First time should not be greater than the second"
                                 "time.");
    }

    return do_get_trigger_time(time_and_witness_value0,
                               time_and_witness_valuef);
  }

  /// Checks whether the witness function should trigger using the current
  /// context only (witness functions generally look at the evolution of
  /// the system between two contacts). In other words, this function
  /// determines whether the witness function lies in the "dead band".
  bool is_zero(const Context<T>& context) const {
    T value = Evaluate(context);
    return (value <= get_positive_dead_band() &&
            value >= get_negative_dead_band());
  }

  /// Checks whether the witness function should trigger using given
  /// values at w0 and wf. Note that this function is not specific to a
  /// particular witness function.
  bool should_trigger(const T& w0, const T& wf) const {
    TriggerType ttype = get_trigger_type();

    // Get the positive and negative dead bands.
    T positive_dead = get_positive_dead_band();
    T negative_dead = get_negative_dead_band();

    switch (ttype) {
      case TriggerType::kNone:
        return false;

      case TriggerType::kPositiveThenNegative:
        return (w0 > positive_dead && wf < positive_dead);

      case TriggerType::kNegativeThenPositive:
        return (w0 < negative_dead && wf > negative_dead);

      case TriggerType::kCrossesZero:
        return ((w0 > positive_dead && wf < positive_dead) ||
                (w0 < negative_dead && wf > negative_dead));

      default:
        DRAKE_ABORT();
    }
  }

 protected:
  // The name of this witness function.
  std::string name_;

 private:
  const System<T>* system_;
};

}  // namespace systems
}  // namespace drake
