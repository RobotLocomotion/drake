#pragma once

#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <class T>
class WitnessFunction;

template <typename T>
struct DiscreteEvent {
  typedef std::function<void(const Context<T>&)> PublishCallback;
  typedef std::function<void(const Context<T>&, DiscreteValues<T>*)>
      DiscreteUpdateCallback;
  typedef std::function<void(const Context<T>&, State<T>*)>
      UnrestrictedUpdateCallback;

  /// These enumerations represent an indication of the type of event that
  /// triggered the event handler, toward obviating the need to redetermine
  /// the reason that the event handler is called.
  enum ActionType {
    /// A default value that causes the handler to abort.
        kUnknownAction = 0,

    /// On publish actions, state does not change.
        kPublishAction = 1,

    /// On discrete updates, discrete state may change.
        kDiscreteUpdateAction = 2,

    /// On unrestricted updates, the state variables may change arbitrarily.
        kUnrestrictedUpdateAction = 3,
  };

  /// The witness function that triggered this event, if any.
  const WitnessFunction<T>* triggered_witness_function{nullptr};

  /// The type of action the system must take in response to the event.
  ActionType action{kUnknownAction};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kPublishAction. If nullptr, Publish() will be used.
  PublishCallback do_publish{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kDiscreteUpdateAction. If nullptr, DoCalcDiscreteVariableUpdates() will
  /// be used.
  DiscreteUpdateCallback do_calc_discrete_variable_update{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kUpdateUnrestrictedAction. If nullptr, DoCalcUnrestrictedUpdate() will be
  /// used.
  UnrestrictedUpdateCallback do_unrestricted_update{nullptr};
};

}  // namespace systems
}  // namespace drake
