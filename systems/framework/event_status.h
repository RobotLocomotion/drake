#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

class SystemBase;

/** Holds the return status from execution of an event handler function, or the
effective status after a series of handler executions due to dispatching of
simultaneous events. Drake API users will typically use only the four factory
methods below to return status, and optionally a human-readable message, from
their event handlers. */

// TODO(sherm1) Add the following text to the above doxygen comment when
// EventStatus is propagated up from the handlers.
/* (Advanced) In case you are writing an event dispatcher (that is, you are
overriding LeafSystem::DoPublish(), LeafSystem::DoCalcDiscreteVariableUpdates(),
or LeafSystem::DoCalcUnrestrictedUpdate()), the dispatcher's return
status should be the returned status of highest severity in a series of event
handlers invoked for simultaneous events. In case of multiple returns at the
same severity, the first one should win. Simultaneous event handler dispatching
should only return early (without processing all events), if an event handler
returns EventStatus::kFailed. */
class EventStatus {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EventStatus)

  /** The numerical values are ordered, with
  did_nothing < success < terminate < fatal. */
  enum Severity {
    /** Successful, but nothing happened; no state update needed. */
    kDidNothing = 0,
    /** Handler executed successfully; state may have been updated. */
    kSucceeded = 1,
    /** Handler succeeded but detected a termination condition (has message). */
    kReachedTermination = 2,
    /** Handler was unable to perform its job (has message). */
    kFailed = 3
  };

  /** Returns "did nothing" status, with no message. */
  static EventStatus DidNothing() { return EventStatus(kDidNothing); }

  /** Returns "succeeded" status, with no message. */
  static EventStatus Succeeded() { return EventStatus(kSucceeded); }

  /** Returns "reached termination" status, with a message explaining why. */
  static EventStatus ReachedTermination(const SystemBase* system,
                                        std::string message) {
    return EventStatus(kReachedTermination, system, std::move(message));
  }

  /** Returns "failed" status, with a message explaining why. */
  static EventStatus Failed(const SystemBase* system, std::string message) {
    return EventStatus(kFailed, system, std::move(message));
  }

  /** Returns the severity of the current status. */
  Severity severity() const { return severity_; }

  /** Returns the optionally-provided subsystem that generated a status
  return that can include a message (reached termination or failed). Returns
  nullptr if no subsystem was provided. */
  const SystemBase* system() const { return system_; }

  /** Returns the optionally-provided human-readable message supplied by the
  event handler that produced the current status. Returns an empty string if
  no message was provided. */
  const std::string& message() const { return message_; }

  /** (Advanced) Replaces the contents of `this` with the more-severe status
  if `candidate` is a more severe status than `this` one. Does nothing if
  `candidate` severity is less than or equal to `this` severity. This method is
  for use in event dispatchers for accumulating status returns from a series of
  event handlers for a set of simultaneous events. */
  EventStatus& KeepMoreSevere(EventStatus candidate) {
    if (candidate.severity() > severity()) *this = candidate;
    return *this;
  }

 private:
  explicit EventStatus(Severity severity) : severity_(severity) {}
  EventStatus(Severity severity, const SystemBase* system, std::string message)
      : severity_(severity), system_(system), message_(std::move(message)) {}

  Severity severity_{kFailed};
  const SystemBase* system_{nullptr};
  std::string message_;
};

}  // namespace systems
}  // namespace drake
