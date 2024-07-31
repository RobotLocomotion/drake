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
class EventStatus {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EventStatus);

  /** The numerical values are ordered, with
  did_nothing < success < terminate < fatal. */
  enum Severity {
    /** Successful, but nothing happened; no state update needed. */
    kDidNothing = 0,
    /** Handler executed successfully; state may have been updated. */
    kSucceeded = 1,
    /** Handler succeeded but detected a normal termination condition (has
    message). Intended primarily for internal use by the Simulator. */
    kReachedTermination = 2,
    /** Handler was unable to perform its job (has message). */
    kFailed = 3
  };

  /** Returns "did nothing" status, with no message. */
  static EventStatus DidNothing() { return EventStatus(kDidNothing); }

  /** Returns "succeeded" status, with no message. */
  static EventStatus Succeeded() { return EventStatus(kSucceeded); }

  // TODO(sherm1) Requiring user code to supply the System explicitly
  //  is asking for trouble. The code that invoked the event handler
  //  will be in a better position to fill in the correct System.

  /** Returns "reached termination" status, with a message explaining why. */
  static EventStatus ReachedTermination(const SystemBase* system,
                                        std::string message) {
    return EventStatus(kReachedTermination, system, std::move(message));
  }

  /** Returns "failed" status, with a message explaining why. */
  static EventStatus Failed(const SystemBase* system, std::string message) {
    return EventStatus(kFailed, system, std::move(message));
  }

  /** Returns `true` if the status is DidNothing. */
  bool did_nothing() const { return severity() == kDidNothing; }

  /** Returns `true` if the status is Succeeded. "Did nothing" can
  also be viewed as successful but you have to check for that separately. */
  bool succeeded() const { return severity() == kSucceeded; }

  /** Returns `true` if the status is ReachedTermination. There will also be
  a message() with more detail. */
  bool reached_termination() const { return severity() == kReachedTermination; }

  /** Returns `true` if the status is Failed. There will also be a message()
  with more detail. */
  bool failed() const { return severity() == kFailed; }

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

  /** If failed(), throws an std::exception with a human-readable message.
  @param function_name The name of the user-callable API that encountered
                       the failure. Don't include "()". */
  void ThrowOnFailure(const char* function_name) const;

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
