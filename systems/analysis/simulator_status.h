#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

/** Holds the status return value from a call to Simulator::AdvanceTo() and
related methods. The argument t to AdvanceTo(t) is called the boundary time,
and the normal return means that simulated time advanced successfully to t.
Early returns may be caused by a termination condition being met, or by failure
of the System to evaluate some quantity needed by the Simulator. For those
cases, the return object holds a reference to the subsystem that failed
and a message from that subsystem that hopefully explains what happened. */
class SimulatorStatus {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimulatorStatus)

  enum ReturnReason {
    /** This is the normal return: AdvanceTo(t) reached time t. There is no
    message and no saved System. */
    kReachedBoundaryTime,
    /** AdvanceTo() returned early because an event handler returned with a
    "reached termination condition" EventStatus (has message with details). */
    kReachedTerminationCondition,
    /** AdvanceTo() failed because an event handler returned with a "failed"
    EventStatus (has message with details). */
    kEventHandlerFailed
  };

  /** Sets this status to "reached boundary time" with no message and with
  the final time set to the boundary time (this is the same as the
  post-construction default). */
  void SetReachedBoundaryTime() {
    reason_ = kReachedBoundaryTime;
    return_time_ = boundary_time_;
    system_ = nullptr;
    message_.clear();
  }

  /** Sets this status to "reached termination" with the early-termination
  time and a message explaining why. */
  void SetReachedTermination(double termination_time, const SystemBase* system,
                             std::string message) {
    SetResult(termination_time, kReachedTerminationCondition, system,
              std::move(message));
  }

  /** Sets this status to "event handler failed" with the early-termination
  time and a message explaining why. */
  void SetEventHandlerFailed(double failure_time, const SystemBase* system,
                             std::string message) {
    SetResult(failure_time, kEventHandlerFailed, system, std::move(message));
  }

  /** Returns a human-readable message explaining the return result. */
  std::string FormatMessage() const;

  /** Returns true if we reached the boundary time with no surprises. */
  bool succeeded() const { return reason() == kReachedBoundaryTime; }

  /** Returns the maximum time we could have reached with this call; whether
  we actually got there depends on the status. This is the time supplied in
  an AdvanceTo() call or the current time for methods that don't advance
  time. */
  double boundary_time() const { return boundary_time_; }

  /** Returns the time which was actually reached. This will be boundary_time()
  if succeeded() returns true. Otherwise it is the earlier time at which a
  termination or error condition was detected. */
  double return_time() const { return return_time_; }

  /** Returns the reason that a %Simulator call returned. */
  ReturnReason reason() const { return reason_; }

  /** Optionally, returns the subsystem to which the status and contained
  message should be attributed. May be nullptr in which case the status
  should be attributed to the System as a whole. */
  const SystemBase* system() const { return system_; }

  /** For termination or error conditions, returns a human-readable message
  explaining what happened. This is the message from the subsystem that
  detected the condition. FormatMessage() returns additional information and
  also includes this message. */
  const std::string& message() const { return message_; }

  /** Returns true if the `other` status contains exactly the same information
  as `this` status. This is likely only useful for unit testing of
  %SimulatorStatus. */
  bool IsIdenticalStatus(const SimulatorStatus& other) {
    return boundary_time() == other.boundary_time() &&
           return_time() == other.return_time() && reason() == other.reason() &&
           system() == other.system() && message() == other.message();
  }

#ifndef DRAKE_DOXYGEN_CXX
  /* (Internal) For use by Simulator methods, creates a SimulatorStatus that
  assumes we will reach the specified `boundary_time` which should be the
  argument t that was supplied for AdvanceTo(t). */
  explicit SimulatorStatus(double boundary_time)
      : boundary_time_(boundary_time) {
    SetReachedBoundaryTime();
  }
#endif

 private:
  void SetResult(double return_time, ReturnReason reason,
                 const SystemBase* system,
                 std::string message) {
    DRAKE_DEMAND(return_time <= boundary_time_);
    return_time_ = return_time;
    reason_ = reason;
    system_ = system;
    message_ = std::move(message);
  }

  double boundary_time_{};
  double return_time_{};  // Initially set to boundary_time.
  ReturnReason reason_{kReachedBoundaryTime};
  const SystemBase* system_{nullptr};
  std::string message_;
};

}  // namespace systems
}  // namespace drake
