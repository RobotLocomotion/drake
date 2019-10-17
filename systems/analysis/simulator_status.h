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
of the System to evaluate some quantity needed by the Simulator. In the latter
cases the return object holds a reference to the subsystem that failed,
and a message from that subsystem that hopefully explains what happened. */
class SimulatorStatus {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimulatorStatus)

  /** Creates a SimulatorStatus that assumes we will reach the specified
  `boundary_time` which should be the argument t that was supplied for
  AdvanceTo(t). */
  explicit SimulatorStatus(double boundary_time)
      : boundary_time_(boundary_time) {
    SetReachedBoundaryTime();
  }

  enum ReturnReason {
    /** This is the normal return; AdvanceTo(t) reached time t. There is no
    message and no saved System. */
    kReachedBoundaryTime = 0,
    /** AdvanceTo() returned early because an event handler returned with a
    "reached termination condition" result (has message with details). */
    kReachedTerminationCondition = 1,
    /** AdvanceTo() failed because an event handler failed. */
    kEventHandlerFailed = 2
  };

  /** Sets this status to "reached boundary time" with no message and with
  the final time set to the boundary time (this is the same as the
  post-construction default). */
  void SetReachedBoundaryTime() {
    reason_ = kReachedBoundaryTime;
    final_time_ = boundary_time_;
    system_ = nullptr;
    message_.clear();
  }

  /** Sets this status to "reached termination" with the early-termination
  time and a message explaining why. */
  void SetReachedTermination(double final_time, const SystemBase* system,
                             std::string message) {
    SetResult(final_time, kReachedTerminationCondition, system,
              std::move(message));
  }

  /** Sets this status to "event handler failed" with the early-termination
  time and a message explaining why. */
  void SetEventHandlerFailed(double final_time, const SystemBase* system,
                             std::string message) {
    SetResult(final_time, kEventHandlerFailed, system, std::move(message));
  }

  /** Returns a human-readable message explaining the return result. */
  std::string FormatMessage() const;

  bool succeeded() const { return reason() == kReachedBoundaryTime; }
  double boundary_time() const { return boundary_time_; }
  double final_time() const { return final_time_; }
  ReturnReason reason() const { return reason_; }
  const SystemBase* system() const { return system_; }
  const std::string& message() const { return message_; }

 private:
  void SetResult(double final_time, ReturnReason reason,
                 const SystemBase* system,
                 std::string message) {
    DRAKE_DEMAND(final_time <= boundary_time_);
    final_time_ = final_time;
    reason_ = reason;
    system_ = system;
    message_ = std::move(message);
  }

  double boundary_time_{};
  double final_time_{};  // Initially set to boundary_time.
  ReturnReason reason_{kReachedBoundaryTime};
  const SystemBase* system_{nullptr};
  std::string message_;
};

}  // namespace systems
}  // namespace drake
