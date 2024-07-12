/** Instrumentation helpers for in-process control of profile sampling by Linux
 `perf`. See README.md for an overview.
 */
#pragma once

namespace drake {
namespace tools {
namespace performance {

/** In-process controller class.

 This class is designed to work together with the top-level launch script
 perf_controlled_record.py. It looks for the names of two FIFO files via the
 environment variables DRAKE_PERF_CTL_FIFO and DRAKE_PERF_ACK_FIFO. If those
 are missing, or the files can't be opened, then control will be unavailable.

 No attempt is made to make this class thread-safe, so be careful to avoid
 issuing commands from multiple threads.
 */
class PerfController {
 public:
  PerfController();

  /** Turn sampling off. */
  void pause();

  /** Turn sampling on. */
  void resume();

  /** @returns true iff sampling control is available. */
  bool is_control_available() const;

 private:
  void send_command(const char* command, int len);
  int ctl_fd_{-1};
  int ack_fd_{-1};
  bool is_control_available_{false};
};

/** @returns the singleton PerfController. */
PerfController& ThePerfController();

/** A scope guard to turn sampling on for the duration of a scope. Just create
 * an instance at the start of a scope and leave it.
 */
class ScopedPerfSampling {
 public:
  /** The constructor turns sampling on. */
  ScopedPerfSampling() { ThePerfController().resume(); }

  /** The destructor turns sampling off. */
  ~ScopedPerfSampling() { ThePerfController().pause(); }
};

}  // namespace performance
}  // namespace tools
}  // namespace drake
