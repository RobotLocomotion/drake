#pragma once

#include <Eigen/Dense>

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {

template <typename T>
class Simulator;

/**
 This system extends the SignalLogger class providing simple playback abilities.
 It is intended to work in conjunction with the DrakeVisualizer. In
 addition to the logging of input to memory, it also provides an output which,
 depending on the state of the system, provides either the passed through
 input value or the cached value.

 This system does not persist the simulation results beyond the scope of the
 Diagram (i.e., nothing written to disk or output in LCM messages).  It is
 simply intended to cache simulation results and then to interactively replay
 them at real time.

 The cached values can be evaluated in a time-continuous manner.  The cached
 samples are interpolated using a zero-order hold (i.e., piecewise constant
 function of time).

 Once this system is set to playback, it cannot be set back to record; the state
 of the simulator's context cannot be guaranteed.

 Typical usage would look like the following:

    - Initialize simulator
       - Place the SignalLoggerPlayback leaf system between the DrakeVisualizer
         system and its upstream dependencies.  What would formerly have been
         the inputs of the DrakeVisualizer system become the inputs of this
         system.  This system's output are connected into the DrakeVisualizer's
         input.
       - Store a pointer to the SignalLoggerPlayback (signal_playback).
    - Define simulation duration: `T`
    - Run simulator to duration time `T` (e.g, `simulator.StepTo(T)`).
    - Initialize playback: `signal_playback->InitializePlayback()`
    - For each playback (e.g., as in a looped playback):
       - Reset playback: `signal_playback->ResetPlayback(&simulator)`
       - Advance the simulator the duration again: `simulator.StepTo(T)`.

 @param T  The vector element type, which must be a valid Eigen scalar.

 Instantiated templates for the following kind of T's are provided:
    - double

 @ingroup primitive_systems
 */
template <typename T>
class SignalLoggerPlayback : public SignalLogger<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignalLoggerPlayback)

  /** Construct the signal logger system.
   @param input_size Dimension of the (single) input port.
   @param batch_allocation_size Storage is (re)allocated in blocks of
   input_size-by-batch_allocation_size.
   */
  explicit SignalLoggerPlayback(int input_size,
                                int batch_allocation_size = 1000);

  /** Reports true if the output data is set by cached data. */
  bool is_playing_back() const { return playback_active_; }

  /** Stops logging and sets the logger to playback; this cannot be undone. */
  void InitializePlayback();

  /** Resets the playback for looping. This must be called prior to each
   simulation loop. This can safely be called redundantly. */
  void ResetPlayback(Simulator<T>* simulator);

 protected:
  /** Sets the output value based on the playback state. */
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  /** Caches the data, unless in playback. */
  void DoPublish(const Context<T>& context) const override;

 private:
  // If true, output is set from cached data.
  bool playback_active_{false};

  // A trajectory pointer that is null when not in playback, non-null when it
  // is.
  PiecewisePolynomial<T> playback_data_{};
};
}  // namespace systems
}  // namespace drake
