#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/primitives/signal_log.h"

namespace drake {
namespace systems {

/// Allow coexistence of the deprecated usafe usage, and replacement safe
/// usage. This will be removed after 2021-11-01.
enum LogStorageMode {
  /// Log is stored in the system object. Allows deprecated data access to
  /// work, but is thread-unsafe.
  kDeprecatedLogPerSystem,
  /// Log is stored within a context. Requires a context lookup, but is
  /// thread-safe.
  kLogPerContext
};

/// A discrete sink block which logs its input to memory (see below). This data
/// is then retrievable (e.g. after a simulation).  The stored log (a
/// SignalLog) holds a large, mutable Eigen matrix for data storage, where each
/// column corresponds to a data point. The SignalLogger a data point and the
/// context time whenever it samples its input.
///
/// By default, sampling is performed every time the Simulator completes a
/// trajectory-advancing substep (that is, via a per-step Publish event), with
/// the first sample occurring during Simulator::Initialize(). That means the
/// samples will generally be unevenly spaced in time. If you prefer regular
/// sampling, you may optionally specify a "publish period" in which case
/// sampling occurs periodically, with the first sample occurring at time 0.
/// Alternatively (not common), you can specify that logging should only occur
/// at "forced publish" events, meaning at explicit calls to System::Publish().
/// The Simulator's "publish every time step" option also results in forced
/// publish events, so should be disabled if you want to control logging
/// yourself.
///
/// @warning %SignalLogger's deprecated log storage mode is _not_ thread-safe
/// because it writes to a mutable buffer internally. If you have a Diagram
/// that contains a SignalLogger, even with each thread having its own Context,
/// the threads will conflict.  You should use the log-per-context mode to
/// avoid trouble.
///
/// @see LogOutput() for a convenient way to add %logging to a Diagram.
/// @see Simulator::set_publish_every_time_step()
/// @see Simulator::set_publish_at_initialization()
///
/// @system
/// name: SignalLogger
/// input_ports:
/// - data
/// @endsystem
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class SignalLogger final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignalLogger)

  /// Constructs the signal logger system.
  ///
  /// @warning %SignalLogger may become slower and slower as the number of
  /// log entries increases due to memory reallocations. You can avoid that by
  /// providing a `batch_allocation_size` comparable to the total expected
  /// number of log entries for your simulation.
  ///
  /// @param input_size Dimension of the (single) input port. This corresponds
  /// to the number of rows of the data matrix.
  /// @param batch_allocation_size Storage is (re)allocated in blocks of
  /// input_size-by-batch_allocation_size.
  /// @param storage_mode Which storage mode to use.
  /// @see LogOutput() helper function for a convenient way to add %logging.
  explicit SignalLogger(int input_size, int batch_allocation_size = 1000,
                        LogStorageMode storage_mode = kDeprecatedLogPerSystem);

  /// Constructs the signal logger system. Batch allocation size (see above) is
  /// chosen internally to be a reasonable default value.
  ///
  /// @param input_size Dimension of the (single) input port. This corresponds
  /// to the number of rows of the data matrix.
  /// @param storage_mode Which storage mode to use.
  /// @see LogOutput() helper function for a convenient way to add %logging.
  SignalLogger(int input_size, LogStorageMode storage_mode);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit SignalLogger(const SignalLogger<U>&);

  /// Sets the publishing period of this system to specify periodic sampling
  /// and disables the default per-step sampling. This method can only be called
  /// once and only if set_forced_publish_only() hasn't been called.
  /// @throws std::exception if called more than once, or if
  ///   set_forced_publish_only() has been called.
  /// @pre `period` must be greater than zero.
  void set_publish_period(double period);

  /// Limits logging to forced publish calls only, that is, explicit calls
  /// to System::Publish() issued directly or by the Simulator and disables the
  /// default per-step sampling. This method cannot be called if
  /// set_publish_period() has been called.
  /// @throws std::exception if set_publish_period() has been called.
  void set_forced_publish_only();

  /// Returns the number of samples taken since construction or last reset().
  DRAKE_DEPRECATED("2021-11-01", "Use log-per-context Get*Log APIs instead.")
  int num_samples() const {
    DRAKE_THROW_UNLESS(storage_mode_ == kDeprecatedLogPerSystem);
    return log_->num_samples();
  }

  /// Provides access to the sample times of the logged data. Time is taken
  /// from the Context when the log entry is added.
  DRAKE_DEPRECATED("2021-11-01", "Use log-per-context Get*Log APIs instead.")
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    DRAKE_THROW_UNLESS(storage_mode_ == kDeprecatedLogPerSystem);
    return log_->sample_times();
  }

  /// Provides access to the logged data.
  DRAKE_DEPRECATED("2021-11-01", "Use log-per-context Get*Log APIs instead.")
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true>
  data() const {
    DRAKE_THROW_UNLESS(storage_mode_ == kDeprecatedLogPerSystem);
    return log_->data();
  }

  /// Clears the logged data.
  DRAKE_DEPRECATED("2021-11-01", "Use log-per-context Get*Log APIs instead.")
  void reset() {
    DRAKE_THROW_UNLESS(storage_mode_ == kDeprecatedLogPerSystem);
    log_->reset();
  }

  /// Access the signal log within this component's context.
  const SignalLog<T>& GetLog(const Context<T>& context) const;

  /// Access the signal log within this component's context, given a containing
  /// system and its matching context.
  const SignalLog<T>& GetLog(const System<T>& outer_system,
                             const Context<T>& outer_context) const;
  /// Access the signal log as a mutable object within this component's
  /// context.
  SignalLog<T>& GetMutableLog(const Context<T>& context) const;

  /// Access the signal log as a mutable object within this component's
  /// context, given a containing system and its matching context.
  SignalLog<T>& GetMutableLog(const System<T>& outer_system,
                              const Context<T>& outer_context) const;

 private:
  template <typename> friend class SignalLogger;

  enum LoggingMode { kPerStep, kPeriodic, kForced };

  // Logging is done in this event handler.
  EventStatus WriteToLog(const Context<T>& context) const;

  // This event handler logs only if we're in per-step logging mode.
  EventStatus PerStepWriteToLog(const Context<T>& context) const {
    if (logging_mode_ == kPerStep)
      return WriteToLog(context);
    return EventStatus::DidNothing();
  }

  LoggingMode logging_mode_{kPerStep};

  LogStorageMode storage_mode_{kDeprecatedLogPerSystem};

  // Deprecated per-system storage.
  mutable std::unique_ptr<SignalLog<T>> log_;

  // The index of a cache entry that stores the log data. It is stored as a
  // cache entry to maintain thread safety in context-per-thread usage.
  CacheIndex log_cache_index_{};
};

/// @anchor log_output_example
/// @name LogOutput Example
///
/// LogOutput provides a convenience function for adding a SignalLogger,
/// initialized to the correct size, and connected to an output in a
/// DiagramBuilder.
///
/// @code
///   DiagramBuilder<double> builder;
///   auto foo = builder.AddSystem<Foo>("name", 3.14);
///   auto logger = LogOutput(foo->get_output_port(), &builder);
/// @endcode
/// @relates drake::systems::SignalLogger

/// Attaches a signal logger system to an output in a DiagramBuilder.
/// @see @ref log_output_example for an example of use.
///
/// @warning %SignalLogger may become slower and slower as the number of
/// log entries increases due to memory reallocations. You can avoid that by
/// providing a `batch_allocation_size` comparable to the total expected
/// number of log entries for your simulation.
///
/// @param src the output port to attach logging to.
/// @param builder the diagram builder.
/// @param batch_allocation_size Storage is (re)allocated in blocks of
/// input_size-by-batch_allocation_size.
/// @param storage_mode Which storage mode to use.
/// @see LogOutput() helper function for a convenient way to add %logging.
template <typename T>
SignalLogger<T>* LogOutput(
    const OutputPort<T>& src,
    DiagramBuilder<T>* builder,
    int batch_allocation_size = 1000,
    LogStorageMode storage_mode = kDeprecatedLogPerSystem) {
  SignalLogger<T>* logger =
      builder->template AddSystem<SignalLogger<T>>(
          src.size(), batch_allocation_size, storage_mode);
  builder->Connect(src, logger->get_input_port());
  return logger;
}

/// Attaches a signal logger system to an output in a DiagramBuilder.  Batch
/// allocation size is chosen internally to be a reasonable default value.
/// @see @ref log_output_example for an example of use.
///
/// @param src the output port to attach logging to.
/// @param builder the diagram builder.
/// @param storage_mode Which storage mode to use.
template <typename T>
SignalLogger<T>* LogOutput(
    const OutputPort<T>& src,
    DiagramBuilder<T>* builder,
    LogStorageMode storage_mode) {
  return LogOutput(src, builder, 1000, storage_mode);
}

}  // namespace systems
}  // namespace drake
