#pragma once

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

// TODO(sherm1) This System has problems as discussed in issue #10228 that
//              should be fixed.
/// A discrete sink block which logs its input to memory (not thread safe). This
/// data is then retrievable (e.g. after a simulation) via a handful of accessor
/// methods. This system holds a large, mutable Eigen matrix for data storage,
/// where each column corresponds to a data point. It saves a data point and
/// the context time whenever it samples its input.
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
/// @warning %SignalLogger is _not_ thread-safe because it writes to a mutable
/// buffer internally. If you have a Diagram that contains a SignalLogger,
/// even with each thread having its own Context, the threads will conflict.
/// You would have to have separate Diagrams in each thread to avoid trouble.
///
/// @see LogOutput() for a convenient way to add %logging to a Diagram.
/// @see Simulator::set_publish_every_time_step()
/// @see Simulator::set_publish_at_initialization()
///
/// @system{ SignalLogger, @input_port{data}, }
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
  /// @see LogOutput() helper function for a convenient way to add %logging.
  explicit SignalLogger(int input_size, int batch_allocation_size = 1000);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit SignalLogger(const SignalLogger<U>&);

  /// Sets the publishing period of this system to specify periodic sampling
  /// and disables the default per-step sampling. This method can only be called
  /// once and only if set_forced_publish_only() hasn't been called.
  /// @throws std::logic_error if called more than once, or if
  ///   set_forced_publish_only() has been called.
  /// @pre `period` must be greater than zero.
  void set_publish_period(double period);

  /// Limits logging to forced publish calls only, that is, explicit calls
  /// to System::Publish() issued directly or by the Simulator and disables the
  /// default per-step sampling. This method cannot be called if
  /// set_publish_period() has been called.
  /// @throws std::logic_error if set_publish_period() has been called.
  void set_forced_publish_only();

  /// Returns the number of samples taken since construction or last reset().
  int num_samples() const { return log_.num_samples(); }

  /// Provides access to the sample times of the logged data. Time is taken
  /// from the Context when the log entry is added.
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return log_.sample_times();
  }

  /// Provides access to the logged data.
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true>
  data() const {
    return log_.data();
  }

  /// Clears the logged data.
  void reset() { log_.reset(); }

  /// Returns the only input port. The port's name is "data" so you can also
  /// access this with GetInputPort("data").
  const InputPort<T>& get_input_port() const;

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

  mutable SignalLog<T> log_;  // TODO(sherm1) Not thread safe :(
};

/// Provides a convenience function for adding a SignalLogger, initialized to
/// the correct size, and connected to an output in a DiagramBuilder.
///
/// @code
///   DiagramBuilder<double> builder;
///   auto foo = builder.AddSystem<Foo>("name", 3.14);
///   auto logger = LogOutput(foo->get_output_port(), &builder);
/// @endcode
/// @relates drake::systems::SignalLogger

template <typename T>
SignalLogger<T>* LogOutput(const OutputPort<T>& src,
                           systems::DiagramBuilder<T>* builder) {
  SignalLogger<T>* logger =
      builder->template AddSystem<SignalLogger<T>>(src.size());
  builder->Connect(src, logger->get_input_port());
  return logger;
}

}  // namespace systems
}  // namespace drake
