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
#include "drake/systems/primitives/vector_log.h"

namespace drake {
namespace systems {

/// A discrete sink block which logs its vector-valued input to per-context
/// memory. This data is then retrievable (e.g. after a simulation).  The
/// stored log (a VectorLog) holds a large, Eigen matrix for data storage,
/// where each column corresponds to a data point. The VectorLogSink saves a
/// data point and the context time whenever it samples its input.
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
/// @see LogVectorOutput() for a convenient way to add %logging to a Diagram.
/// @see Simulator::set_publish_every_time_step()
/// @see Simulator::set_publish_at_initialization()
///
/// @system
/// name: VectorLogSink
/// input_ports:
/// - data
/// @endsystem
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class VectorLogSink final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorLogSink)

  /// Constructs the vector log sink.
  ///
  /// @param input_size Dimension of the (single) input port. This corresponds
  /// to the number of rows of the data matrix.
  /// @see LogVectorOutput() helper function for a convenient way to add
  /// %logging.
  explicit VectorLogSink(int input_size);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit VectorLogSink(const VectorLogSink<U>&);

  /// Sets the publishing period of this system to specify periodic sampling
  /// and disables the default per-step sampling. This method can only be called
  /// once and only if SetForcedPublishOnly() hasn't been called.
  /// @throws std::exception if called more than once, or if
  ///   SetForcedPublishOnly() has been called.
  /// @pre `period` must be greater than zero.
  void SetPublishPeriod(double period);

  /// Limits logging to forced publish calls only, that is, explicit calls
  /// to System::Publish() issued directly or by the Simulator and disables the
  /// default per-step sampling. This method cannot be called if
  /// SetPublishPeriod() has been called.
  /// @throws std::exception if SetPublishPeriod() has been called.
  void SetForcedPublishOnly();

  /// Access the log within this component's context.
  const VectorLog<T>& GetLog(const Context<T>& context) const;

  /// Access the log as a mutable object within this component's context.
  VectorLog<T>& GetMutableLog(const Context<T>& context) const;

  /// Access the log within a containing root context.
  const VectorLog<T>& FindLog(const Context<T>& context) const;

  /// Access the log as a mutable object within a containing root context.
  VectorLog<T>& FindMutableLog(const Context<T>& context) const;

 private:
  template <typename> friend class VectorLogSink;

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

  // The index of a cache entry that stores the log data. It is stored as a
  // cache entry to maintain thread safety in context-per-thread usage.
  CacheIndex log_cache_index_{};
};

/// LogVectorOutput provides a convenience function for adding a VectorLogSink,
/// initialized to the correct size, and connected to an output in a
/// DiagramBuilder.
///
/// @code
///   DiagramBuilder<double> builder;
///   auto foo = builder.AddSystem<Foo>("name", 3.14);
///   auto logger = LogVectorOutput(foo->get_output_port(), &builder);
/// @endcode
/// @relates drake::systems::VectorLogSink
///
/// @param src the output port to attach logging to.
/// @param builder the diagram builder.
template <typename T>
VectorLogSink<T>* LogVectorOutput(const OutputPort<T>& src,
                                  DiagramBuilder<T>* builder) {
  VectorLogSink<T>* sink =
      builder->template AddSystem<VectorLogSink<T>>(src.size());
  builder->Connect(src, sink->get_input_port());
  return sink;
}

}  // namespace systems
}  // namespace drake
