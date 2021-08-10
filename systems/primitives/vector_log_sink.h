#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/primitives/vector_log.h"

namespace drake {
namespace systems {

/// A discrete sink block which logs its vector-valued input to per-context
/// memory. This data is then retrievable outside of System operation,
/// e.g. after a simulation. See the warning below.
///
/// The stored log (a VectorLog) holds a large, Eigen matrix for data storage,
/// where each column corresponds to a data point. The VectorLogSink saves a
/// data point and the context time whenever it samples its input.
///
/// @warning The logged data MUST NOT be used to modify the behavior of a
/// simulation. In technical terms, the log is not stored as System State, so
/// should not be considered part of that state. This distinction allows the
/// implementation to use `Publish()` as the event handler, rather than one of
/// the state-modifying handlers.
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
/// publish events, so should be disabled (the default setting) if you want to
/// control logging yourself.
///
/// @see LogVectorOutput() for a convenient way to add %logging to a Diagram.
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
  /// @anchor vector_log_sink_default_triggers
  /// Sets the default set of publish triggers:
  ///   if publish_period = 0, publishes on forced events and per step,
  ///   if publish_period > 0, publishes on forced events and periodically.
  ///
  /// @param input_size Dimension of the (single) input port. This corresponds
  /// to the number of rows of the data matrix.
  /// @param publish_period Period that messages will be published (optional).
  /// If the publish period is zero or not supplied, VectorLogSink will use
  /// per-step publishing instead; see
  /// LeafSystem::DeclarePerStepPublishEvent().
  /// @pre publish_period is non-negative.
  /// @see LogVectorOutput() helper function for a convenient way to add
  /// %logging.
  explicit VectorLogSink(int input_size, double publish_period = 0.0);

  /// Constructs the vector log sink with a specified set of publish triggers.
  ///
  /// @param input_size Dimension of the (single) input port. This corresponds
  /// to the number of rows of the data matrix.
  /// @param publish_triggers Set of triggers that determine when messages will
  /// be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
  /// Will throw an error if empty or if unsupported types are provided.
  /// @param publish_period Period that messages will be published (optional).
  /// publish_period should only be non-zero if one of the publish_triggers is
  /// kPeriodic.
  /// @pre publish_period is non-negative.
  /// @pre publish_period > 0 if and only if publish_triggers contains
  /// kPeriodic.
  /// @see LogVectorOutput() helper function for a convenient way to add
  /// %logging.
  VectorLogSink(int input_size,
                const TriggerTypeSet& publish_triggers,
                double publish_period = 0.0);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit VectorLogSink(const VectorLogSink<U>&);

  /// Access the log within this component's context.
  /// @throws std::exception if context was not created for this system.
  const VectorLog<T>& GetLog(const Context<T>& context) const;

  /// Access the log as a mutable object within this component's context.
  /// @throws std::exception if context was not created for this system.
  VectorLog<T>& GetMutableLog(Context<T>* context) const;

  /// Access the log within a containing root context.
  /// @throws std::exception if supplied context is not a root context, or was
  /// not created for the containing diagram.
  const VectorLog<T>& FindLog(const Context<T>& root_context) const;

  /// Access the log as a mutable object within a containing root context.
  /// @throws std::exception if supplied context is not a root context, or was
  /// not created for the containing diagram.
  VectorLog<T>& FindMutableLog(Context<T>* root_context) const;

 private:
  template <typename> friend class VectorLogSink;

  // Access the mutable vector log stored in the given `context`'s cache entry.
  // @throws std::exception if context was not created for this system.
  VectorLog<T>& GetLogFromCache(const Context<T>& context) const;

  // Remember trigger details for use in scalar conversion.
  TriggerTypeSet publish_triggers_;
  double publish_period_{};

  // Logging is done in this event handler.
  EventStatus WriteToLog(const Context<T>& context) const;

  // The index of a cache entry that stores the log data. It is stored as a
  // cache entry to maintain thread safety in context-per-thread usage.
  CacheIndex log_cache_index_{};
};

/// LogVectorOutput provides a convenience function for adding a VectorLogSink,
/// initialized to the correct size, and connected to an output in a
/// DiagramBuilder. This overload supports the default set of publish triggers.
/// See @ref vector_log_sink_default_triggers "default triggers description".
///
/// @param src the output port to attach logging to.
/// @param builder the diagram builder.
/// @param publish_period Period that messages will be published (optional).
/// If the publish period is zero, VectorLogSink will use per-step
/// publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
/// @pre publish_period is non-negative.
template <typename T>
VectorLogSink<T>* LogVectorOutput(const OutputPort<T>& src,
                                  DiagramBuilder<T>* builder,
                                  double publish_period = 0.0) {
  VectorLogSink<T>* sink =
      builder->template AddSystem<VectorLogSink<T>>(src.size(), publish_period);
  builder->Connect(src, sink->get_input_port());
  return sink;
}

/// LogVectorOutput provides a convenience function for adding a VectorLogSink,
/// initialized to the correct size, and connected to an output in a
/// DiagramBuilder. This overload supports the full variety of publish trigger
/// behavior.
///
/// @param src the output port to attach logging to.
/// @param builder the diagram builder.
/// @param publish_triggers Set of triggers that determine when messages will
/// be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
/// Will throw an error if empty or if unsupported types are provided.
/// @param publish_period Period that messages will be published (optional).
/// publish_period should only be non-zero if one of the publish_triggers is
/// kPeriodic.
/// @pre publish_period is non-negative.
/// @pre publish_period > 0 if and only if publish_triggers contains kPeriodic.
template <typename T>
VectorLogSink<T>* LogVectorOutput(
    const OutputPort<T>& src,
    DiagramBuilder<T>* builder,
    const TriggerTypeSet& publish_triggers,
    double publish_period = 0.0) {
  VectorLogSink<T>* sink =
      builder->template AddSystem<VectorLogSink<T>>(
          src.size(), publish_triggers, publish_period);
  builder->Connect(src, sink->get_input_port());
  return sink;
}

}  // namespace systems
}  // namespace drake
