#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_state.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {

/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains Context objects for all its constituent Systems.
/// @see Context for more information.
///
/// In general, users should not need to interact with a DiagramContext
/// directly. Use the accessors on Diagram instead.
///
/// @tparam_default_scalar
template <typename T>
class DiagramContext final : public Context<T> {
 public:
  /// @name  Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is protected for use in implementing Clone().
  DiagramContext(DiagramContext&&) = delete;
  DiagramContext& operator=(const DiagramContext&) = delete;
  DiagramContext& operator=(DiagramContext&&) = delete;
  //@}

  /// Identifies a child subsystem's input port.
  using InputPortIdentifier = std::pair<SubsystemIndex, InputPortIndex>;
  /// Identifies a child subsystem's output port.
  using OutputPortIdentifier = std::pair<SubsystemIndex, OutputPortIndex>;

  /// Constructs a DiagramContext with the given @p num_subcontexts, which is
  /// final: you cannot resize a DiagramContext after construction. The
  /// number and ordering of subcontexts is identical to the number and
  /// ordering of subsystems in the corresponding Diagram.
  explicit DiagramContext(int num_subcontexts);

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SubsystemIndex index, std::unique_ptr<Context<T>> context);

  /// (Internal use only) Declares that a particular input port of a child
  /// subsystem is an input to the entire Diagram that allocates this Context.
  /// Sets up tracking of the child port's dependency on the parent
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeExportedInputPortToDiagramPort(
      InputPortIndex input_port_index,
      const InputPortIdentifier& subsystem_input_port);

  /// (Internal use only) Declares that a particular output port of this
  /// diagram is simply forwarded from an output port of one of its child
  /// subsystems. Sets up tracking of the diagram port's dependency on the child
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeDiagramPortToExportedOutputPort(
      OutputPortIndex output_port_index,
      const OutputPortIdentifier& subsystem_output_port);

  /// (Internal use only) Declares that a connection exists between a peer
  /// output port and input port in this Diagram, and registers the input port's
  /// dependency tracker with the output port's dependency tracker. By "peer"
  /// we mean that both ports belong to immediate child subsystems of this
  /// Diagram (it is also possible for both ports to belong to the same
  /// subsystem).
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeInputPortToOutputPort(const OutputPortIdentifier& output_port,
                                      const InputPortIdentifier& input_port);

  /// (Internal use only) Makes the diagram state, parameter, and composite
  /// cache entry trackers subscribe to the corresponding constituent trackers
  /// in the child subcontexts.
  void SubscribeDiagramCompositeTrackersToChildrens();

  /// (Internal use only) Generates the state vector for the entire diagram by
  /// wrapping the states of all the constituent diagrams.
  void MakeState();

  /// (Internal use only) Generates the parameters for the entire diagram by
  /// wrapping the parameters of all the constituent Systems. The wrapper simply
  /// holds pointers to the parameters in the subsystem Contexts. It does not
  /// make a copy, or take ownership.
  void MakeParameters();

  // TODO(david-german-tri): Rename to get_subsystem_context.
  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  const Context<T>& GetSubsystemContext(SubsystemIndex index) const {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

  // TODO(david-german-tri): Rename to get_mutable_subsystem_context.
  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  Context<T>& GetMutableSubsystemContext(SubsystemIndex index) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  DiagramContext(const DiagramContext& source);

 private:
  friend class DiagramContextTest;
  using ContextBase::AddInputPort;    // For DiagramContextTest.
  using ContextBase::AddOutputPort;

  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final;

  std::unique_ptr<State<T>> DoCloneState() const final;

  // Print summary information for the diagram context and recurse into
  // the (non-empty) subcontexts.
  std::string do_to_string() const final;

  // Returns the number of immediate child subcontexts in this DiagramContext.
  int num_subcontexts() const {
    return static_cast<int>(contexts_.size());
  }

  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  // Recursively sets the time on all subcontexts.
  void DoPropagateTimeChange(const T& time_sec,
                             const std::optional<T>& true_time,
                             int64_t change_event) final;

  // Recursively sets the accuracy on all subcontexts.
  void DoPropagateAccuracyChange(const std::optional<double>& accuracy,
                                 int64_t change_event) final;

  // Recursively notifies subcontexts of bulk changes.
  void DoPropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) final;

  // Recursively notifies subcontexts of some caching behavior change.
  void DoPropagateCachingChange(
      void (Cache::*caching_change)()) const final;

  // For this method `this` is the source being copied into `clone`.
  void DoPropagateBuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const final;

  // For this method, `this` is the clone copied from `source`.
  void DoPropagateFixContextPointers(
      const ContextBase& source,
      const DependencyTracker::PointerMap& tracker_map) final;

  // The contexts are stored in SubsystemIndex order, and contexts_ is equal in
  // length to the number of subsystems specified at construction time.
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the Diagram, which includes all its subsystem states.
  std::unique_ptr<DiagramState<T>> state_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramContext)
