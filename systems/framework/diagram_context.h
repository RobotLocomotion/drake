#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

//==============================================================================
//                                DIAGRAM STATE
//==============================================================================
/// DiagramState is a State, annotated with pointers to all the mutable
/// substates that it spans.
template <typename T>
class DiagramState : public State<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramState)

  /// Constructs a DiagramState consisting of @p size substates.
  explicit DiagramState<T>(int size) :
      State<T>(),
      substates_(size),
      owned_substates_(size) {}

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds. Does not take ownership of @p substate, which must live
  /// as long as this object.
  void set_substate(int index, State<T>* substate) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    substates_[index] = substate;
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds.
  void set_and_own_substate(int index, std::unique_ptr<State<T>> substate) {
    set_substate(index, substate.get());
    owned_substates_[index] = std::move(substate);
  }

  /// Returns the substate at @p index.
  const State<T>& get_substate(int index) const {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Returns the substate at @p index.
  State<T>& get_mutable_substate(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Finalizes this state as a span of all the constituent substates.
  void Finalize() {
    DRAKE_DEMAND(!finalized_);
    finalized_ = true;
    std::vector<ContinuousState<T>*> sub_xcs;
    sub_xcs.reserve(num_substates());
    std::vector<BasicVector<T>*> sub_xds;
    std::vector<AbstractValue*> sub_xas;
    for (State<T>* substate : substates_) {
      // Continuous
      sub_xcs.push_back(&substate->get_mutable_continuous_state());
      // Discrete
      const std::vector<BasicVector<T>*>& xd_data =
          substate->get_mutable_discrete_state().get_data();
      sub_xds.insert(sub_xds.end(), xd_data.begin(), xd_data.end());
      // Abstract
      AbstractValues& xa = substate->get_mutable_abstract_state();
      for (int i_xa = 0; i_xa < xa.size(); ++i_xa) {
        sub_xas.push_back(&xa.get_mutable_value(i_xa));
      }
    }

    // This State consists of a continuous, discrete, and abstract state, each
    // of which is a spanning vector over the continuous, discrete, and abstract
    // parts of the constituent states.  The spanning vectors do not own any
    // of the actual memory that contains state variables. They just hold
    // pointers to that memory.
    this->set_continuous_state(
        std::make_unique<DiagramContinuousState<T>>(sub_xcs));
    this->set_discrete_state(std::make_unique<DiscreteValues<T>>(sub_xds));
    this->set_abstract_state(std::make_unique<AbstractValues>(sub_xas));
  }

 private:
  int num_substates() const {
    return static_cast<int>(substates_.size());
  }

  bool finalized_{false};
  std::vector<State<T>*> substates_;
  std::vector<std::unique_ptr<State<T>>> owned_substates_;
};

//==============================================================================
//                              DIAGRAM CONTEXT
//==============================================================================
/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains contexts and outputs for all the constituent
/// Systems, wired up as specified by calls to `DiagramContext::Connect`.
///
/// In general, users should not need to interact with a DiagramContext
/// directly. Use the accessors on Diagram instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class DiagramContext : public Context<T> {
 public:
  /// @name  Does not allow move or assignent; copy is protected.
  //@{
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
  explicit DiagramContext(int num_subcontexts)
      : contexts_(num_subcontexts),
        state_(std::make_unique<DiagramState<T>>(num_subcontexts)) {}

  /// Returns the number of immediate child subcontexts in this %DiagramContext.
  // This shadows the ContextBase method of the same name to avoid the
  // virtual function call when used locally, but is functionally identical.
  int num_subcontexts() const {
    return static_cast<int>(contexts_.size());
  }

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SubsystemIndex index, std::unique_ptr<Context<T>> context) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] == nullptr);
    context->set_parent(this, index);
    contexts_[index] = std::move(context);
  }

  /// (Internal use only) Declares that a particular input port of a child
  /// subsystem is an input to the entire Diagram that allocates this Context.
  /// Sets up tracking of the child port's dependency on the parent
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportInput(InputPortIndex iport_index,
                   const InputPortIdentifier& subsystem_iport) {
    // Identify and validate the destination input port.
    SubsystemIndex subsystem_index = subsystem_iport.first;
    InputPortIndex subsystem_iport_index = subsystem_iport.second;
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
    DRAKE_DEMAND(0 <= subsystem_iport_index &&
                 subsystem_iport_index < subcontext.get_num_input_ports());

    // Get this Diagram's input port that serves as the source.
    const DependencyTicket iport_ticket =
        this->input_port_tickets()[iport_index];
    DependencyTracker& iport_tracker =
        this->get_mutable_tracker(iport_ticket);
    const DependencyTicket subsystem_iport_ticket =
        subcontext.input_port_tickets()[subsystem_iport_index];
    DependencyTracker& subsystem_iport_tracker =
        subcontext.get_mutable_tracker(subsystem_iport_ticket);
    subsystem_iport_tracker.SubscribeToPrerequisite(&iport_tracker);
  }

  /// (Internal use only) Declares that a particular output port of this
  /// diagram is simply forwarded from an output port of one of its child
  /// subsystems. Sets up tracking of the diagram port's dependency on the child
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportOutput(OutputPortIndex oport_index,
                    const OutputPortIdentifier& subsystem_oport) {
    // Identify and validate the source output port.
    SubsystemIndex subsystem_index = subsystem_oport.first;
    OutputPortIndex subsystem_oport_index = subsystem_oport.second;
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
    DRAKE_DEMAND(0 <= subsystem_oport_index &&
        subsystem_oport_index < subcontext.get_num_output_ports());

    // Get the child subsystem's output port tracker that serves as the source.
    const DependencyTicket subsystem_oport_ticket =
        subcontext.output_port_tickets()[subsystem_oport_index];
    DependencyTracker& subsystem_oport_tracker =
        subcontext.get_mutable_tracker(subsystem_oport_ticket);

    // Get the diagram's output port tracker that is the destination.
    const DependencyTicket oport_ticket =
        this->output_port_tickets()[oport_index];
    DependencyTracker& oport_tracker =
        this->get_mutable_tracker(oport_ticket);

    oport_tracker.SubscribeToPrerequisite(&subsystem_oport_tracker);
  }

  /// (Internal use only) Declares that a connection exists between a peer
  /// output port and input port in this Diagram, records that fact in the
  /// connection map, and registers the input port's dependency
  /// tracker with the output port's dependency tracker. By "peer" we mean that
  /// both ports belong to immediate child subsystems of this Diagram (it is
  /// also possible for both ports to belong to the same subsystem).
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation and cloning only.
  void Connect(const OutputPortIdentifier& oport,
               const InputPortIdentifier& iport) {
    // Identify and validate the source output port.
    SubsystemIndex oport_system_index = oport.first;
    OutputPortIndex oport_index = oport.second;
    Context<T>& oport_context = GetMutableSubsystemContext(oport_system_index);
    DRAKE_DEMAND(oport_index >= 0);
    DRAKE_DEMAND(oport_index < oport_context.get_num_output_ports());

    // Identify and validate the destination input port.
    SubsystemIndex iport_system_index = iport.first;
    InputPortIndex iport_index = iport.second;
    Context<T>& iport_context = GetMutableSubsystemContext(iport_system_index);
    DRAKE_DEMAND(iport_index >= 0);
    DRAKE_DEMAND(iport_index < iport_context.get_num_input_ports());

    DependencyTicket oport_ticket =
        oport_context.output_port_tickets()[oport_index];
    DependencyTicket iport_ticket =
        iport_context.input_port_tickets()[iport_index];
    DependencyTracker& oport_tracker =
        oport_context.get_mutable_tracker(oport_ticket);
    DependencyTracker& iport_tracker =
        iport_context.get_mutable_tracker(iport_ticket);
    iport_tracker.SubscribeToPrerequisite(&oport_tracker);
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void MakeState() {
    auto state = std::make_unique<DiagramState<T>>(num_subcontexts());
    for (int i = 0; i < num_subcontexts(); ++i) {
      Context<T>* context = contexts_[i].get();
      // Using `access` here to avoid sending invalidations.
      state->set_substate(i, &context->access_mutable_state());
    }
    state->Finalize();
    state_ = std::move(state);
  }

  /// Generates the parameters for the entire diagram by wrapping the parameters
  /// of all the constituent Systems. The wrapper simply holds pointers to the
  /// parameters in the subsystem Contexts.  It does not make a copy, or take
  /// ownership.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void MakeParameters() {
    std::vector<BasicVector<T>*> numeric_params;
    std::vector<AbstractValue*> abstract_params;
    for (auto& subcontext : contexts_) {
      Parameters<T>& subparams = subcontext->get_mutable_parameters();
      for (int i = 0; i < subparams.num_numeric_parameters(); ++i) {
        numeric_params.push_back(&subparams.get_mutable_numeric_parameter(i));
      }
      for (int i = 0; i < subparams.num_abstract_parameters(); ++i) {
        abstract_params.push_back(
            &subparams.get_mutable_abstract_parameter(i));
      }
    }
    Parameters<T>& params = this->get_mutable_parameters();
    params.set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    params.set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));
  }

  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  /// TODO(david-german-tri): Rename to get_subsystem_context.
  const Context<T>& GetSubsystemContext(SubsystemIndex index) const {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  /// TODO(david-german-tri): Rename to get_mutable_subsystem_context.
  Context<T>& GetMutableSubsystemContext(SubsystemIndex index) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  DiagramContext(const DiagramContext& source)
      : Context<T>(source),
        contexts_(source.num_subcontexts()),
        state_(std::make_unique<DiagramState<T>>(source.num_subcontexts())) {
    // Clone all the subsystem contexts and outputs.
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      DRAKE_DEMAND(source.contexts_[i] != nullptr);
      AddSystem(i, source.contexts_[i]->CloneWithoutPointers());
    }

    // Build a superstate over the subsystem contexts.
    MakeState();

    // Build superparameters over the subsystem contexts.
    MakeParameters();

    // Everything else was handled by the Context<T> copy constructor.
  }

  DiagramContext<T>* DoCloneWithoutPointers() const final {
    return new DiagramContext<T>(*this);
  }

  /// The caller owns the returned memory.
  State<T>* DoCloneState() const final {
    DiagramState<T>* clone = new DiagramState<T>(num_subcontexts());

    for (int i = 0; i < num_subcontexts(); i++) {
      Context<T>* context = contexts_[i].get();
      clone->set_and_own_substate(i, context->CloneState());
    }

    clone->Finalize();
    return clone;
  }

 private:
  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  // Recursively sets the time on all subcontexts.
  void DoPropagateTimeChange(const T& time_sec, int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      subcontext->PropagateTimeChange(time_sec, change_event);
    }
  }

  // Recursively sets the accuracy on all subcontexts.
  void DoPropagateAccuracyChange(const optional<double>& accuracy,
                                 int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      subcontext->PropagateAccuracyChange(accuracy, change_event);
    }
  }

  // Recursively notifies subcontexts of bulk changes.
  void DoPropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*NoteBulkChange)(int64_t change_event)) {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      subcontext->PropagateBulkChange(change_event, NoteBulkChange);
    }
  }

  int do_num_subcontexts() const final {
    return num_subcontexts();  // That is, the local one.
  }

  // Note the covariant return type here, instead of ContextBase&. That allows
  // us to use this locally to get a Context<T> without downcasting.
  const Context<T>& do_get_subcontext(
      SubsystemIndex index) const final {
    DRAKE_ASSERT(index >= 0 && index < num_subcontexts());
    DRAKE_ASSERT(contexts_[index] != nullptr);
    return *contexts_[index].get();
  }

  // The contexts are stored in SubsystemIndex order, and contexts_ is equal in
  // length to the number of subsystems specified at construction time.
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the Diagram, which includes all its subsystem states.
  std::unique_ptr<DiagramState<T>> state_;
};

}  // namespace systems
}  // namespace drake
