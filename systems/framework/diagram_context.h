#pragma once

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

// TODO(sherm1) This should be in its own file.
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
  explicit DiagramContext(int num_subcontexts)
      : contexts_(num_subcontexts),
        state_(std::make_unique<DiagramState<T>>(num_subcontexts)) {}

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SubsystemIndex index, std::unique_ptr<Context<T>> context) {
    DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
    DRAKE_DEMAND(contexts_[index] == nullptr);
    ContextBase::set_parent(context.get(), this);
    contexts_[index] = std::move(context);
  }

  /// (Internal use only) Declares that a particular input port of a child
  /// subsystem is an input to the entire Diagram that allocates this Context.
  /// Sets up tracking of the child port's dependency on the parent
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeExportedInputPortToDiagramPort(
      InputPortIndex input_port_index,
      const InputPortIdentifier& subsystem_input_port) {
    // Identify and validate the destination input port.
    const SubsystemIndex subsystem_index = subsystem_input_port.first;
    const InputPortIndex subsystem_iport_index = subsystem_input_port.second;
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
    DRAKE_DEMAND(0 <= subsystem_iport_index &&
                 subsystem_iport_index < subcontext.num_input_ports());

    // Get this Diagram's input port that serves as the source.
    const DependencyTicket iport_ticket =
        this->input_port_ticket(input_port_index);
    DependencyTracker& iport_tracker = this->get_mutable_tracker(iport_ticket);
    const DependencyTicket subcontext_iport_ticket =
        subcontext.input_port_ticket(subsystem_iport_index);
    DependencyTracker& subcontext_iport_tracker =
        subcontext.get_mutable_tracker(subcontext_iport_ticket);
    subcontext_iport_tracker.SubscribeToPrerequisite(&iport_tracker);
  }

  /// (Internal use only) Declares that a particular output port of this
  /// diagram is simply forwarded from an output port of one of its child
  /// subsystems. Sets up tracking of the diagram port's dependency on the child
  /// port. Aborts if the subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void SubscribeDiagramPortToExportedOutputPort(
      OutputPortIndex output_port_index,
      const OutputPortIdentifier& subsystem_output_port) {
    // Identify and validate the source output port.
    const SubsystemIndex subsystem_index = subsystem_output_port.first;
    const OutputPortIndex subsystem_oport_index = subsystem_output_port.second;
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem_index);
    DRAKE_DEMAND(0 <= subsystem_oport_index &&
                 subsystem_oport_index < subcontext.num_output_ports());

    // Get the child subsystem's output port tracker that serves as the source.
    const DependencyTicket subcontext_oport_ticket =
        subcontext.output_port_ticket(subsystem_oport_index);
    DependencyTracker& subcontext_oport_tracker =
        subcontext.get_mutable_tracker(subcontext_oport_ticket);

    // Get the diagram's output port tracker that is the destination.
    const DependencyTicket oport_ticket =
        this->output_port_ticket(output_port_index);
    DependencyTracker& oport_tracker = this->get_mutable_tracker(oport_ticket);

    oport_tracker.SubscribeToPrerequisite(&subcontext_oport_tracker);
  }

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
                                      const InputPortIdentifier& input_port) {
    // Identify and validate the source output port.
    const SubsystemIndex oport_system_index = output_port.first;
    const OutputPortIndex oport_index = output_port.second;
    Context<T>& oport_context = GetMutableSubsystemContext(oport_system_index);
    DRAKE_DEMAND(oport_index >= 0);
    DRAKE_DEMAND(oport_index < oport_context.num_output_ports());

    // Identify and validate the destination input port.
    const SubsystemIndex iport_system_index = input_port.first;
    const InputPortIndex iport_index = input_port.second;
    Context<T>& iport_context = GetMutableSubsystemContext(iport_system_index);
    DRAKE_DEMAND(iport_index >= 0);
    DRAKE_DEMAND(iport_index < iport_context.num_input_ports());

    // Dig out the dependency trackers for both ports so we can subscribe the
    // input port tracker to the output port tracker.
    const DependencyTicket oport_ticket =
        oport_context.output_port_ticket(oport_index);
    const DependencyTicket iport_ticket =
        iport_context.input_port_ticket(iport_index);
    DependencyTracker& oport_tracker =
        oport_context.get_mutable_tracker(oport_ticket);
    DependencyTracker& iport_tracker =
        iport_context.get_mutable_tracker(iport_ticket);
    iport_tracker.SubscribeToPrerequisite(&oport_tracker);
  }

  /// (Internal use only) Makes the diagram state, parameter, and composite
  /// cache entry trackers subscribe to the corresponding constituent trackers
  /// in the child subcontexts.
  // Diagrams don't provide diagram-level tickets for individual
  // discrete or abstract state or individual numerical or abstract parameters.
  // That means we need only subscribe the aggregate trackers xd, xa, pn, pa
  // to their children's xd, xa, pn, pa, resp.
  void SubscribeDiagramCompositeTrackersToChildrens() {
    std::vector<internal::BuiltInTicketNumbers> composites{
        internal::kQTicket,  // Value sources.
        internal::kVTicket,
        internal::kZTicket,
        internal::kXdTicket,
        internal::kXaTicket,
        internal::kPnTicket,
        internal::kPaTicket,
        internal::kXcdotTicket,  // Cache entries.
        internal::kPeTicket,
        internal::kKeTicket,
        internal::kPcTicket,
        internal::kPncTicket};

    // Validate the claim above that Diagrams do not have tickets for individual
    // variables and parameters.
    DRAKE_DEMAND(!this->owns_any_variables_or_parameters());

    // Collect the diagram trackers for each composite item above.
    DependencyGraph& graph = this->get_mutable_dependency_graph();
    std::vector<DependencyTracker*> diagram_trackers;
    for (auto ticket : composites) {
      diagram_trackers.push_back(
          &graph.get_mutable_tracker(DependencyTicket(ticket)));
    }

    // Subscribe those trackers to the corresponding subcontext trackers.
    for (auto& subcontext : contexts_) {
      DependencyGraph& subgraph = subcontext->get_mutable_dependency_graph();
      for (size_t i = 0; i < composites.size(); ++i) {
        auto& sub_tracker =
            subgraph.get_mutable_tracker(DependencyTicket(composites[i]));
        diagram_trackers[i]->SubscribeToPrerequisite(&sub_tracker);
      }
    }
  }

  /// (Internal use only) Generates the state vector for the entire diagram by
  /// wrapping the states of all the constituent diagrams.
  void MakeState() {
    auto state = std::make_unique<DiagramState<T>>(num_subcontexts());
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      Context<T>& subcontext = *contexts_[i].get();
      // Using `access` here to avoid sending invalidations.
      state->set_substate(i, &Context<T>::access_mutable_state(&subcontext));
    }
    state->Finalize();
    state->get_mutable_continuous_state().set_system_id(this->get_system_id());
    state_ = std::move(state);
  }

  /// (Internal use only) Generates the parameters for the entire diagram by
  /// wrapping the parameters of all the constituent Systems. The wrapper simply
  /// holds pointers to the parameters in the subsystem Contexts. It does not
  /// make a copy, or take ownership.
  void MakeParameters() {
    std::vector<BasicVector<T>*> numeric_params;
    std::vector<AbstractValue*> abstract_params;
    for (auto& subcontext : contexts_) {
      // Using `access` here to avoid sending invalidations.
      Parameters<T>& subparams =
          Context<T>::access_mutable_parameters(&*subcontext);
      for (int i = 0; i < subparams.num_numeric_parameter_groups(); ++i) {
        numeric_params.push_back(&subparams.get_mutable_numeric_parameter(i));
      }
      for (int i = 0; i < subparams.num_abstract_parameters(); ++i) {
        abstract_params.push_back(&subparams.get_mutable_abstract_parameter(i));
      }
    }
    auto params = std::make_unique<Parameters<T>>();
    params->set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    params->set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));
    this->init_parameters(std::move(params));
  }

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
  DiagramContext(const DiagramContext& source)
      : Context<T>(source),
        contexts_(source.num_subcontexts()),
        state_(std::make_unique<DiagramState<T>>(source.num_subcontexts())) {
    // Clone all the subsystem contexts.
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      DRAKE_DEMAND(source.contexts_[i] != nullptr);
      AddSystem(i, Context<T>::CloneWithoutPointers(*source.contexts_[i]));
    }

    // Build a superstate over the subsystem contexts.
    MakeState();

    // Build superparameters over the subsystem contexts.
    MakeParameters();

    // Everything else was handled by the Context<T> copy constructor.
  }

 private:
  friend class DiagramContextTest;
  using ContextBase::AddInputPort;    // For DiagramContextTest.
  using ContextBase::AddOutputPort;

  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<ContextBase>(new DiagramContext<T>(*this));
  }

  std::unique_ptr<State<T>> DoCloneState() const final {
    auto clone = std::make_unique<DiagramState<T>>(num_subcontexts());

    for (SubsystemIndex i(0); i < num_subcontexts(); i++) {
      Context<T>* context = contexts_[i].get();
      clone->set_and_own_substate(i, context->CloneState());
    }

    clone->Finalize();
    return clone;
  }

  // Print summary information for the diagram context and recurse into
  // the (non-empty) subcontexts.
  std::string do_to_string() const final {
    std::ostringstream os;

    os << this->GetSystemPathname() << " Context (of a Diagram)\n";
    os << std::string(this->GetSystemPathname().size() + 24, '-') << "\n";
    if (this->num_continuous_states())
      os << this->num_continuous_states() << " total continuous states\n";
    if (this->num_discrete_state_groups()) {
      int num_discrete_states = 0;
      for (int i = 0; i < this->num_discrete_state_groups(); i++) {
        num_discrete_states += this->get_discrete_state(i).size();
      }
      os << num_discrete_states << " total discrete states in "
         << this->num_discrete_state_groups() << " groups\n";
    }
    if (this->num_abstract_states())
      os << this->num_abstract_states() << " total abstract states\n";

    if (this->num_numeric_parameter_groups()) {
      int num_numeric_parameters = 0;
      for (int i = 0; i < this->num_numeric_parameter_groups(); i++) {
        num_numeric_parameters += this->get_numeric_parameter(i).size();
      }
      os << num_numeric_parameters << " total numeric parameters in "
         << this->num_numeric_parameter_groups() << " groups\n";
    }
    if (this->num_abstract_parameters())
      os << this->num_abstract_parameters() << " total abstract parameters\n";

    for (systems::SubsystemIndex i{0}; i < num_subcontexts(); i++) {
      const Context<T>& subcontext = this->GetSubsystemContext(i);
      // Only print this context if it has something useful to print.
      if (subcontext.get_continuous_state_vector().size() ||
          subcontext.num_discrete_state_groups() ||
          subcontext.num_abstract_states() ||
          subcontext.num_numeric_parameter_groups() ||
          subcontext.num_abstract_parameters()) {
        os << "\n" << subcontext.to_string();
      }
    }

    return os.str();
  }

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
  void DoPropagateTimeChange(const T& time_sec, int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      Context<T>::PropagateTimeChange(&*subcontext, time_sec, change_event);
    }
  }

  // Recursively sets the accuracy on all subcontexts.
  void DoPropagateAccuracyChange(const std::optional<double>& accuracy,
                                 int64_t change_event) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      Context<T>::PropagateAccuracyChange(&*subcontext, accuracy, change_event);
    }
  }

  // Recursively notifies subcontexts of bulk changes.
  void DoPropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      ContextBase::PropagateBulkChange(&*subcontext, change_event,
                                       note_bulk_change);
    }
  }

  // Recursively notifies subcontexts of some caching behavior change.
  void DoPropagateCachingChange(
      void (Cache::*caching_change)()) const final {
    for (auto& subcontext : contexts_) {
      DRAKE_ASSERT(subcontext != nullptr);
      ContextBase::PropagateCachingChange(*subcontext, caching_change);
    }
  }

  // For this method `this` is the source being copied into `clone`.
  void DoPropagateBuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const final {
    auto& clone_diagram = dynamic_cast<const DiagramContext<T>&>(clone);
    DRAKE_DEMAND(clone_diagram.contexts_.size() == contexts_.size());
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      ContextBase::BuildTrackerPointerMap(
          *contexts_[i], *clone_diagram.contexts_[i], &*tracker_map);
    }
  }

  // For this method, `this` is the clone copied from `source`.
  void DoPropagateFixContextPointers(
      const ContextBase& source,
      const DependencyTracker::PointerMap& tracker_map) final {
    auto& source_diagram = dynamic_cast<const DiagramContext<T>&>(source);
    DRAKE_DEMAND(contexts_.size() == source_diagram.contexts_.size());
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
      ContextBase::FixContextPointers(*source_diagram.contexts_[i], tracker_map,
                                      &*contexts_[i]);
    }
  }

  // The contexts are stored in SubsystemIndex order, and contexts_ is equal in
  // length to the number of subsystems specified at construction time.
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the Diagram, which includes all its subsystem states.
  std::unique_ptr<DiagramState<T>> state_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramState)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramContext)
