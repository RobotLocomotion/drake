#include "drake/systems/framework/diagram_context.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {

template <typename T>
DiagramContext<T>::DiagramContext(int num_subcontexts)
    : contexts_(num_subcontexts),
      state_(std::make_unique<DiagramState<T>>(num_subcontexts)) {}

template <typename T>
void DiagramContext<T>::AddSystem(
    SubsystemIndex index, std::unique_ptr<Context<T>> context) {
  DRAKE_DEMAND(index >= 0 && index < num_subcontexts());
  DRAKE_DEMAND(contexts_[index] == nullptr);
  ContextBase::set_parent(context.get(), this);
  contexts_[index] = std::move(context);
}

template <typename T>
void DiagramContext<T>::SubscribeExportedInputPortToDiagramPort(
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

template <typename T>
void DiagramContext<T>::SubscribeDiagramPortToExportedOutputPort(
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

template <typename T>
void DiagramContext<T>::SubscribeInputPortToOutputPort(
    const OutputPortIdentifier& output_port,
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

template <typename T>
void DiagramContext<T>::SubscribeDiagramCompositeTrackersToChildrens() {
  // Diagrams don't provide diagram-level tickets for individual
  // discrete or abstract state or individual numerical or abstract parameters.
  // That means we need only subscribe the aggregate trackers xd, xa, pn, pa
  // to their children's xd, xa, pn, pa, resp.
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

template <typename T>
void DiagramContext<T>::MakeState() {
  auto state = std::make_unique<DiagramState<T>>(num_subcontexts());
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
    Context<T>& subcontext = *contexts_[i].get();
    // Using `access` here to avoid sending invalidations.
    state->set_substate(i, &Context<T>::access_mutable_state(&subcontext));
  }
  state->Finalize();
  state->set_system_id(this->get_system_id());
  state_ = std::move(state);
}

template <typename T>
void DiagramContext<T>::MakeParameters() {
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
  params->set_system_id(this->get_system_id());
  this->init_parameters(std::move(params));
}

template <typename T>
DiagramContext<T>::DiagramContext(const DiagramContext& source)
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

template <typename T>
std::unique_ptr<ContextBase> DiagramContext<T>::DoCloneWithoutPointers() const {
  return std::unique_ptr<ContextBase>(new DiagramContext<T>(*this));
}

template <typename T>
std::unique_ptr<State<T>> DiagramContext<T>::DoCloneState() const {
  auto clone = std::make_unique<DiagramState<T>>(num_subcontexts());

  for (SubsystemIndex i(0); i < num_subcontexts(); i++) {
    Context<T>* context = contexts_[i].get();
    clone->set_and_own_substate(i, context->CloneState());
  }

  clone->Finalize();
  return clone;
}

template <typename T>
std::string DiagramContext<T>::do_to_string() const {
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

template <typename T>
void DiagramContext<T>::DoPropagateTimeChange(
    const T& time_sec,
    const std::optional<T>& true_time,
    int64_t change_event) {
  for (auto& subcontext : contexts_) {
    DRAKE_ASSERT(subcontext != nullptr);
    Context<T>::PropagateTimeChange(&*subcontext, time_sec, true_time,
                                    change_event);
  }
}

template <typename T>
void DiagramContext<T>::DoPropagateAccuracyChange(
    const std::optional<double>& accuracy,
    int64_t change_event) {
  for (auto& subcontext : contexts_) {
    DRAKE_ASSERT(subcontext != nullptr);
    Context<T>::PropagateAccuracyChange(&*subcontext, accuracy, change_event);
  }
}

template <typename T>
void DiagramContext<T>::DoPropagateBulkChange(
    int64_t change_event,
    void (ContextBase::*note_bulk_change)(int64_t change_event)) {
  for (auto& subcontext : contexts_) {
    DRAKE_ASSERT(subcontext != nullptr);
    ContextBase::PropagateBulkChange(&*subcontext, change_event,
                                     note_bulk_change);
  }
}

template <typename T>
void DiagramContext<T>::DoPropagateCachingChange(
    void (Cache::*caching_change)()) const {
  for (auto& subcontext : contexts_) {
    DRAKE_ASSERT(subcontext != nullptr);
    ContextBase::PropagateCachingChange(*subcontext, caching_change);
  }
}

template <typename T>
void DiagramContext<T>::DoPropagateBuildTrackerPointerMap(
    const ContextBase& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  auto& clone_diagram = dynamic_cast<const DiagramContext<T>&>(clone);
  DRAKE_DEMAND(clone_diagram.contexts_.size() == contexts_.size());
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
    ContextBase::BuildTrackerPointerMap(
        *contexts_[i], *clone_diagram.contexts_[i], &*tracker_map);
  }
}

template <typename T>
void DiagramContext<T>::DoPropagateFixContextPointers(
    const ContextBase& source,
    const DependencyTracker::PointerMap& tracker_map) {
  auto& source_diagram = dynamic_cast<const DiagramContext<T>&>(source);
  DRAKE_DEMAND(contexts_.size() == source_diagram.contexts_.size());
  for (SubsystemIndex i(0); i < num_subcontexts(); ++i) {
    ContextBase::FixContextPointers(*source_diagram.contexts_[i], tracker_map,
                                    &*contexts_[i]);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramContext)
