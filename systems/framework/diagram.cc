#include "drake/systems/framework/diagram.h"

#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_set>

#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"
#include "drake/common/string_unordered_set.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_visitor.h"
#include "drake/systems/framework/wrapped_system.h"

namespace drake {
namespace systems {

template <typename T>
Diagram<T>::~Diagram() {}

template <typename T>
std::vector<const systems::System<T>*> Diagram<T>::GetSystems() const {
  std::vector<const systems::System<T>*> result;
  result.reserve(registered_systems_.size());
  for (const auto& system : registered_systems_) {
    result.push_back(system.get());
  }
  return result;
}

template <typename T>
void Diagram<T>::Accept(SystemVisitor<T>* v) const {
  DRAKE_DEMAND(v != nullptr);
  v->VisitDiagram(*this);
}

template <typename T>
const std::map<typename Diagram<T>::InputPortLocator,
               typename Diagram<T>::OutputPortLocator>&
Diagram<T>::connection_map() const {
  return connection_map_;
}

template <typename T>
std::vector<typename Diagram<T>::InputPortLocator>
Diagram<T>::GetInputPortLocators(InputPortIndex port_index) const {
  DRAKE_DEMAND(port_index >= 0 && port_index < this->num_input_ports());
  std::vector<typename Diagram<T>::InputPortLocator> result;
  for (const auto& map_pair : input_port_map_) {
    if (map_pair.second == port_index) {
      result.push_back(map_pair.first);
    }
  }
  return result;
}

template <typename T>
typename Diagram<T>::InputPortLocator Diagram<T>::GetArbitraryInputPortLocator(
    InputPortIndex port_index) const {
  DRAKE_DEMAND(port_index >= 0 && port_index < this->num_input_ports());
  const auto ids = GetInputPortLocators(port_index);
  DRAKE_ASSERT(!ids.empty());
  return ids[0];
}

template <typename T>
const typename Diagram<T>::OutputPortLocator&
Diagram<T>::get_output_port_locator(OutputPortIndex port_index) const {
  DRAKE_DEMAND(port_index >= 0 &&
               port_index < static_cast<int>(output_port_ids_.size()));
  return output_port_ids_[port_index];
}

template <typename T>
std::multimap<int, int> Diagram<T>::GetDirectFeedthroughs() const {
  std::unordered_map<const System<T>*, std::multimap<int, int>> memoize;
  std::multimap<int, int> pairs;
  for (InputPortIndex u(0); u < this->num_input_ports(); ++u) {
    for (OutputPortIndex v(0); v < this->num_output_ports(); ++v) {
      if (DiagramHasDirectFeedthrough(u, v, &memoize)) {
        pairs.emplace(u, v);
      }
    }
  }
  return pairs;
}

template <typename T>
std::unique_ptr<CompositeEventCollection<T>>
Diagram<T>::DoAllocateCompositeEventCollection() const {
  const int num_systems = num_subsystems();
  std::vector<std::unique_ptr<CompositeEventCollection<T>>> subevents(
      num_systems);
  for (SubsystemIndex i(0); i < num_systems; ++i) {
    subevents[i] = registered_systems_[i]->AllocateCompositeEventCollection();
  }

  return std::make_unique<DiagramCompositeEventCollection<T>>(
      std::move(subevents));
}

template <typename T>
void Diagram<T>::SetDefaultParameters(const Context<T>& context,
                                      Parameters<T>* params) const {
  this->ValidateContext(context);
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  this->ValidateCreatedForThisSystem(params);

  int numeric_parameter_offset = 0;
  int abstract_parameter_offset = 0;

  // Set default parameters of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);

    if (!subcontext.num_numeric_parameter_groups() &&
        !subcontext.num_abstract_parameters()) {
      // Then there is no work to do for this subcontext.
      continue;
    }

    // Make a new Parameters<T> structure with pointers to the mutable
    // subsystem parameter values.  This does not make a copy of the
    // underlying data.
    // TODO(russt): Consider implementing a DiagramParameters, analogous to
    // DiagramState, to avoid these dynamic allocations if they prove
    // expensive.

    std::vector<BasicVector<T>*> numeric_params;
    for (int j = 0; j < subcontext.num_numeric_parameter_groups(); ++j) {
      numeric_params.push_back(
          &params->get_mutable_numeric_parameter(numeric_parameter_offset + j));
    }
    numeric_parameter_offset += subcontext.num_numeric_parameter_groups();

    std::vector<AbstractValue*> abstract_params;
    for (int j = 0; j < subcontext.num_abstract_parameters(); ++j) {
      abstract_params.push_back(&params->get_mutable_abstract_parameter(
          abstract_parameter_offset + j));
    }
    abstract_parameter_offset += subcontext.num_abstract_parameters();

    Parameters<T> subparameters;
    subparameters.set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    subparameters.set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));
    subparameters.set_system_id(subcontext.get_system_id());

    registered_systems_[i]->SetDefaultParameters(subcontext, &subparameters);
  }
}

template <typename T>
void Diagram<T>::SetDefaultState(const Context<T>& context,
                                 State<T>* state) const {
  this->ValidateContext(context);
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  this->ValidateCreatedForThisSystem(state);
  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  // Set default state of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);
    auto& substate = diagram_state->get_mutable_substate(i);
    registered_systems_[i]->SetDefaultState(subcontext, &substate);
  }
}

template <typename T>
void Diagram<T>::SetRandomState(const Context<T>& context, State<T>* state,
                                RandomGenerator* generator) const {
  this->ValidateContext(context);
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  this->ValidateCreatedForThisSystem(state);
  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  // Set state of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);
    auto& substate = diagram_state->get_mutable_substate(i);
    registered_systems_[i]->SetRandomState(subcontext, &substate, generator);
  }
}

template <typename T>
void Diagram<T>::SetRandomParameters(const Context<T>& context,
                                     Parameters<T>* params,
                                     RandomGenerator* generator) const {
  this->ValidateContext(context);
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  this->ValidateCreatedForThisSystem(params);

  int numeric_parameter_offset = 0;
  int abstract_parameter_offset = 0;

  // Set parameters of each constituent system.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    auto& subcontext = diagram_context->GetSubsystemContext(i);

    if (!subcontext.num_numeric_parameter_groups() &&
        !subcontext.num_abstract_parameters()) {
      // Then there is no work to do for this subcontext.
      continue;
    }

    // Make a new Parameters<T> structure with pointers to the mutable
    // subsystem parameter values.  This does not make a copy of the
    // underlying data.
    // TODO(russt): This code is duplicated from SetDefaultParameters.
    // Consider extracting it to a helper method (waiting for the rule of
    // three).

    std::vector<BasicVector<T>*> numeric_params;
    std::vector<AbstractValue*> abstract_params;
    for (int j = 0; j < subcontext.num_numeric_parameter_groups(); ++j) {
      numeric_params.push_back(
          &params->get_mutable_numeric_parameter(numeric_parameter_offset + j));
    }
    numeric_parameter_offset += subcontext.num_numeric_parameter_groups();
    for (int j = 0; j < subcontext.num_abstract_parameters(); ++j) {
      abstract_params.push_back(&params->get_mutable_abstract_parameter(
          abstract_parameter_offset + j));
    }
    abstract_parameter_offset += subcontext.num_abstract_parameters();
    Parameters<T> subparameters;
    subparameters.set_numeric_parameters(
        std::make_unique<DiscreteValues<T>>(numeric_params));
    subparameters.set_abstract_parameters(
        std::make_unique<AbstractValues>(abstract_params));
    subparameters.set_system_id(subcontext.get_system_id());

    registered_systems_[i]->SetRandomParameters(subcontext, &subparameters,
                                                generator);
  }
}

template <typename T>
std::unique_ptr<EventCollection<PublishEvent<T>>>
Diagram<T>::AllocateForcedPublishEventCollection() const {
  return AllocateForcedEventCollection<PublishEvent<T>>(
      &System<T>::AllocateForcedPublishEventCollection);
}

template <typename T>
std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
Diagram<T>::AllocateForcedDiscreteUpdateEventCollection() const {
  return AllocateForcedEventCollection<DiscreteUpdateEvent<T>>(
      &System<T>::AllocateForcedDiscreteUpdateEventCollection);
}

template <typename T>
std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
Diagram<T>::AllocateForcedUnrestrictedUpdateEventCollection() const {
  return AllocateForcedEventCollection<UnrestrictedUpdateEvent<T>>(
      &System<T>::AllocateForcedUnrestrictedUpdateEventCollection);
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Diagram<T>::AllocateTimeDerivatives()
    const {
  std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
  for (const auto& system : registered_systems_) {
    sub_derivatives.push_back(system->AllocateTimeDerivatives());
  }
  auto result =
      std::make_unique<DiagramContinuousState<T>>(std::move(sub_derivatives));
  result->set_system_id(this->get_system_id());
  return result;
}

template <typename T>
std::unique_ptr<DiscreteValues<T>> Diagram<T>::AllocateDiscreteVariables()
    const {
  std::vector<std::unique_ptr<DiscreteValues<T>>> sub_discretes;
  for (const auto& system : registered_systems_) {
    sub_discretes.push_back(system->AllocateDiscreteVariables());
  }
  auto result =
      std::make_unique<DiagramDiscreteValues<T>>(std::move(sub_discretes));
  result->set_system_id(this->get_system_id());
  return result;
}

template <typename T>
void Diagram<T>::DoCalcTimeDerivatives(const Context<T>& context,
                                       ContinuousState<T>* derivatives) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  auto diagram_derivatives =
      dynamic_cast<DiagramContinuousState<T>*>(derivatives);
  DRAKE_DEMAND(diagram_derivatives != nullptr);
  const int n = diagram_derivatives->num_substates();
  DRAKE_DEMAND(num_subsystems() == n);

  // Evaluate the derivatives of each constituent system.
  for (SubsystemIndex i(0); i < n; ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    ContinuousState<T>& subderivatives =
        diagram_derivatives->get_mutable_substate(i);
    registered_systems_[i]->CalcTimeDerivatives(subcontext, &subderivatives);
  }
}

template <typename T>
void Diagram<T>::DoCalcImplicitTimeDerivativesResidual(
    const Context<T>& context, const ContinuousState<T>& proposed_derivatives,
    EigenPtr<VectorX<T>> residual) const {
  // Check that the arguments are at least structurally compatible with
  // this Diagram.
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);
  const auto* diagram_derivatives =
      dynamic_cast<const DiagramContinuousState<T>*>(&proposed_derivatives);
  DRAKE_DEMAND(diagram_derivatives != nullptr);
  const int n = diagram_derivatives->num_substates();
  DRAKE_DEMAND(num_subsystems() == n);

  // The public method has already verified that the output vector is the right
  // length.

  // Evaluate the residuals from each constituent system.
  int next = 0;  // Next available slot in residual vector.
  for (SubsystemIndex i(0); i < n; ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    const ContinuousState<T>& subderivatives =
        diagram_derivatives->get_substate(i);
    const System<T>& subsystem = *registered_systems_[i];
    const int num_sub_residuals =
        subsystem.implicit_time_derivatives_residual_size();

    // The "const" here is for the returned object; the data to which it
    // refers is still mutable because residual is.
    const auto segment = residual->segment(next, num_sub_residuals);
    subsystem.CalcImplicitTimeDerivativesResidual(subcontext, subderivatives,
                                                  &segment);
    next += num_sub_residuals;
  }

  // Make sure we wrote to every element.
  DRAKE_DEMAND(next == residual->size());
}

template <typename T>
bool Diagram<T>::HasSubsystemNamed(std::string_view name) const {
  for (const auto& child : registered_systems_) {
    if (child->get_name() == name) {
      return true;
    }
  }
  return false;
}

template <typename T>
const System<T>& Diagram<T>::GetSubsystemByName(std::string_view name) const {
  for (const auto& child : registered_systems_) {
    if (child->get_name() == name) {
      return *child;
    }
  }
  std::vector<std::string> subsystem_names;
  subsystem_names.reserve(registered_systems_.size());
  for (const auto& child : registered_systems_) {
    subsystem_names.emplace_back(child->get_name());
  }

  throw std::logic_error(fmt::format(
      "System {} does not have a subsystem named {}. The existing subsystems "
      "are named {{{}}}.",
      this->GetSystemName(), name, fmt::join(subsystem_names, ", ")));
}

template <typename T>
const ContinuousState<T>& Diagram<T>::GetSubsystemDerivatives(
    const System<T>& subsystem, const ContinuousState<T>& derivatives) const {
  this->ValidateCreatedForThisSystem(derivatives);
  auto diagram_derivatives =
      dynamic_cast<const DiagramContinuousState<T>*>(&derivatives);
  DRAKE_DEMAND(diagram_derivatives != nullptr);
  const SubsystemIndex i = GetSystemIndexOrAbort(&subsystem);
  return diagram_derivatives->get_substate(i);
}

template <typename T>
const DiscreteValues<T>& Diagram<T>::GetSubsystemDiscreteValues(
    const System<T>& subsystem,
    const DiscreteValues<T>& discrete_values) const {
  this->ValidateCreatedForThisSystem(discrete_values);
  auto diagram_discrete_state =
      dynamic_cast<const DiagramDiscreteValues<T>*>(&discrete_values);
  DRAKE_DEMAND(diagram_discrete_state != nullptr);
  const SubsystemIndex i = GetSystemIndexOrAbort(&subsystem);
  return diagram_discrete_state->get_subdiscrete(i);
}

template <typename T>
const CompositeEventCollection<T>&
Diagram<T>::GetSubsystemCompositeEventCollection(
    const System<T>& subsystem,
    const CompositeEventCollection<T>& events) const {
  this->ValidateCreatedForThisSystem(events);
  auto ret = DoGetTargetSystemCompositeEventCollection(subsystem, &events);
  DRAKE_DEMAND(ret != nullptr);
  return *ret;
}

template <typename T>
CompositeEventCollection<T>&
Diagram<T>::GetMutableSubsystemCompositeEventCollection(
    const System<T>& subsystem, CompositeEventCollection<T>* events) const {
  this->ValidateCreatedForThisSystem(events);
  auto ret =
      DoGetMutableTargetSystemCompositeEventCollection(subsystem, events);
  DRAKE_DEMAND(ret != nullptr);
  return *ret;
}

template <typename T>
State<T>& Diagram<T>::GetMutableSubsystemState(const System<T>& subsystem,
                                               Context<T>* context) const {
  this->ValidateContext(context);
  Context<T>& subcontext = GetMutableSubsystemContext(subsystem, context);
  return subcontext.get_mutable_state();
}

template <typename T>
State<T>& Diagram<T>::GetMutableSubsystemState(const System<T>& subsystem,
                                               State<T>* state) const {
  this->ValidateCreatedForThisSystem(state);
  auto ret = DoGetMutableTargetSystemState(subsystem, state);
  DRAKE_DEMAND(ret != nullptr);
  return *ret;
}

template <typename T>
const State<T>& Diagram<T>::GetSubsystemState(const System<T>& subsystem,
                                              const State<T>& state) const {
  this->ValidateCreatedForThisSystem(state);
  auto ret = DoGetTargetSystemState(subsystem, &state);
  DRAKE_DEMAND(ret != nullptr);
  return *ret;
}

namespace {
// Filter the options per the subsystem's name.
std::map<std::string, std::string> FilterGraphvizOptions(
    const std::map<std::string, std::string>& diagram_options,
    const SystemBase& subsystem) {
  const std::string key_prefix = subsystem.get_name() + "/";
  const size_t key_prefix_size = key_prefix.size();
  std::map<std::string, std::string> result;
  for (const auto& [key, value] : diagram_options) {
    if (key.substr(0, key_prefix_size) == key_prefix) {
      result.emplace(key.substr(key_prefix_size), value);
    }
  }
  return result;
}
}  // namespace

template <typename T>
typename Diagram<T>::GraphvizFragment Diagram<T>::DoGetGraphvizFragment(
    const GraphvizFragmentParams& params) const {
  // If we've reached max depth, treat us like a leaf system.
  if (params.max_depth <= 0) {
    return System<T>::DoGetGraphvizFragment(params);
  }

  // Recursively assemble the subsystems' fragments.
  std::map<const System<T>*, GraphvizFragment> subsystem_fragments;
  for (const auto& subsystem : registered_systems_) {
    subsystem_fragments.emplace(
        subsystem.get(),
        subsystem->GetGraphvizFragment(
            params.max_depth - 1,
            FilterGraphvizOptions(params.options, *subsystem)));
  }

  // Assemble our fragments into a subgraph.
  GraphvizFragment result;
  const int64_t id = this->get_system_id().get_value();

  // Open the diagram, with its label.
  result.fragments.push_back(fmt::format(
      R"""(subgraph cluster{}diagram {{
color=black
concentrate=true
label=<<TABLE BORDER="0"><TR><TD>
{}
</TD></TR></TABLE>>;
)""",
      id, fmt::join(params.header_lines, "<BR/>\n")));

  // Add clusters for input and output ports. This helper function is called up
  // to twice (once for input ports, if any; once for output ports, if any).
  // It appends the graphviz content to `fragments` and port IDs to `port_ids`.
  // The node that holds the ports will use ID `ports_node_id`.
  auto print_ports = [](std::vector<std::string>* fragments,
                        std::vector<std::string>* port_ids,
                        const std::string& kind /* "input" or "output" */,
                        const std::string& ports_node_id,
                        const std::string& color,
                        const std::vector<std::string>& labels) {
    fragments->push_back(fmt::format(
        R"""(subgraph cluster{} {{
rank=same
color=lightgrey
style=filled
label="{} ports"
{} [shape=none, label=<
<TABLE BORDER="0" COLOR="{}" CELLSPACING="3" STYLE="rounded">
)""",
        ports_node_id, kind, ports_node_id, color));
    for (int i = 0; i < ssize(labels); ++i) {
      fragments->push_back(
          fmt::format(R"""(<TR><TD BORDER="1" PORT="{}">{}</TD></TR>
)""",
                      i, labels[i]));
      port_ids->push_back(fmt::format("{}:{}", ports_node_id, i));
    }
    fragments->push_back(R"""(</TABLE>
>];
}
)""");
  };
  if (this->num_input_ports() > 0) {
    std::string inputs_node_id = fmt::format("{}in", params.node_id);
    print_ports(&result.fragments, &result.input_ports, "input", inputs_node_id,
                "blue", GetGraphvizPortLabels(/* input= */ true));
  }
  if (this->num_output_ports() > 0) {
    std::string outputs_node_id = fmt::format("{}out", params.node_id);
    print_ports(&result.fragments, &result.output_ports, "output",
                outputs_node_id, "green",
                GetGraphvizPortLabels(/* input= */ false));
  }

  // Open a cluster for the subsystems.
  result.fragments.push_back(fmt::format(
      R"""(subgraph cluster{}subsystems {{
color=white
label=""
)""",
      id));

  // -- Add the subsystems themselves.
  for (const auto& subsystem : registered_systems_) {
    GraphvizFragment& sub_frag = subsystem_fragments.at(subsystem.get());
    std::move(sub_frag.fragments.begin(), sub_frag.fragments.end(),
              std::back_inserter(result.fragments));
  }

  // -- Add the connections as edges.
  for (const auto& edge : connection_map_) {
    const System<T>* src_sys = edge.second.first;
    const OutputPortIndex src_index = edge.second.second;
    const System<T>* dest_sys = edge.first.first;
    const InputPortIndex dest_index = edge.first.second;
    result.fragments.push_back(fmt::format(
        "{}:e -> {}:w\n",
        subsystem_fragments.at(src_sys).output_ports.at(src_index),
        subsystem_fragments.at(dest_sys).input_ports.at(dest_index)));
  }

  // -- Add edges from the input and output port nodes to the subsystems that
  //    actually service that port. These edges are highlighted in blue
  //    (input) and green (output), matching the port nodes.
  for (int i = 0; i < this->num_input_ports(); ++i) {
    for (const auto& port_id : GetInputPortLocators(InputPortIndex(i))) {
      const System<T>* sub_sys = port_id.first;
      const InputPortIndex sub_index = port_id.second;
      const std::string& from = result.input_ports.at(i);
      const std::string& to =
          subsystem_fragments.at(sub_sys).input_ports.at(sub_index);
      result.fragments.push_back(
          fmt::format("{}:e -> {}:w [color=blue];\n", from, to));
    }
  }
  for (int i = 0; i < this->num_output_ports(); ++i) {
    const auto& port_id = output_port_ids_.at(i);
    const System<T>* sub_sys = port_id.first;
    const OutputPortIndex sub_index = port_id.second;
    const std::string& from =
        subsystem_fragments.at(sub_sys).output_ports.at(sub_index);
    const std::string& to = result.output_ports.at(i);
    result.fragments.push_back(
        fmt::format("{}:e -> {}:w [color=green];\n", from, to));
  }

  // Close the cluster for subsystems and then the diagram.
  result.fragments.push_back("}\n}\n");

  return result;
}

template <typename T>
std::vector<std::string> Diagram<T>::GetGraphvizPortLabels(bool input) const {
  return internal::DiagramSystemBaseAttorney::GetGraphvizPortLabels(*this,
                                                                    input);
}

template <typename T>
SubsystemIndex Diagram<T>::GetSystemIndexOrAbort(const System<T>* sys) const {
  auto it = system_index_map_.find(sys);
  DRAKE_DEMAND(it != system_index_map_.end());
  return it->second;
}

template <typename T>
bool Diagram<T>::AreConnected(const OutputPort<T>& output,
                              const InputPort<T>& input) const {
  InputPortLocator in{&input.get_system(), input.get_index()};
  OutputPortLocator out{&output.get_system(), output.get_index()};

  const auto range = connection_map_.equal_range(in);
  for (auto iter = range.first; iter != range.second; ++iter) {
    if (iter->second == out) return true;
  }
  return false;
}

template <typename T>
Diagram<T>::Diagram()
    : System<T>(SystemScalarConverter::MakeWithoutSubtypeChecking<Diagram>()) {}

template <typename T>
Diagram<T>::Diagram(SystemScalarConverter converter)
    : System<T>(std::move(converter)) {}

template <typename T>
T Diagram<T>::DoCalcWitnessValue(const Context<T>& context,
                                 const WitnessFunction<T>& witness_func) const {
  const System<T>& system = witness_func.get_system();
  const Context<T>& subcontext = GetSubsystemContext(system, context);
  return witness_func.CalcWitnessValue(subcontext);
}

template <typename T>
void Diagram<T>::AddTriggeredWitnessFunctionToCompositeEventCollection(
    Event<T>* event, CompositeEventCollection<T>* events) const {
  DRAKE_DEMAND(events != nullptr);
  DRAKE_DEMAND(event != nullptr);

  // Get the event data- it will need to be modified.
  WitnessTriggeredEventData<T>* data =
      event->template get_mutable_event_data<WitnessTriggeredEventData<T>>();
  DRAKE_DEMAND(data != nullptr);

  // Get the vector of events corresponding to the subsystem.
  const System<T>& subsystem = data->triggered_witness()->get_system();
  CompositeEventCollection<T>& subevents =
      GetMutableSubsystemCompositeEventCollection(subsystem, events);

  // Get the continuous states at both window endpoints.
  auto diagram_xc0 =
      dynamic_cast<const DiagramContinuousState<T>*>(data->xc0());
  DRAKE_DEMAND(diagram_xc0 != nullptr);
  auto diagram_xcf =
      dynamic_cast<const DiagramContinuousState<T>*>(data->xcf());
  DRAKE_DEMAND(diagram_xcf != nullptr);

  // Modify the pointer to the event data to point to the sub-system
  // continuous states.
  data->set_xc0(DoGetTargetSystemContinuousState(subsystem, diagram_xc0));
  data->set_xcf(DoGetTargetSystemContinuousState(subsystem, diagram_xcf));

  // Add the event to the collection.
  event->AddToComposite(&subevents);
}

template <typename T>
void Diagram<T>::DoGetWitnessFunctions(
    const Context<T>& context,
    std::vector<const WitnessFunction<T>*>* witnesses) const {
  // A temporary vector is necessary since the vector of witnesses is
  // declared to be empty on entry to DoGetWitnessFunctions().
  std::vector<const WitnessFunction<T>*> temp_witnesses;

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  SubsystemIndex index(0);

  for (const auto& system : registered_systems_) {
    DRAKE_ASSERT(index == GetSystemIndexOrAbort(system.get()));
    temp_witnesses.clear();
    system->GetWitnessFunctions(diagram_context->GetSubsystemContext(index),
                                &temp_witnesses);
    witnesses->insert(witnesses->end(), temp_witnesses.begin(),
                      temp_witnesses.end());
    ++index;
  }
}

template <typename T>
const Context<T>* Diagram<T>::DoGetTargetSystemContext(
    const System<T>& target_system, const Context<T>* context) const {
  if (&target_system == this) {
    return context;
  }

  return GetSubsystemStuff<const Context<T>, const DiagramContext<T>>(
      target_system, context, &System<T>::DoGetTargetSystemContext,
      &DiagramContext<T>::GetSubsystemContext);
}

template <typename T>
State<T>* Diagram<T>::DoGetMutableTargetSystemState(
    const System<T>& target_system, State<T>* state) const {
  if (&target_system == this) {
    return state;
  }

  return GetSubsystemStuff<State<T>, DiagramState<T>>(
      target_system, state, &System<T>::DoGetMutableTargetSystemState,
      &DiagramState<T>::get_mutable_substate);
}

template <typename T>
const ContinuousState<T>* Diagram<T>::DoGetTargetSystemContinuousState(
    const System<T>& target_system, const ContinuousState<T>* xc) const {
  if (&target_system == this) {
    return xc;
  }

  return GetSubsystemStuff<const ContinuousState<T>,
                           const DiagramContinuousState<T>>(
      target_system, xc, &System<T>::DoGetTargetSystemContinuousState,
      &DiagramContinuousState<T>::get_substate);
}

template <typename T>
const State<T>* Diagram<T>::DoGetTargetSystemState(
    const System<T>& target_system, const State<T>* state) const {
  if (&target_system == this) {
    return state;
  }

  return GetSubsystemStuff<const State<T>, const DiagramState<T>>(
      target_system, state, &System<T>::DoGetTargetSystemState,
      &DiagramState<T>::get_substate);
}

template <typename T>
CompositeEventCollection<T>*
Diagram<T>::DoGetMutableTargetSystemCompositeEventCollection(
    const System<T>& target_system, CompositeEventCollection<T>* events) const {
  if (&target_system == this) {
    return events;
  }

  return GetSubsystemStuff<CompositeEventCollection<T>,
                           DiagramCompositeEventCollection<T>>(
      target_system, events,
      &System<T>::DoGetMutableTargetSystemCompositeEventCollection,
      &DiagramCompositeEventCollection<T>::get_mutable_subevent_collection);
}

template <typename T>
const CompositeEventCollection<T>*
Diagram<T>::DoGetTargetSystemCompositeEventCollection(
    const System<T>& target_system,
    const CompositeEventCollection<T>* events) const {
  if (&target_system == this) return events;

  return GetSubsystemStuff<const CompositeEventCollection<T>,
                           const DiagramCompositeEventCollection<T>>(
      target_system, events,
      &System<T>::DoGetTargetSystemCompositeEventCollection,
      &DiagramCompositeEventCollection<T>::get_subevent_collection);
}

template <typename T>
void Diagram<T>::DoMapVelocityToQDot(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    VectorBase<T>* qdot) const {
  // Check that the dimensions of the continuous state in the context match
  // the dimensions of the provided generalized velocity and configuration
  // derivatives.
  const ContinuousState<T>& xc = context.get_continuous_state();
  const int nq = xc.get_generalized_position().size();
  const int nv = xc.get_generalized_velocity().size();
  DRAKE_DEMAND(nq == qdot->size());
  DRAKE_DEMAND(nv == generalized_velocity.size());

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Iterate over the subsystems, asking each subsystem to map its subslice of
  // velocity to configuration derivatives. This approach is valid because the
  // DiagramContinuousState guarantees that the subsystem states are
  // concatenated in order.
  int v_index = 0;  // The next index to read in generalized_velocity.
  int q_index = 0;  // The next index to write in qdot.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    // Find the continuous state of subsystem i.
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

    // Select the chunk of generalized_velocity belonging to subsystem i.
    const int num_v = sub_xc.get_generalized_velocity().size();
    if (num_v == 0) continue;
    const Eigen::Ref<const VectorX<T>>& v_slice =
        generalized_velocity.segment(v_index, num_v);

    // Select the chunk of qdot belonging to subsystem i.
    const int num_q = sub_xc.get_generalized_position().size();
    Subvector<T> dq_slice(qdot, q_index, num_q);

    // Delegate the actual mapping to subsystem i itself.
    registered_systems_[i]->MapVelocityToQDot(subcontext, v_slice, &dq_slice);

    // Advance the indices.
    v_index += num_v;
    q_index += num_q;
  }
}

template <typename T>
void Diagram<T>::DoMapQDotToVelocity(
    const Context<T>& context, const Eigen::Ref<const VectorX<T>>& qdot,
    VectorBase<T>* generalized_velocity) const {
  // Check that the dimensions of the continuous state in the context match
  // the dimensions of the provided generalized velocity and configuration
  // derivatives.
  const ContinuousState<T>& xc = context.get_continuous_state();
  const int nq = xc.get_generalized_position().size();
  const int nv = xc.get_generalized_velocity().size();
  DRAKE_DEMAND(nq == qdot.size());
  DRAKE_DEMAND(nv == generalized_velocity->size());

  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Iterate over the subsystems, asking each subsystem to map its subslice of
  // configuration derivatives to velocity. This approach is valid because the
  // DiagramContinuousState guarantees that the subsystem states are
  // concatenated in order.
  int q_index = 0;  // The next index to read in qdot.
  int v_index = 0;  // The next index to write in generalized_velocity.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    // Find the continuous state of subsystem i.
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

    // Select the chunk of qdot belonging to subsystem i.
    const int num_q = sub_xc.get_generalized_position().size();
    if (num_q == 0) continue;
    const Eigen::Ref<const VectorX<T>>& dq_slice = qdot.segment(q_index, num_q);

    // Select the chunk of generalized_velocity belonging to subsystem i.
    const int num_v = sub_xc.get_generalized_velocity().size();
    Subvector<T> v_slice(generalized_velocity, v_index, num_v);

    // Delegate the actual mapping to subsystem i itself.
    registered_systems_[i]->MapQDotToVelocity(subcontext, dq_slice, &v_slice);

    // Advance the indices.
    v_index += num_v;
    q_index += num_q;
  }
}

template <typename T>
void Diagram<T>::DoCalcNextUpdateTime(const Context<T>& context,
                                      CompositeEventCollection<T>* event_info,
                                      T* next_update_time) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  CacheEntryValue& value =
      this->get_cache_entry(event_times_buffer_cache_index_)
          .get_mutable_cache_entry_value(context);
  auto& event_times_buffer = value.GetMutableValueOrThrow<std::vector<T>>();
  DRAKE_DEMAND(static_cast<int>(event_times_buffer.size()) == num_subsystems());

  // In assert-enabled builds, enforce the invariant that no stale values in
  // event_times_buffer are reused across invocations. In effect,
  // event_times_buffer should have the semantics of a stack variable, despite
  // being cache-managed heap storage. The enforcement appears in two parts: an
  // assert-build-only clearing of the storage to invalid values, and a later
  // assertion that the invalid values are all replaced.
  auto set_to_nan = [](std::vector<T>* vec) {
    std::fill(vec->begin(), vec->end(),
              std::numeric_limits<
                  typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  };
  DRAKE_ASSERT_VOID(set_to_nan(&event_times_buffer));

  *next_update_time = std::numeric_limits<double>::infinity();

  // Iterate over the subsystems, and harvest the most imminent updates.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    const T sub_time =
        registered_systems_[i]->CalcNextUpdateTime(subcontext, &subinfo);
    event_times_buffer[i] = sub_time;

    if (sub_time < *next_update_time) {
      *next_update_time = sub_time;
    }
  }

  // Check that all vector entries were replaced.
  auto none_are_nan = [](const std::vector<T>& vec) {
    using std::isnan;
    return std::none_of(vec.begin(), vec.end(), [](const T& v) {
      return isnan(v);
    });
  };
  DRAKE_ASSERT(none_are_nan(event_times_buffer));

  // For all the subsystems whose next update time is bigger than
  // next_update_time, clear their event collections.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    if (event_times_buffer[i] > *next_update_time)
      info->get_mutable_subevent_collection(i).Clear();
  }
}

template <typename T>
std::string Diagram<T>::GetUnsupportedScalarConversionMessage(
    const std::type_info& source_type,
    const std::type_info& destination_type) const {
  // Start with the default message for this system.
  std::stringstream result;
  result << SystemBase::GetUnsupportedScalarConversionMessage(source_type,
                                                              destination_type);

  // Append extra details, if we are able to.
  std::vector<std::string> causes;
  for (const auto& system : registered_systems_) {
    const auto& converter = system->get_system_scalar_converter();
    if (converter.IsConvertible(destination_type, source_type)) {
      continue;
    }
    causes.push_back(internal::DiagramSystemBaseAttorney::
                         GetUnsupportedScalarConversionMessage(
                             *system, source_type, destination_type));
  }
  if (!causes.empty()) {
    result << fmt::format(" (because {})", fmt::join(causes, " and "));
  }

  return result.str();
}

template <typename T>
std::unique_ptr<AbstractValue> Diagram<T>::DoAllocateInput(
    const InputPort<T>& input_port) const {
  // Ask an arbitrary connected subsystem to perform the allocation.
  const auto id = GetArbitraryInputPortLocator(input_port.get_index());
  const System<T>* subsystem = id.first;
  const InputPortIndex subindex = id.second;
  return subsystem->AllocateInputAbstract(subsystem->get_input_port(subindex));
}

template <typename T>
std::unique_ptr<ContextBase> Diagram<T>::DoAllocateContext() const {
  // Reserve inputs as specified during Diagram initialization.
  auto context = std::make_unique<DiagramContext<T>>(num_subsystems());
  this->InitializeContextBase(&*context);

  // Recursively construct each constituent system and its subsystems,
  // then add to this diagram Context.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const System<T>& system = *registered_systems_[i];
    auto subcontext =
        dynamic_pointer_cast_or_throw<Context<T>>(system.AllocateContext());
    context->AddSystem(i, std::move(subcontext));
  }

  // Creates this diagram's composite data structures that collect its
  // subsystems' resources, which must have already been allocated above. No
  // dependencies are set up in these two calls. Note that MakeState()
  // establishes the system_id labels for all the helper objects at the root of
  // the state tree.
  context->MakeParameters();
  context->MakeState();

  // Subscribe each of the Diagram's composite-entity dependency trackers to
  // the trackers for the corresponding constituent entities in the child
  // subsystems. This ensures that changes made at the subcontext level are
  // propagated correctly to the diagram context level. That includes state
  // and parameter changes, as well as local changes that affect composite
  // diagram-level computations like xcdot and pe.
  context->SubscribeDiagramCompositeTrackersToChildrens();

  // Peer-to-peer connections wire a child subsystem's input port to a
  // child subsystem's output port. Subscribe each child input port to the
  // child output port on which it depends.
  for (const auto& connection : connection_map_) {
    const OutputPortLocator& src = connection.second;
    const InputPortLocator& dest = connection.first;
    context->SubscribeInputPortToOutputPort(
        ConvertToContextPortIdentifier(src),
        ConvertToContextPortIdentifier(dest));
  }

  // Diagram-external input ports are exported from child subsystems (meaning
  // the Diagram input is fed into the input of one of its children.
  // Subscribe the child subsystem's input port to its parent Diagram's input
  // port on which it depends.
  for (InputPortIndex i(0); i < this->num_input_ports(); ++i) {
    for (const auto& id : GetInputPortLocators(i)) {
      context->SubscribeExportedInputPortToDiagramPort(
          i, ConvertToContextPortIdentifier(id));
    }
  }

  // Diagram-external output ports are exported from child subsystem output
  // ports. Subscribe each Diagram-level output to the child-level output on
  // which it depends.
  for (OutputPortIndex i(0); i < this->num_output_ports(); ++i) {
    const OutputPortLocator& id = output_port_ids_[i];
    context->SubscribeDiagramPortToExportedOutputPort(
        i, ConvertToContextPortIdentifier(id));
  }

  return context;
}

template <typename T>
const AbstractValue* Diagram<T>::EvalConnectedSubsystemInputPort(
    const ContextBase& context_base,
    const InputPortBase& input_port_base) const {
  // Profiling revealed that it is too expensive to do a dynamic_cast here.
  // A static_cast is safe as long we validate the SystemId, so that we know
  // this Context is ours. Proving that our caller would have already checked
  // the SystemId is too complicated, so we'll always just check ourselves.
  this->ValidateContext(context_base);
  auto& diagram_context = static_cast<const DiagramContext<T>&>(context_base);

  // Profiling revealed that it is too expensive to do a dynamic_cast here.
  // A static_cast is safe as long as the given input_port_base was actually
  // an InputPort<T>, and since our sole caller is SystemBase which always
  // retrieves the input_port_base from `this`, the <T> must be correct.
  auto& system = static_cast<const System<T>&>(
      internal::PortBaseAttorney::get_system_interface(input_port_base));
  const InputPortLocator id{&system, input_port_base.get_index()};

  // Find if this input port is exported (connected to an input port of this
  // containing diagram).
  const auto external_it = input_port_map_.find(id);
  const bool is_exported = (external_it != input_port_map_.end());

  // Find if this input port is connected to an output port.
  // TODO(sherm1) Fix this. Shouldn't have to search.
  const auto upstream_it = connection_map_.find(id);
  const bool is_connected = (upstream_it != connection_map_.end());

  if (!(is_exported || is_connected)) {
    return nullptr;
  }

  DRAKE_DEMAND(is_exported ^ is_connected);

  if (is_exported) {
    // The upstream source is an input to this whole Diagram; evaluate that
    // input port and use the result as the value for this one.
    const InputPortIndex i = external_it->second;
    return this->EvalAbstractInput(diagram_context, i);
  }

  // The upstream source is an output port of one of this Diagram's child
  // subsystems; evaluate it.
  // TODO(david-german-tri): Add online algebraic loop detection here.
  DRAKE_ASSERT(is_connected);
  const OutputPortLocator& prerequisite = upstream_it->second;
  return &this->EvalSubsystemOutputPort(diagram_context, prerequisite);
}

template <typename T>
std::string Diagram<T>::GetParentPathname() const {
  return this->GetSystemPathname();
}

template <typename T>
const SystemBase& Diagram<T>::GetRootSystemBase() const {
  const auto* parent_service = this->get_parent_service();
  return parent_service ? parent_service->GetRootSystemBase() : *this;
}

template <typename T>
bool Diagram<T>::DiagramHasDirectFeedthrough(
    int input_port, int output_port,
    std::unordered_map<const System<T>*, std::multimap<int, int>>* memoize)
    const {
  DRAKE_ASSERT(input_port >= 0);
  DRAKE_ASSERT(input_port < this->num_input_ports());
  DRAKE_ASSERT(output_port >= 0);
  DRAKE_ASSERT(output_port < this->num_output_ports());
  DRAKE_ASSERT(memoize != nullptr);

  const auto input_ids = GetInputPortLocators(InputPortIndex(input_port));
  std::set<InputPortLocator> target_input_ids(input_ids.begin(),
                                              input_ids.end());

  // Search the graph for a direct-feedthrough connection from the output_port
  // back to the input_port. Maintain a set of the output port identifiers
  // that are known to have a direct-feedthrough path to the output_port.
  std::set<OutputPortLocator> active_set;
  active_set.insert(output_port_ids_[output_port]);
  while (!active_set.empty()) {
    const OutputPortLocator current_output_id = *active_set.begin();
    const size_t removed_count = active_set.erase(current_output_id);
    DRAKE_ASSERT(removed_count == 1);
    const System<T>* const sys = current_output_id.first;
    auto memoized_feedthroughs = memoize->find(sys);
    if (memoized_feedthroughs == memoize->end()) {
      bool inserted{};
      std::tie(memoized_feedthroughs, inserted) =
          memoize->emplace(sys, sys->GetDirectFeedthroughs());
      DRAKE_ASSERT(inserted);
    }
    const std::multimap<int, int>& sys_feedthroughs =
        memoized_feedthroughs->second;
    for (const auto& [sys_input, sys_output] : sys_feedthroughs) {
      if (sys_output == current_output_id.second) {
        const InputPortLocator curr_input_id(sys, sys_input);
        if (target_input_ids.contains(curr_input_id)) {
          // We've found a direct-feedthrough path to the input_port.
          return true;
        } else {
          // We've found an intermediate input port has a direct-feedthrough
          // path to the output_port. Add the upstream output port (if there
          // is one) to the active set.
          auto it = connection_map_.find(curr_input_id);
          if (it != connection_map_.end()) {
            const OutputPortLocator& upstream_output = it->second;
            active_set.insert(upstream_output);
          }
        }
      }
    }
  }
  // If there are no intermediate output ports with a direct-feedthrough path
  // to the output_port, there is no direct feedthrough to it from the
  // input_port.
  return false;
}

template <typename T>
template <typename EventType>
std::unique_ptr<EventCollection<EventType>>
Diagram<T>::AllocateForcedEventCollection(
    std::function<std::unique_ptr<EventCollection<EventType>>(const System<T>*)>
        allocator_func) const {
  const int num_systems = num_subsystems();
  auto ret = std::make_unique<DiagramEventCollection<EventType>>(num_systems);
  for (SubsystemIndex i(0); i < num_systems; ++i) {
    std::unique_ptr<EventCollection<EventType>> subevent_collection =
        allocator_func(registered_systems_[i].get());

    // The DiagramEventCollection should own these subevents- this function
    // will not maintain its own references to them.
    ret->set_and_own_subevent_collection(i, std::move(subevent_collection));
  }
  return ret;
}

template <typename T>
EventStatus Diagram<T>::DispatchPublishHandler(
    const Context<T>& context,
    const EventCollection<PublishEvent<T>>& event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);
  const DiagramEventCollection<PublishEvent<T>>& info =
      dynamic_cast<const DiagramEventCollection<PublishEvent<T>>&>(event_info);

  EventStatus overall_status = EventStatus::DidNothing();
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<PublishEvent<T>>& subinfo =
        info.get_subevent_collection(i);

    if (subinfo.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      const EventStatus per_subsystem_status =
          registered_systems_[i]->Publish(subcontext, subinfo);
      overall_status.KeepMoreSevere(per_subsystem_status);
      // Unlike the discrete & unrestricted event policy, we don't stop handling
      // publish events when one fails; we just report the first failure after
      // all the publishes are done.
    }
  }
  return overall_status;
}

template <typename T>
EventStatus Diagram<T>::DispatchDiscreteVariableUpdateHandler(
    const Context<T>& context,
    const EventCollection<DiscreteUpdateEvent<T>>& events,
    DiscreteValues<T>* discrete_state) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);
  auto diagram_discrete =
      dynamic_cast<DiagramDiscreteValues<T>*>(discrete_state);
  DRAKE_DEMAND(diagram_discrete != nullptr);

  const DiagramEventCollection<DiscreteUpdateEvent<T>>& diagram_events =
      dynamic_cast<const DiagramEventCollection<DiscreteUpdateEvent<T>>&>(
          events);

  EventStatus overall_status = EventStatus::DidNothing();
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<DiscreteUpdateEvent<T>>& subevents =
        diagram_events.get_subevent_collection(i);

    if (subevents.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      DiscreteValues<T>& subdiscrete =
          diagram_discrete->get_mutable_subdiscrete(i);

      const EventStatus per_subsystem_status =
          registered_systems_[i]->CalcDiscreteVariableUpdate(
              subcontext, subevents, &subdiscrete);
      overall_status.KeepMoreSevere(per_subsystem_status);
      if (overall_status.failed()) break;  // Stop at the first disaster.
    }
  }
  return overall_status;
}

template <typename T>
void Diagram<T>::DoApplyDiscreteVariableUpdate(
    const EventCollection<DiscreteUpdateEvent<T>>& events,
    DiscreteValues<T>* discrete_state, Context<T>* context) const {
  // If this method is called, these are all Diagram objects.
  const auto& diagram_events =
      dynamic_cast<const DiagramEventCollection<DiscreteUpdateEvent<T>>&>(
          events);
  auto& diagram_discrete =
      dynamic_cast<DiagramDiscreteValues<T>&>(*discrete_state);
  auto& diagram_context = dynamic_cast<DiagramContext<T>&>(*context);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<DiscreteUpdateEvent<T>>& subevents =
        diagram_events.get_subevent_collection(i);

    if (subevents.HasEvents()) {
      DiscreteValues<T>& subdiscrete =
          diagram_discrete.get_mutable_subdiscrete(i);
      Context<T>& subcontext = diagram_context.GetMutableSubsystemContext(i);

      registered_systems_[i]->ApplyDiscreteVariableUpdate(
          subevents, &subdiscrete, &subcontext);
    }
  }
}

template <typename T>
EventStatus Diagram<T>::DispatchUnrestrictedUpdateHandler(
    const Context<T>& context,
    const EventCollection<UnrestrictedUpdateEvent<T>>& events,
    State<T>* state) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  DRAKE_DEMAND(diagram_context != nullptr);
  auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
  DRAKE_DEMAND(diagram_state != nullptr);

  const DiagramEventCollection<UnrestrictedUpdateEvent<T>>& diagram_events =
      dynamic_cast<const DiagramEventCollection<UnrestrictedUpdateEvent<T>>&>(
          events);

  EventStatus overall_status = EventStatus::DidNothing();
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<UnrestrictedUpdateEvent<T>>& subevents =
        diagram_events.get_subevent_collection(i);

    if (subevents.HasEvents()) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      State<T>& substate = diagram_state->get_mutable_substate(i);

      const EventStatus per_subsystem_status =
          registered_systems_[i]->CalcUnrestrictedUpdate(subcontext, subevents,
                                                         &substate);
      overall_status.KeepMoreSevere(per_subsystem_status);
      if (overall_status.failed()) break;  // Stop at the first disaster.
    }
  }
  return overall_status;
}

template <typename T>
void Diagram<T>::DoApplyUnrestrictedUpdate(
    const EventCollection<UnrestrictedUpdateEvent<T>>& events, State<T>* state,
    Context<T>* context) const {
  // If this method is called, these are all Diagram objects.
  const auto& diagram_events =
      dynamic_cast<const DiagramEventCollection<UnrestrictedUpdateEvent<T>>&>(
          events);
  auto& diagram_state = dynamic_cast<DiagramState<T>&>(*state);
  auto& diagram_context = dynamic_cast<DiagramContext<T>&>(*context);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const EventCollection<UnrestrictedUpdateEvent<T>>& subevents =
        diagram_events.get_subevent_collection(i);

    if (subevents.HasEvents()) {
      State<T>& substate = diagram_state.get_mutable_substate(i);
      Context<T>& subcontext = diagram_context.GetMutableSubsystemContext(i);

      registered_systems_[i]->ApplyUnrestrictedUpdate(subevents, &substate,
                                                      &subcontext);
    }
  }
}

template <typename T>
template <typename BaseStuff, typename DerivedStuff>
BaseStuff* Diagram<T>::GetSubsystemStuff(
    const System<T>& target_system, BaseStuff* my_stuff,
    std::function<BaseStuff*(const System<T>*, const System<T>&, BaseStuff*)>
        recursive_getter,
    std::function<BaseStuff&(DerivedStuff*, SubsystemIndex)> get_child_stuff)
    const {
  static_assert(
      std::is_same_v<BaseStuff, typename std::remove_pointer_t<BaseStuff>>,
      "BaseStuff cannot be a pointer");
  static_assert(std::is_same_v<DerivedStuff,
                               typename std::remove_pointer_t<DerivedStuff>>,
                "DerivedStuff cannot be a pointer");

  DRAKE_DEMAND(my_stuff != nullptr);
  DRAKE_DEMAND(&target_system != this);
  DerivedStuff& my_stuff_as_derived = dynamic_cast<DerivedStuff&>(*my_stuff);

  SubsystemIndex index(0);
  for (const auto& child : registered_systems_) {
    BaseStuff& child_stuff = get_child_stuff(&my_stuff_as_derived, index);

    BaseStuff* const target_stuff =
        recursive_getter(child.get(), target_system, &child_stuff);

    if (target_stuff != nullptr) {
      return target_stuff;
    }
    ++index;
  }

  return nullptr;
}

template <typename T>
template <typename NewType>
std::unique_ptr<typename Diagram<NewType>::Blueprint>
Diagram<T>::ConvertScalarType() const {
  internal::OwnedSystems<NewType> new_systems;
  // Recursively convert all the subsystems.
  std::map<const System<T>*, const System<NewType>*> old_to_new_map;
  for (const auto& old_system : registered_systems_) {
    // Convert old_system to new_system using the old_system's converter.
    std::shared_ptr<System<NewType>> new_system =
        old_system->get_system_scalar_converter().template Convert<NewType>(
            *old_system);
    DRAKE_DEMAND(new_system != nullptr);

    // Special case to work around Python difficulties -- if the new_system is a
    // leaf system implemented in Python, then undo the wrapping that helped it
    // survive the SystemScalarConverter. Refer to the pydrake/bindings/systems
    // helper function DefineSystemScalarConverter() to understand where the
    // WrappedSystem was originally injected into this control flow.
    if (auto* wrapped =
            dynamic_cast<internal::WrappedSystem<NewType>*>(new_system.get());
        wrapped != nullptr) {
      // Extract the inner system.
      const std::string name = new_system->get_name();
      DRAKE_DEMAND(wrapped->registered_systems_.size() == 1);
      std::shared_ptr<System<NewType>> inner = wrapped->registered_systems_[0];
      // Discard the wrapper.
      SystemBase::set_parent_service(inner.get(), nullptr);
      new_system.reset();
      // Use the inner system when creating this Diagram.
      new_system = inner;
      new_system->set_name(name);
    }

    // Because we called the scalar converter directly without going through
    // System::ToScalarTypeMaybe, we need to re-implement its logic to copy any
    // external constraints. Note that we must call this function on Diagram for
    // reasons of access control, but it's really a System<NewType> function.
    Diagram<NewType>::HandlePostConstructionScalarConversion(*old_system,
                                                             new_system.get());

    // Update our mapping and take ownership.
    old_to_new_map[old_system.get()] = new_system.get();
    new_systems.push_back(std::move(new_system));
  }

  // Set up the blueprint.
  auto blueprint = std::make_unique<typename Diagram<NewType>::Blueprint>();
  // Make all the inputs, preserving index assignments.
  for (int k = 0; k < this->num_input_ports(); k++) {
    const auto name = this->get_input_port(k).get_name();
    for (const auto& id : GetInputPortLocators(InputPortIndex(k))) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const InputPortIndex port = id.second;
      blueprint->input_port_ids.emplace_back(new_system, port);
      blueprint->input_port_names.emplace_back(name);
    }
  }
  // Make all the outputs.
  for (const OutputPortLocator& id : output_port_ids_) {
    const System<NewType>* new_system = old_to_new_map[id.first];
    const OutputPortIndex port = id.second;
    blueprint->output_port_ids.emplace_back(new_system, port);
  }
  for (OutputPortIndex i{0}; i < this->num_output_ports(); i++) {
    blueprint->output_port_names.emplace_back(
        this->get_output_port(i).get_name());
  }
  // Make all the connections.
  for (const auto& edge : connection_map_) {
    const InputPortLocator& old_dest = edge.first;
    const System<NewType>* const dest_system = old_to_new_map[old_dest.first];
    const InputPortIndex dest_port = old_dest.second;
    const typename Diagram<NewType>::InputPortLocator new_dest{dest_system,
                                                               dest_port};

    const OutputPortLocator& old_src = edge.second;
    const System<NewType>* const src_system = old_to_new_map[old_src.first];
    const OutputPortIndex src_port = old_src.second;
    const typename Diagram<NewType>::OutputPortLocator new_src{src_system,
                                                               src_port};

    blueprint->connection_map[new_dest] = new_src;
  }
  // Move the new systems into the blueprint.
  blueprint->systems = std::move(new_systems);

  // Do nothing about life_support. Since scalar conversion is effectively a
  // deep copy, the lifetime extensions provided by life_support are not needed
  // here.

  return blueprint;
}

template <typename T>
std::map<PeriodicEventData, std::vector<const Event<T>*>,
         PeriodicEventDataComparator>
Diagram<T>::DoMapPeriodicEventsByTiming(const Context<T>& context) const {
  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator>
      periodic_events_map;

  for (int i = 0; i < num_subsystems(); ++i) {
    const System<T>& sub_system = *registered_systems_[i];
    const Context<T>& sub_context = GetSubsystemContext(sub_system, context);
    auto sub_map = sub_system.MapPeriodicEventsByTiming(&sub_context);
    for (const auto& sub_attr_events : sub_map) {
      const auto& sub_vec = sub_attr_events.second;
      auto& vec = periodic_events_map[sub_attr_events.first];
      vec.insert(vec.end(), sub_vec.begin(), sub_vec.end());
    }
  }

  return periodic_events_map;
}

// Strategy:
// For each subsystem, obtain its set of uniquely-timed periodic discrete
// updates (if any) in an EventCollection. The first of these sets the timing;
// all others must have the same timing. Once the full collection has been
// assembled, we can use CalcDiscreteVariableUpdate() to get the result.
// See the LeafSystem implementation of this virtual for clarification.
template <typename T>
void Diagram<T>::DoFindUniquePeriodicDiscreteUpdatesOrThrow(
    const char* api_name, const Context<T>& context,
    std::optional<PeriodicEventData>* timing,
    EventCollection<DiscreteUpdateEvent<T>>* events) const {
  auto& diagram_collection =
      dynamic_cast<DiagramEventCollection<DiscreteUpdateEvent<T>>&>(*events);

  for (int i = 0; i < num_subsystems(); ++i) {
    const System<T>& sub_system = *registered_systems_[i];
    const Context<T>& sub_context = GetSubsystemContext(sub_system, context);
    EventCollection<DiscreteUpdateEvent<T>>& sub_collection =
        diagram_collection.get_mutable_subevent_collection(i);
    System<T>::FindUniquePeriodicDiscreteUpdatesOrThrow(
        api_name, sub_system, sub_context, &*timing, &sub_collection);
  }
}

template <typename T>
void Diagram<T>::DoGetPeriodicEvents(
    const Context<T>& context, CompositeEventCollection<T>* event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    registered_systems_[i]->GetPeriodicEvents(subcontext, &subinfo);
  }
}

template <typename T>
void Diagram<T>::DoGetPerStepEvents(
    const Context<T>& context, CompositeEventCollection<T>* event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    registered_systems_[i]->GetPerStepEvents(subcontext, &subinfo);
  }
}

template <typename T>
void Diagram<T>::DoGetInitializationEvents(
    const Context<T>& context, CompositeEventCollection<T>* event_info) const {
  auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
  auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
  DRAKE_DEMAND(diagram_context != nullptr);
  DRAKE_DEMAND(info != nullptr);

  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
    CompositeEventCollection<T>& subinfo =
        info->get_mutable_subevent_collection(i);

    registered_systems_[i]->GetInitializationEvents(subcontext, &subinfo);
  }
}

template <typename T>
Diagram<T>::Diagram(std::unique_ptr<Blueprint> blueprint) : Diagram() {
  Initialize(std::move(blueprint));
}

template <typename T>
void Diagram<T>::Initialize(std::unique_ptr<Blueprint> blueprint) {
  // We must be given something to own.
  DRAKE_DEMAND(!blueprint->systems.empty());
  // We must not already own any subsystems.
  DRAKE_DEMAND(registered_systems_.empty());

  // Move the corresponding data from the blueprint into private member
  // variables.
  connection_map_ = std::move(blueprint->connection_map);
  output_port_ids_ = std::move(blueprint->output_port_ids);
  registered_systems_ = std::move(blueprint->systems);
  life_support_ = std::move(blueprint->life_support);

  // This cache entry just maintains temporary storage. It is only ever used
  // by DoCalcNextUpdateTime(). Since this declaration of the cache entry
  // invokes no invalidation support from the cache system, it is the
  // responsibility of DoCalcNextUpdateTime() to ensure that no stale data is
  // used.
  event_times_buffer_cache_index_ =
      this->DeclareCacheEntry("event_times_buffer",
                              ValueProducer(std::vector<T>(num_subsystems()),
                                            &ValueProducer::NoopCalc),
                              {this->nothing_ticket()})
          .cache_index();

  // Generate a map from the System pointer to its index in the registered
  // order.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    system_index_map_[registered_systems_[i].get()] = i;
    SystemBase::set_parent_service(registered_systems_[i].get(), this);
  }

  // Generate constraints for the diagram from the constraints on the
  // subsystems.
  for (SubsystemIndex i(0); i < num_subsystems(); ++i) {
    const auto sys = registered_systems_[i].get();
    for (SystemConstraintIndex j(0); j < sys->num_constraints(); ++j) {
      const auto c = &(sys->get_constraint(j));
      ContextConstraintCalc<T> diagram_calc =
          [this, sys, c](const Context<T>& context, VectorX<T>* value) {
            c->Calc(this->GetSubsystemContext(*sys, context), value);
          };
      this->AddConstraint(std::make_unique<SystemConstraint<T>>(
          this, diagram_calc, c->bounds(),
          sys->get_name() + ":" + c->description()));
    }
  }

  // Every system must appear exactly once.
  DRAKE_DEMAND(registered_systems_.size() == system_index_map_.size());
  // Every port named in the connection_map_ must actually exist.
  DRAKE_ASSERT(PortsAreValid());
  // Every subsystem must have a unique name.
  DRAKE_THROW_UNLESS(NamesAreUniqueAndNonEmpty());

  // Add the inputs to the Diagram topology, and check their invariants.
  DRAKE_DEMAND(blueprint->input_port_ids.size() ==
               blueprint->input_port_names.size());
  std::vector<std::string>::iterator name_iter =
      blueprint->input_port_names.begin();
  for (const InputPortLocator& id : blueprint->input_port_ids) {
    ExportOrConnectInput(id, *name_iter++);
  }
  DRAKE_DEMAND(output_port_ids_.size() == blueprint->output_port_names.size());
  name_iter = blueprint->output_port_names.begin();
  for (const OutputPortLocator& id : output_port_ids_) {
    ExportOutput(id, *name_iter++);
  }

  // Identify the intersection of the subsystems' scalar conversion support.
  // Remove all conversions that at least one subsystem did not support.
  SystemScalarConverter& this_scalar_converter =
      this->get_mutable_system_scalar_converter();
  for (const auto& system : registered_systems_) {
    this_scalar_converter.RemoveUnlessAlsoSupportedBy(
        system->get_system_scalar_converter());
  }

  this->set_forced_publish_events(
      AllocateForcedEventCollection<PublishEvent<T>>(
          &System<T>::AllocateForcedPublishEventCollection));
  this->set_forced_discrete_update_events(
      AllocateForcedEventCollection<DiscreteUpdateEvent<T>>(
          &System<T>::AllocateForcedDiscreteUpdateEventCollection));
  this->set_forced_unrestricted_update_events(
      AllocateForcedEventCollection<UnrestrictedUpdateEvent<T>>(
          &System<T>::AllocateForcedUnrestrictedUpdateEventCollection));

  // Total up all needed Context resources, and the size of the implicit time
  // derivative residual. Note that we are depending on sub-Diagrams already to
  // have been initialized so that their counts are already correct.
  SystemBase::ContextSizes& sizes = this->get_mutable_context_sizes();
  int residual_size = 0;
  for (const auto& system : registered_systems_) {
    sizes += SystemBase::get_context_sizes(*system);
    residual_size += system->implicit_time_derivatives_residual_size();
  }
  this->set_implicit_time_derivatives_residual_size(residual_size);
}

template <typename T>
void Diagram<T>::ExportOrConnectInput(const InputPortLocator& port,
                                      std::string name) {
  const System<T>* const sys = port.first;
  const int port_index = port.second;
  // Fail quickly if this system is not part of the diagram.
  GetSystemIndexOrAbort(sys);

  if (this->HasInputPort(name)) {
    input_port_map_[port] = this->GetInputPort(name).get_index();
  } else {
    // Add this port to our externally visible topology.
    const auto& subsystem_input_port = sys->get_input_port(port_index);
    const InputPort<T>& new_port = this->DeclareInputPort(
        std::move(name), subsystem_input_port.get_data_type(),
        subsystem_input_port.size(), subsystem_input_port.get_random_type());
    input_port_map_[port] = new_port.get_index();
  }
}

template <typename T>
void Diagram<T>::ExportOutput(const OutputPortLocator& port, std::string name) {
  const System<T>* const sys = port.first;
  const int port_index = port.second;
  const auto& source_output_port = sys->get_output_port(port_index);
  // TODO(sherm1) Use implicit_cast when available (from abseil).
  auto diagram_port = internal::FrameworkFactory::Make<DiagramOutputPort<T>>(
      this,  // implicit_cast<const System<T>*>(this)
      this,  // implicit_cast<SystemBase*>(this)
      this->get_system_id(), std::move(name),
      OutputPortIndex(this->num_output_ports()),
      this->assign_next_dependency_ticket(), &source_output_port,
      GetSystemIndexOrAbort(&source_output_port.get_system()));
  this->AddOutputPort(std::move(diagram_port));
}

template <typename T>
const AbstractValue& Diagram<T>::EvalSubsystemOutputPort(
    const DiagramContext<T>& context, const OutputPortLocator& id) const {
  const System<T>* const system = id.first;
  const OutputPortIndex port_index(id.second);
  const OutputPort<T>& port = system->get_output_port(port_index);
  const SubsystemIndex i = GetSystemIndexOrAbort(system);
  DRAKE_LOGGER_TRACE("Evaluating output for subsystem {}, port {}",
                     system->GetSystemPathname(), port_index);
  const Context<T>& subsystem_context = context.GetSubsystemContext(i);
  return port.template Eval<AbstractValue>(subsystem_context);
}

template <typename T>
typename DiagramContext<T>::InputPortIdentifier
Diagram<T>::ConvertToContextPortIdentifier(
    const InputPortLocator& locator) const {
  typename DiagramContext<T>::InputPortIdentifier identifier;
  identifier.first = GetSystemIndexOrAbort(locator.first);
  identifier.second = locator.second;
  return identifier;
}

template <typename T>
typename DiagramContext<T>::OutputPortIdentifier
Diagram<T>::ConvertToContextPortIdentifier(
    const OutputPortLocator& locator) const {
  typename DiagramContext<T>::OutputPortIdentifier identifier;
  identifier.first = GetSystemIndexOrAbort(locator.first);
  identifier.second = locator.second;
  return identifier;
}

template <typename T>
bool Diagram<T>::PortsAreValid() const {
  for (const auto& entry : connection_map_) {
    const InputPortLocator& dest = entry.first;
    const OutputPortLocator& src = entry.second;
    if (dest.second < 0 || dest.second >= dest.first->num_input_ports()) {
      return false;
    }
    if (src.second < 0 || src.second >= src.first->num_output_ports()) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool Diagram<T>::NamesAreUniqueAndNonEmpty() const {
  string_unordered_set names;
  for (const auto& system : registered_systems_) {
    const std::string& name = system->get_name();
    if (name.empty()) {
      // This can only happen if someone blanks out the name *after* adding
      // it to DiagramBuilder; if an empty name is given to DiagramBuilder,
      // a default non-empty name is automatically assigned.
      log()->error("Subsystem of type {} has no name",
                   NiceTypeName::Get(*system));
      // We skip names.insert here, so that the return value will be false.
      continue;
    }
    if (names.find(name) != names.end()) {
      log()->error("Non-unique name \"{}\" for subsystem of type {}", name,
                   NiceTypeName::Get(*system));
    }
    names.insert(name);
  }
  return names.size() == registered_systems_.size();
}

template <typename T>
int Diagram<T>::num_subsystems() const {
  return static_cast<int>(registered_systems_.size());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Diagram);
