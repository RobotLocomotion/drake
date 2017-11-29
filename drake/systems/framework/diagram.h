#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace systems {

template <typename T>
class Diagram;
template <typename T>
class DiagramBuilder;

namespace internal {

/// Returns a vector of raw pointers that correspond placewise with the
/// unique_ptrs in the vector @p in.
template <typename U>
std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
  std::vector<U*> out(in.size());
  std::transform(in.begin(), in.end(), out.begin(),
                 [](const std::unique_ptr<U>& p) { return p.get(); });
  return out;
}

//==============================================================================
//                          DIAGRAM OUTPUT PORT
//==============================================================================
/// Holds information about the subsystem output port that has been exported to
/// become one of this Diagram's output ports. The actual methods for
/// determining the port's value are supplied by the LeafSystem that ultimately
/// underlies the source port, although that may be any number of levels down.
template <typename T>
class DiagramOutputPort : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramOutputPort)

  /// Construct a %DiagramOutputPort for the given `diagram` that exports the
  /// indicated port. That port's owning system must be a subsystem of the
  /// diagram.
  DiagramOutputPort(const Diagram<T>& diagram,
                    const OutputPort<T>* source_output_port)
      : OutputPort<T>(diagram, source_output_port->get_data_type(),
                      source_output_port->size()),
        source_output_port_(source_output_port),
        subsystem_index_(
            diagram.GetSystemIndexOrAbort(&source_output_port->get_system())) {}

  ~DiagramOutputPort() final = default;

  /// Obtain a reference to the subsystem output port that was exported to
  /// create this diagram port. Note that the source may itself be a diagram
  /// output port.
  const OutputPort<T>& get_source_output_port() const {
    return *source_output_port_;
  }

 private:
  // These forward to the source system output port, passing in just the source
  // System's Context, not the whole Diagram context we're given.
  std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>& context) const final {
    const Context<T>& subcontext = get_subcontext(context);
    return source_output_port_->Allocate(subcontext);
  }

  void DoCalc(const Context<T>& context, AbstractValue* value) const final {
    const Context<T>& subcontext = get_subcontext(context);
    return source_output_port_->Calc(subcontext, value);
  }

  const AbstractValue& DoEval(const Context<T>& context) const final {
    const Context<T>& subcontext = get_subcontext(context);
    return source_output_port_->Eval(subcontext);
  }

  // Dig out the right subcontext for delegation.
  const Context<T>& get_subcontext(const Context<T>& context) const {
    const DiagramContext<T>* diagram_context =
        dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    return diagram_context->GetSubsystemContext(subsystem_index_);
  }

  const OutputPort<T>* const source_output_port_;
  const int subsystem_index_;
};

//==============================================================================
//                             DIAGRAM OUTPUT
//==============================================================================
/// DiagramOutput is an implementation of SystemOutput that holds unowned
/// OutputPortValue pointers. It is used to expose the outputs of constituent
/// systems as outputs of a Diagram.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramOutput : public SystemOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramOutput)

  DiagramOutput() = default;

  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPortValue* get_mutable_port_value(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPortValue& get_port_value(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<OutputPortValue*>* get_mutable_port_values() { return &ports_; }

 protected:
  // Returns a clone that has the same number of output ports, with values
  // set to nullptr.
  DiagramOutput<T>* DoClone() const override {
    DiagramOutput<T>* clone = new DiagramOutput<T>();
    clone->ports_.resize(get_num_ports());
    return clone;
  }

 private:
  std::vector<OutputPortValue*> ports_;
};

//==============================================================================
//                          DIAGRAM TIME DERIVATIVES
//==============================================================================
/// DiagramTimeDerivatives is a version of DiagramContinuousState that owns
/// the constituent continuous states. As the name implies, it is only useful
/// for the time derivatives.
template <typename T>
class DiagramTimeDerivatives : public DiagramContinuousState<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramTimeDerivatives)

  explicit DiagramTimeDerivatives(
      std::vector<std::unique_ptr<ContinuousState<T>>>&& substates)
      : DiagramContinuousState<T>(Unpack(substates)),
        substates_(std::move(substates)) {}

  ~DiagramTimeDerivatives() override {}

 private:
  std::vector<std::unique_ptr<ContinuousState<T>>> substates_;
};

//==============================================================================
//                          DIAGRAM DISCRETE VARIABLES
//==============================================================================
/// DiagramDiscreteVariables is a version of DiscreteState that owns
/// the constituent discrete states. As the name implies, it is only useful
/// for the discrete updates.
template <typename T>
class DiagramDiscreteVariables : public DiscreteValues<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramDiscreteVariables)

  explicit DiagramDiscreteVariables(
      std::vector<std::unique_ptr<DiscreteValues<T>>>&& subdiscretes)
      : DiscreteValues<T>(Flatten(Unpack(subdiscretes))),
        subdiscretes_(std::move(subdiscretes)) {}

  ~DiagramDiscreteVariables() override {}

  int num_subdiscretes() const {
    return static_cast<int>(subdiscretes_.size());
  }

  DiscreteValues<T>* get_mutable_subdiscrete(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_subdiscretes());
    return subdiscretes_[index].get();
  }

 private:
  std::vector<BasicVector<T>*> Flatten(
      const std::vector<DiscreteValues<T>*>& in) const {
    std::vector<BasicVector<T>*> out;
    for (const DiscreteValues<T>* xd : in) {
      const std::vector<BasicVector<T>*>& xd_data = xd->get_data();
      out.insert(out.end(), xd_data.begin(), xd_data.end());
    }
    return out;
  }

  std::vector<std::unique_ptr<DiscreteValues<T>>> subdiscretes_;
};

}  // namespace internal

//==============================================================================
//                                  DIAGRAM
//==============================================================================
/// Diagram is a System composed of one or more constituent Systems, arranged
/// in a directed graph where the vertices are the constituent Systems
/// themselves, and the edges connect the output of one constituent System
/// to the input of another. To construct a Diagram, use a DiagramBuilder.
///
/// Each System in the Diagram must have a unique, non-empty name.
///
/// @tparam T The mathematical scalar type. Must be a valid Eigen scalar.
template <typename T>
class Diagram : public System<T>,
                public detail::InputPortEvaluatorInterface<T> {
 public:
  // Diagram objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Diagram)

  typedef typename std::pair<const System<T>*, int> PortIdentifier;

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Diagram(const Diagram<U>& other)
      : Diagram(other.template ConvertScalarType<T>()) {}

  ~Diagram() override {}

  /// Returns the list of contained Systems.
  std::vector<const systems::System<T>*> GetSystems() const {
    std::vector<const systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  std::multimap<int, int> GetDirectFeedthroughs() const final {
    std::multimap<int, int> pairs;
    for (int u = 0; u < this->get_num_input_ports(); ++u) {
      for (int v = 0; v < this->get_num_output_ports(); ++v) {
        if (DoHasDirectFeedthrough(u, v)) {
          pairs.emplace(u, v);
        }
      }
    }
    return pairs;
  };

  /// Allocates a DiagramEventCollection for this Diagram.
  /// @sa System::AllocateCompositeEventCollection().
  std::unique_ptr<CompositeEventCollection<T>>
  AllocateCompositeEventCollection() const final {
    const int num_systems = num_subsystems();
    std::vector<std::unique_ptr<CompositeEventCollection<T>>> subevents(
        num_systems);
    for (int i = 0; i < num_systems; ++i) {
      subevents[i] = registered_systems_[i]->AllocateCompositeEventCollection();
    }

    return std::make_unique<DiagramCompositeEventCollection<T>>(
        std::move(subevents));
  }

  std::unique_ptr<Context<T>> AllocateContext() const override {
    const int num_systems = num_subsystems();
    // Reserve inputs as specified during Diagram initialization.
    auto context = std::make_unique<DiagramContext<T>>(num_systems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_systems; ++i) {
      const System<T>* const sys = registered_systems_[i].get();
      auto subcontext = sys->AllocateContext();
      auto suboutput = sys->AllocateOutput(*subcontext);
      context->AddSystem(i, std::move(subcontext), std::move(suboutput));
    }

    // Wire up the Diagram-internal inputs and outputs.
    for (const auto& connection : dependency_graph_) {
      const PortIdentifier& src = connection.second;
      const PortIdentifier& dest = connection.first;
      context->Connect(ConvertToContextPortIdentifier(src),
                       ConvertToContextPortIdentifier(dest));
    }

    // Declare the Diagram-external inputs.
    for (const PortIdentifier& id : input_port_ids_) {
      context->ExportInput(ConvertToContextPortIdentifier(id));
    }

    context->MakeState();
    context->MakeParameters();
    return std::move(context);
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // Set default state of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto& subcontext = diagram_context->GetSubsystemContext(i);
      auto& substate = diagram_state->get_mutable_substate(i);
      registered_systems_[i]->SetDefaultState(subcontext, &substate);
    }
  }

  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* params) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    int numeric_parameter_offset = 0;
    int abstract_parameter_offset = 0;

    // Set default parameters of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto& subcontext = diagram_context->GetSubsystemContext(i);

      if (!subcontext.num_numeric_parameters() &&
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
      for (int j = 0; j < subcontext.num_numeric_parameters(); ++j) {
        numeric_params.push_back(&params->get_mutable_numeric_parameter(
            numeric_parameter_offset + j));
      }
      numeric_parameter_offset += subcontext.num_numeric_parameters();

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

      registered_systems_[i]->SetDefaultParameters(subcontext, &subparameters);
    }
  }

  void SetRandomState(const Context<T>& context, State<T>* state,
                      RandomGenerator* generator) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // Set state of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto& subcontext = diagram_context->GetSubsystemContext(i);
      auto& substate = diagram_state->get_mutable_substate(i);
      registered_systems_[i]->SetRandomState(subcontext, &substate, generator);
    }
  }

  void SetRandomParameters(const Context<T>& context, Parameters<T>* params,
                           RandomGenerator* generator) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    int numeric_parameter_offset = 0;
    int abstract_parameter_offset = 0;

    // Set parameters of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto& subcontext = diagram_context->GetSubsystemContext(i);

      if (!subcontext.num_numeric_parameters() &&
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
      for (int j = 0; j < subcontext.num_numeric_parameters(); ++j) {
        numeric_params.push_back(&params->get_mutable_numeric_parameter(
            numeric_parameter_offset + j));
      }
      numeric_parameter_offset += subcontext.num_numeric_parameters();
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

      registered_systems_[i]->SetRandomParameters(subcontext, &subparameters,
                                                  generator);
    }
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // The output ports of this Diagram are output ports of its constituent
    // systems. Create a DiagramOutput with that many ports.
    auto output = std::make_unique<internal::DiagramOutput<T>>();
    output->get_mutable_port_values()->resize(output_port_ids_.size());
    ExposeSubsystemOutputs(*diagram_context, output.get());
    return std::move(output);
  }

  /// @cond
  // The three methods below are hidden from doxygen, as described in
  // documentation for their corresponding methods in System.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const final {
    return AllocateForcedEventCollection<PublishEvent<T>>(
        &System<T>::AllocateForcedPublishEventCollection);
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const final {
    return AllocateForcedEventCollection<DiscreteUpdateEvent<T>>(
        &System<T>::AllocateForcedDiscreteUpdateEventCollection);
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const final {
    return AllocateForcedEventCollection<UnrestrictedUpdateEvent<T>>(
        &System<T>::AllocateForcedUnrestrictedUpdateEventCollection);
  }
  /// @endcond

  /// Aggregates the time derivatives from each subsystem into a
  /// DiagramTimeDerivatives.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const auto& system : registered_systems_) {
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    std::vector<std::unique_ptr<DiscreteValues<T>>> sub_discretes;
    for (const auto& system : registered_systems_) {
      sub_discretes.push_back(system->AllocateDiscreteVariables());
    }
    return std::unique_ptr<DiscreteValues<T>>(
        new internal::DiagramDiscreteVariables<T>(std::move(sub_discretes)));
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();
    DRAKE_DEMAND(num_subsystems() == n);

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      registered_systems_[i]->CalcTimeDerivatives(subcontext, subderivatives);
    }
  }

  /// Retrieves the state derivatives for a particular subsystem from the
  /// derivatives for the entire diagram. Aborts if @p subsystem is not
  /// actually a subsystem of this diagram. Returns nullptr if @p subsystem
  /// is stateless.
  const ContinuousState<T>* GetSubsystemDerivatives(
      const ContinuousState<T>& derivatives, const System<T>* subsystem) const {
    DRAKE_DEMAND(subsystem != nullptr);
    auto diagram_derivatives =
        dynamic_cast<const DiagramContinuousState<T>*>(&derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int i = GetSystemIndexOrAbort(subsystem);
    return diagram_derivatives->get_substate(i);
  }

  /// Returns a constant reference to the subcontext that corresponds to the
  /// system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  const Context<T>& GetSubsystemContext(const System<T>& subsystem,
                                        const Context<T>& context) const {
    auto ret = DoGetTargetSystemContext(subsystem, &context);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>& GetMutableSubsystemContext(const System<T>& subsystem,
                                         Context<T>* context) const {
    auto ret = DoGetMutableTargetSystemContext(subsystem, context);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the const subsystem composite event collection from @p events
  /// that corresponds to @p subsystem. Aborts if @p subsystem is not a
  /// subsystem of this diagram.
  const CompositeEventCollection<T>&
  GetSubsystemCompositeEventCollection(const System<T>& subsystem,
      const CompositeEventCollection<T>& events) const {
    auto ret = DoGetTargetSystemCompositeEventCollection(subsystem, &events);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the mutable subsystem composite event collection that corresponds
  /// to @p subsystem. Aborts if @p subsystem is not a subsystem of this
  /// diagram.
  CompositeEventCollection<T>& GetMutableSubsystemCompositeEventCollection(
      const System<T>& subsystem, CompositeEventCollection<T>* events) const {
    auto ret = DoGetMutableTargetSystemCompositeEventCollection(
        subsystem, events);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Retrieves the state for a particular subsystem from the context for the
  /// entire diagram. Invalidates all entries in that subsystem's cache that
  /// depend on State. Aborts if @p subsystem is not actually a subsystem of
  /// this diagram.
  ///
  /// TODO(david-german-tri): Provide finer-grained accessors for finer-grained
  /// invalidation.
  State<T>& GetMutableSubsystemState(const System<T>& subsystem,
                                     Context<T>* context) const {
    Context<T>& subcontext = GetMutableSubsystemContext(subsystem, context);
    return subcontext.get_mutable_state();
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  State<T>& GetMutableSubsystemState(const System<T>& subsystem,
                                     State<T>* state) const {
    auto ret = DoGetMutableTargetSystemState(subsystem, state);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  const State<T>& GetSubsystemState(const System<T>& subsystem,
                                    const State<T>& state) const {
    auto ret = DoGetTargetSystemState(subsystem, &state);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the full path of this Diagram in the tree of Diagrams. Implemented
  /// here to satisfy InputPortEvaluatorInterface, although we want the exact
  /// same behavior as in System.
  void GetPath(std::stringstream* output) const override {
    return System<T>::GetPath(output);
  }

  //----------------------------------------------------------------------------
  /// @name                      Graphviz methods
  //@{

  /// Returns a Graphviz fragment describing this Diagram. To obtain a complete
  /// Graphviz graph, call System<T>::GetGraphvizString.
  void GetGraphvizFragment(std::stringstream* dot) const override {
    // Open the Diagram.
    const int64_t id = this->GetGraphvizId();
    *dot << "subgraph cluster" << id << "diagram" " {" << std::endl;
    *dot << "color=black" << std::endl;
    *dot << "concentrate=true" << std::endl;
    std::string name = this->get_name();
    if (name.empty()) name = std::to_string(id);
    *dot << "label=\"" << name << "\";" << std::endl;

    // Add a cluster for the input port nodes.
    *dot << "subgraph cluster" << id << "inputports" << " {" << std::endl;
    *dot << "rank=same" << std::endl;
    *dot << "color=lightgrey" << std::endl;
    *dot << "style=filled" << std::endl;
    *dot << "label=\"input ports\"" << std::endl;
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      this->GetGraphvizInputPortToken(this->get_input_port(i), dot);
      *dot << "[color=blue, label=\"u" << i << "\"];" << std::endl;
    }
    *dot << "}" << std::endl;

    // Add a cluster for the output port nodes.
    *dot << "subgraph cluster" << id << "outputports" << " {" << std::endl;
    *dot << "rank=same" << std::endl;
    *dot << "color=lightgrey" << std::endl;
    *dot << "style=filled" << std::endl;
    *dot << "label=\"output ports\"" << std::endl;
    for (int i = 0; i < this->get_num_output_ports(); ++i) {
      this->GetGraphvizOutputPortToken(this->get_output_port(i), dot);
      *dot << "[color=green, label=\"y" << i << "\"];" << std::endl;
    }
    *dot << "}" << std::endl;

    // Add a cluster for the subsystems.
    *dot << "subgraph cluster" << id << "subsystems" << " {" << std::endl;
    *dot << "color=white" << std::endl;
    *dot << "label=\"\"" << std::endl;
    // -- Add the subsystems themselves.
    for (const auto& subsystem : registered_systems_) {
      subsystem->GetGraphvizFragment(dot);
    }
    // -- Add the connections as edges.
    for (const auto& edge : dependency_graph_) {
      const PortIdentifier& src = edge.second;
      const System<T>* src_sys = src.first;
      const PortIdentifier& dest = edge.first;
      const System<T>* dest_sys = dest.first;
      src_sys->GetGraphvizOutputPortToken(src_sys->get_output_port(src.second),
                                          dot);
      *dot << " -> ";
      dest_sys->GetGraphvizInputPortToken(dest_sys->get_input_port(dest.second),
                                          dot);
      *dot << ";" << std::endl;
    }

    // -- Add edges from the input and output port nodes to the subsystems that
    //    actually service that port.  These edges are higlighted in blue
    //    (input) and green (output), matching the port nodes.
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      const auto& port_id = input_port_ids_[i];
      this->GetGraphvizInputPortToken(this->get_input_port(i), dot);
      *dot << " -> ";
      port_id.first->GetGraphvizInputPortToken(
          port_id.first->get_input_port(port_id.second), dot);
      *dot << " [color=blue];" << std::endl;
    }

    for (int i = 0; i < this->get_num_output_ports(); ++i) {
      const auto& port_id = output_port_ids_[i];
      port_id.first->GetGraphvizOutputPortToken(
          port_id.first->get_output_port(port_id.second), dot);
      *dot << " -> ";
      this->GetGraphvizOutputPortToken(this->get_output_port(i), dot);
      *dot << " [color=green];" << std::endl;
    }
    *dot << "}" << std::endl;

    // Close the diagram.
    *dot << "}" << std::endl;
  }

  void GetGraphvizInputPortToken(const InputPortDescriptor<T>& port,
                                 std::stringstream* dot) const override {
    DRAKE_DEMAND(port.get_system() == this);
    *dot << "_" << this->GetGraphvizId() << "_u" << port.get_index();
  }

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  std::stringstream* dot) const override {
    DRAKE_DEMAND(&port.get_system() == this);
    *dot << "_" << this->GetGraphvizId() << "_y" << port.get_index();
  }

  //@}

  /// Evaluates the value of the subsystem input port with the given @p id
  /// in the given @p context. Satisfies InputPortEvaluatorInterface.
  ///
  /// This is a framework implementation detail. User code should not call
  /// this function.
  void EvaluateSubsystemInputPort(
      const Context<T>* context,
      const InputPortDescriptor<T>& descriptor) const override {
    DRAKE_DEMAND(context != nullptr);
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(context);
    DRAKE_DEMAND(diagram_context != nullptr);
    const PortIdentifier id{descriptor.get_system(), descriptor.get_index()};

    // Find if this input port is exported.
    const auto external_it =
        std::find(input_port_ids_.begin(), input_port_ids_.end(), id);
    const bool is_exported = (external_it != input_port_ids_.end());

    // Find if this input port is connected to an output port.
    const auto upstream_it = dependency_graph_.find(id);
    const bool is_connected = (upstream_it != dependency_graph_.end());

    DRAKE_DEMAND(is_exported ^ is_connected);

    if (is_exported) {
      // The upstream output port is an input of this whole Diagram; ask our
      // parent to evaluate it.
      const int i = external_it - input_port_ids_.begin();
      this->EvalInputPort(*diagram_context, i);
    } else {
      // The upstream output port exists in this Diagram; evaluate it.
      // TODO(david-german-tri): Add online algebraic loop detection here.
      DRAKE_ASSERT(is_connected);
      const PortIdentifier& prerequisite = upstream_it->second;
      this->EvaluateOutputPort(*diagram_context, prerequisite);
    }
  }

  /// Returns the index of the given @p sys in this diagram, or aborts if @p sys
  /// is not a member of the diagram.
  int GetSystemIndexOrAbort(const System<T>* sys) const {
    auto it = system_index_map_.find(sys);
    DRAKE_DEMAND(it != system_index_map_.end());
    return it->second;
  }

 protected:
  /// Constructs an uninitialized Diagram. Subclasses that use this constructor
  /// are obligated to call DiagramBuilder::BuildInto(this).  Provides scalar-
  /// type conversion support only if every contained subsystem provides the
  /// same support.
  Diagram() : System<T>(
      SystemScalarConverter(
          SystemTypeTag<systems::Diagram>{},
          SystemScalarConverter::GuaranteedSubtypePreservation::kDisabled)) {}

  /// (Advanced) Constructs an uninitialized Diagram.  Subclasses that use this
  /// constructor are obligated to call DiagramBuilder::BuildInto(this).
  ///
  /// Declares scalar-type conversion support using @p converter.  Support for
  /// a given pair of types `T, U` to convert from and to will be enabled only
  /// if every contained subsystem supports that pair.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  explicit Diagram(SystemScalarConverter converter)
      : System<T>(std::move(converter)) {}

  /// For the subsystem associated with @p witness_func, gets its subcontext
  /// from @p context, passes the subcontext to @p witness_func' Evaulate
  /// method and returns the result. Aborts if the subsystem is not part of
  /// this Diagram.
  T DoEvaluateWitness(const Context<T>& context,
                      const WitnessFunction<T>& witness_func) const final {
    const System<T>& system = witness_func.get_system();
    const Context<T>& subcontext = GetSubsystemContext(system, context);
    return witness_func.Evaluate(subcontext);
  }

  /// For the subsystem associated with @p witness_func, gets its mutable
  /// sub composite event collection from @p events, and passes it to
  /// @p witness_func's AddEvent method. Aborts if the subsystem is not part of
  /// this Diagram.
  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      const WitnessFunction<T>& witness_func,
      CompositeEventCollection<T>* events) const final {
    DRAKE_DEMAND(events);
    const System<T>& subsystem = witness_func.get_system();
    CompositeEventCollection<T>& subevents =
        GetMutableSubsystemCompositeEventCollection(subsystem, events);
    witness_func.AddEvent(&subevents);
  }

  /// Provides witness functions of subsystems that are active at the beginning
  /// of a continuous time interval. The vector of witness functions is not
  /// ordered in a particular manner.
  void DoGetWitnessFunctions(const Context<T>& context,
                std::vector<const WitnessFunction<T>*>* witnesses) const final {
    // A temporary vector is necessary since the vector of witnesses is
    // declared to be empty on entry to DoGetWitnessFunctions().
    std::vector<const WitnessFunction<T>*> temp_witnesses;

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    int index = 0;  // The subsystem index.

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

  /// Returns a pointer to mutable context if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  Context<T>* DoGetMutableTargetSystemContext(
      const System<T>& target_system, Context<T>* context) const final {
    if (&target_system == this)
      return context;

    return GetSubsystemStuff<Context<T>, DiagramContext<T>>(
        target_system, context,
        &System<T>::DoGetMutableTargetSystemContext,
        &DiagramContext<T>::GetMutableSubsystemContext);
  }

  /// Returns a pointer to const context if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  const Context<T>* DoGetTargetSystemContext(
      const System<T>& target_system, const Context<T>* context) const final {
    if (&target_system == this)
      return context;

    return GetSubsystemStuff<const Context<T>, const DiagramContext<T>>(
        target_system, context,
        &System<T>::DoGetTargetSystemContext,
        &DiagramContext<T>::GetSubsystemContext);
  }

  /// Returns a pointer to mutable state if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  State<T>* DoGetMutableTargetSystemState(
      const System<T>& target_system, State<T>* state) const final {
    if (&target_system == this)
      return state;

    return GetSubsystemStuff<State<T>, DiagramState<T>>(
        target_system, state,
        &System<T>::DoGetMutableTargetSystemState,
        &DiagramState<T>::get_mutable_substate);
  }

  /// Returns a pointer to const state if @p target_system is a subsystem
  /// of this, nullptr is returned otherwise.
  const State<T>* DoGetTargetSystemState(
      const System<T>& target_system, const State<T>* state) const final {
    if (&target_system == this)
      return state;

    return GetSubsystemStuff<const State<T>, const DiagramState<T>>(
        target_system, state,
        &System<T>::DoGetTargetSystemState,
        &DiagramState<T>::get_substate);
  }

  /// Returns a pointer to mutable composite event collection if
  /// @p target_system is a subsystem of this, nullptr is returned otherwise.
  CompositeEventCollection<T>* DoGetMutableTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      CompositeEventCollection<T>* events) const final {
    if (&target_system == this)
      return events;

    return GetSubsystemStuff<CompositeEventCollection<T>,
                             DiagramCompositeEventCollection<T>>(
        target_system, events,
        &System<T>::DoGetMutableTargetSystemCompositeEventCollection,
        &DiagramCompositeEventCollection<T>::get_mutable_subevent_collection);
  }

  /// Returns a pointer to const composite event collection if
  /// @p target_system is a subsystem of this, nullptr is returned otherwise.
  const CompositeEventCollection<T>* DoGetTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      const CompositeEventCollection<T>* events) const final {
    if (&target_system == this)
      return events;

    return GetSubsystemStuff<const CompositeEventCollection<T>,
                             const DiagramCompositeEventCollection<T>>(
        target_system, events,
        &System<T>::DoGetTargetSystemCompositeEventCollection,
        &DiagramCompositeEventCollection<T>::get_subevent_collection);
  }

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const override {
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
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

      // Select the chunk of generalized_velocity belonging to subsystem i.
      const int num_v = sub_xc.get_generalized_velocity().size();
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

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapQDotToVelocity(const Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           VectorBase<T>* generalized_velocity) const override {
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
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      const ContinuousState<T>& sub_xc = subcontext.get_continuous_state();

      // Select the chunk of qdot belonging to subsystem i.
      const int num_q = sub_xc.get_generalized_position().size();
      const Eigen::Ref<const VectorX<T>>& dq_slice =
          qdot.segment(q_index, num_q);

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

  /// Computes the next update time based on the configured actions, for scalar
  /// types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            CompositeEventCollection<T>* event_info,
                            T* time) const override {
    DoCalcNextUpdateTimeImpl(context, event_info, time);
  }

  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputVector(subsystem->get_input_port(subindex))
        .release();
  }

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputAbstract(subsystem->get_input_port(subindex))
        .release();
  }

 private:
  // Returns true if there might be direct feedthrough from the given
  // @p input_port of the Diagram to the given @p output_port of the Diagram.
  bool DoHasDirectFeedthrough(int input_port, int output_port) const {
    DRAKE_ASSERT(input_port >= 0);
    DRAKE_ASSERT(input_port < this->get_num_input_ports());
    DRAKE_ASSERT(output_port >= 0);
    DRAKE_ASSERT(output_port < this->get_num_output_ports());

    const PortIdentifier& target_input_id = input_port_ids_[input_port];

    // Search the graph for a direct-feedthrough connection from the output_port
    // back to the input_port. Maintain a set of the output port identifiers
    // that are known to have a direct-feedthrough path to the output_port.
    std::set<PortIdentifier> active_set;
    active_set.insert(output_port_ids_[output_port]);
    while (!active_set.empty()) {
      const PortIdentifier current_output_id = *active_set.begin();
      size_t removed_count = active_set.erase(current_output_id);
      DRAKE_ASSERT(removed_count == 1);
      const System<T>* sys = current_output_id.first;
      for (int i = 0; i < sys->get_num_input_ports(); ++i) {
        if (sys->HasDirectFeedthrough(i, current_output_id.second)) {
          const PortIdentifier curr_input_id(sys, i);
          if (curr_input_id == target_input_id) {
            // We've found a direct-feedthrough path to the input_port.
            return true;
          } else {
            // We've found an intermediate input port has a direct-feedthrough
            // path to the output_port. Add the upstream output port (if there
            // is one) to the active set.
            auto it = dependency_graph_.find(curr_input_id);
            if (it != dependency_graph_.end()) {
              const PortIdentifier& upstream_output = it->second;
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

  template <typename EventType>
  std::unique_ptr<EventCollection<EventType>> AllocateForcedEventCollection(
      std::function<
      std::unique_ptr<EventCollection<EventType>>(const System<T>*)>
  allocater_func) const {
    const int num_systems = num_subsystems();
    auto ret = std::make_unique<DiagramEventCollection<EventType>>(num_systems);
    for (int i = 0; i < num_systems; ++i) {
      std::unique_ptr<EventCollection<EventType>> subevent_collection =
          allocater_func(registered_systems_[i].get());
      ret->set_and_own_subevent_collection(i, std::move(subevent_collection));
    }
    return std::move(ret);
  }

  // For each subsystem, if there is a publish event in its corresponding
  // subevent collection, calls its Publish method with the appropriate
  // subcontext and subevent collection.
  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& event_info) const final {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context);
    const DiagramEventCollection<PublishEvent<T>>& info =
        dynamic_cast<const DiagramEventCollection<PublishEvent<T>>&>(
            event_info);

    for (int i = 0; i < num_subsystems(); ++i) {
      const EventCollection<PublishEvent<T>>& subinfo =
          info.get_subevent_collection(i);

      if (subinfo.HasEvents()) {
        const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
        registered_systems_[i]->Publish(subcontext, subinfo);
      }
    }
  }

  // For each subsystem, if there is a discrete update event in its
  // corresponding subevent collection, calls its CalcDiscreteVariableUpdates
  // method with the appropriate subcontext, subevent collection and
  // substate.
  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& event_info,
      DiscreteValues<T>* discrete_state) const final {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context);
    auto diagram_discrete =
        dynamic_cast<internal::DiagramDiscreteVariables<T>*>(discrete_state);
    DRAKE_DEMAND(diagram_discrete);

    // As a baseline, initialize all the discrete variables to their
    // current values.
    // TODO(siyuan): should have a API level CopyFrom for DiscreteValues.
    for (int i = 0; i < diagram_discrete->num_groups(); ++i) {
      diagram_discrete->get_mutable_vector(i).set_value(
          context.get_discrete_state(i).get_value());
    }

    const DiagramEventCollection<DiscreteUpdateEvent<T>>& info =
        dynamic_cast<const DiagramEventCollection<DiscreteUpdateEvent<T>>&>(
            event_info);

    for (int i = 0; i < num_subsystems(); ++i) {
      const EventCollection<DiscreteUpdateEvent<T>>& subinfo =
          info.get_subevent_collection(i);

      if (subinfo.HasEvents()) {
        const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
        DiscreteValues<T>* subdiscrete =
            diagram_discrete->get_mutable_subdiscrete(i);
        DRAKE_DEMAND(subdiscrete != nullptr);

        registered_systems_[i]->CalcDiscreteVariableUpdates(subcontext, subinfo,
                                                            subdiscrete);
      }
    }
  }

  // For each subsystem, if there is an unrestricted update event in its
  // corresponding subevent collection, calls its CalcUnrestrictedUpdate
  // method with the appropriate subcontext, subevent collection and substate.
  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& event_info,
      State<T>* state) const final {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context);
    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // No need to set state to context's state, since it has already been done
    // in System::CalcUnrestrictedUpdate().

    const DiagramEventCollection<UnrestrictedUpdateEvent<T>>& info =
        dynamic_cast<const DiagramEventCollection<UnrestrictedUpdateEvent<T>>&>(
            event_info);

    for (int i = 0; i < num_subsystems(); ++i) {
      const EventCollection<UnrestrictedUpdateEvent<T>>& subinfo =
          info.get_subevent_collection(i);

      if (subinfo.HasEvents()) {
        const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
        State<T>& substate = diagram_state->get_mutable_substate(i);

        registered_systems_[i]->CalcUnrestrictedUpdate(subcontext, subinfo,
            &substate);
      }
    }
  }

  /// Tries to recursively find @p target_system's BaseStuff
  /// (context / state / etc). nullptr is returned if @p target_system is not
  /// a subsystem of this diagram. This template function should only be used
  /// to reduce code repetition for DoGetMutableTargetSystemContext(),
  /// DoGetTargetSystemContext(), DoGetMutableTargetSystemState(), and
  /// DoGetTargetSystemState().
  /// @param target_system The sub system of interest.
  /// @param my_stuff BaseStuff that's associated with this diagram.
  /// @param recursive_getter A member function of System that returns sub
  /// context or state. Should be one of the four functions listed above.
  /// @param get_child_stuff A member function of DiagramContext or DiagramState
  /// that returns context or state given the index of the subsystem.
  ///
  /// @tparam BaseStuff Can be Context<T>, const Context<T>, State<T> and
  /// const State<T>.
  /// @tparam DerivedStuff Can be DiagramContext<T>,
  /// const DiagramContext<T>, DiagramState<T> and const DiagramState<T>.
  ///
  /// @pre @p target_system cannot be `this`. The caller should check for this
  /// edge case.
  template <typename BaseStuff, typename DerivedStuff>
  BaseStuff* GetSubsystemStuff(
      const System<T>& target_system, BaseStuff* my_stuff,
      std::function<BaseStuff* (const System<T>*, const System<T>&, BaseStuff*)>
          recursive_getter,
      std::function<BaseStuff& (DerivedStuff*, int)> get_child_stuff) const {
    static_assert(
        std::is_same<BaseStuff,
                     typename std::remove_pointer<BaseStuff>::type>::value,
        "BaseStuff cannot be a pointer");
    static_assert(
        std::is_same<DerivedStuff,
                     typename std::remove_pointer<DerivedStuff>::type>::value,
        "DerivedStuff cannot be a pointer");

    DRAKE_DEMAND(my_stuff != nullptr);
    DRAKE_DEMAND(&target_system != this);
    DerivedStuff& my_stuff_as_derived = dynamic_cast<DerivedStuff&>(*my_stuff);

    int index = 0;
    for (const auto& child : registered_systems_) {
      BaseStuff& child_stuff =
          get_child_stuff(&my_stuff_as_derived, index);

      BaseStuff* const target_stuff =
          recursive_getter(child.get(), target_system, &child_stuff);

      if (target_stuff != nullptr) {
        return target_stuff;
      }
      index++;
    }

    return nullptr;
  }

  /// Uses this Diagram<T> to manufacture a Diagram<NewType>::Blueprint,
  /// using system scalar conversion.
  ///
  /// @tparam NewType The scalar type to which to convert.
  template <typename NewType>
  std::unique_ptr<typename Diagram<NewType>::Blueprint> ConvertScalarType()
      const {
    std::vector<std::unique_ptr<System<NewType>>> new_systems;
    // Recursively convert all the subsystems.
    std::map<const System<T>*, const System<NewType>*> old_to_new_map;
    for (const auto& old_system : registered_systems_) {
      // Convert old_system to new_system using the old_system's converter.
      std::unique_ptr<System<NewType>> new_system =
          old_system->get_system_scalar_converter().
          template Convert<NewType>(*old_system);
      DRAKE_DEMAND(new_system != nullptr);

      // Update our mapping and take ownership.
      old_to_new_map[old_system.get()] = new_system.get();
      new_systems.push_back(std::move(new_system));
    }

    // Set up the blueprint.
    auto blueprint = std::make_unique<typename Diagram<NewType>::Blueprint>();
    // Make all the inputs and outputs.
    for (const PortIdentifier& id : input_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint->input_port_ids.emplace_back(new_system, port);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint->output_port_ids.emplace_back(new_system, port);
    }
    // Make all the connections.
    for (const auto& edge : dependency_graph_) {
      const PortIdentifier& old_dest = edge.first;
      const System<NewType>* const dest_system = old_to_new_map[old_dest.first];
      const int dest_port = old_dest.second;
      const typename Diagram<NewType>::PortIdentifier new_dest{dest_system,
                                                               dest_port};

      const PortIdentifier& old_src = edge.second;
      const System<NewType>* const src_system = old_to_new_map[old_src.first];
      const int src_port = old_src.second;
      const typename Diagram<NewType>::PortIdentifier new_src{src_system,
                                                              src_port};

      blueprint->dependency_graph[new_dest] = new_src;
    }
    // Move the new systems into the blueprint.
    blueprint->systems = std::move(new_systems);

    return blueprint;
  }

  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>&, CompositeEventCollection<T1>*,
                           T1*) const {
    DRAKE_ABORT_MSG(
        "The default implementation of Diagram<T>::DoCalcNextUpdateTime "
        "only works with types that are drake::is_numeric.");
  }

  // Computes the next update time across all the scheduled events, for
  // scalar types that are numeric.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type DoCalcNextUpdateTimeImpl(
      const Context<T1>& context, CompositeEventCollection<T1>* event_info,
      T1* time) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T1>*>(&context);
    auto info = dynamic_cast<DiagramCompositeEventCollection<T1>*>(event_info);
    DRAKE_DEMAND(diagram_context != nullptr);
    DRAKE_DEMAND(info != nullptr);

    *time = std::numeric_limits<T1>::infinity();

    // Iterate over the subsystems, and harvest the most imminent updates.
    std::vector<T1> times(num_subsystems());
    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T1>& subcontext = diagram_context->GetSubsystemContext(i);
      CompositeEventCollection<T1>& subinfo =
          info->get_mutable_subevent_collection(i);
      const T1 sub_time =
          registered_systems_[i]->CalcNextUpdateTime(subcontext, &subinfo);
      times[i] = sub_time;

      if (sub_time < *time) {
        *time = sub_time;
      }
    }

    // For all the subsystems whose next update time is bigger than *time,
    // clear their event collections.
    for (int i = 0; i < num_subsystems(); ++i) {
      if (times[i] > *time)
        info->get_mutable_subevent_collection(i).Clear();
    }
  }

  std::map<typename Event<T>::PeriodicAttribute, std::vector<const Event<T>*>,
      PeriodicAttributeComparator<T>> DoGetPeriodicEvents() const override {
    std::map<typename Event<T>::PeriodicAttribute,
        std::vector<const Event<T>*>,
        PeriodicAttributeComparator<T>> periodic_events_map;

    for (int i = 0; i < num_subsystems(); ++i) {
      auto sub_map = registered_systems_[i]->GetPeriodicEvents();
      for (const auto& sub_attr_events : sub_map) {
        const auto& sub_vec = sub_attr_events.second;
        auto& vec = periodic_events_map[sub_attr_events.first];
        vec.insert(vec.end(), sub_vec.begin(), sub_vec.end());
      }
    }

    return periodic_events_map;
  }

  void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* event_info) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
    DRAKE_DEMAND(diagram_context != nullptr);
    DRAKE_DEMAND(info != nullptr);

    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      CompositeEventCollection<T>& subinfo =
          info->get_mutable_subevent_collection(i);

      registered_systems_[i]->GetPerStepEvents(subcontext, &subinfo);
    }
  }

  void DoGetInitializationEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* event_info) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    auto info = dynamic_cast<DiagramCompositeEventCollection<T>*>(event_info);
    DRAKE_DEMAND(diagram_context != nullptr);
    DRAKE_DEMAND(info != nullptr);

    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T>& subcontext = diagram_context->GetSubsystemContext(i);
      CompositeEventCollection<T>& subinfo =
          info->get_mutable_subevent_collection(i);

      registered_systems_[i]->GetInitializationEvents(subcontext, &subinfo);
    }
  }

  // A structural outline of a Diagram, produced by DiagramBuilder.
  struct Blueprint {
    // The ordered subsystem ports that are inputs to the entire diagram.
    std::vector<PortIdentifier> input_port_ids;
    // The ordered subsystem ports that are outputs of the entire diagram.
    std::vector<PortIdentifier> output_port_ids;
    // A map from the input ports of constituent systems to the output ports
    // on which they depend. This graph is possibly cyclic, but must not
    // contain an algebraic loop.
    std::map<PortIdentifier, PortIdentifier> dependency_graph;
    // All of the systems to be included in the diagram.
    std::vector<std::unique_ptr<System<T>>> systems;
  };

  // Constructs a Diagram from the Blueprint that a DiagramBuilder produces.
  // This constructor is private because only DiagramBuilder calls it. The
  // constructor takes the systems from the blueprint.
  explicit Diagram(std::unique_ptr<Blueprint> blueprint) : Diagram() {
    Initialize(std::move(blueprint));
  }

  // Validates the given @p blueprint and sets up the Diagram accordingly.
  void Initialize(std::unique_ptr<Blueprint> blueprint) {
    // We must be given something to own.
    DRAKE_DEMAND(!blueprint->systems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(registered_systems_.empty());

    // Copy the data from the blueprint into private member variables.
    dependency_graph_ = std::move(blueprint->dependency_graph);
    input_port_ids_ = std::move(blueprint->input_port_ids);
    output_port_ids_ = std::move(blueprint->output_port_ids);
    registered_systems_ = std::move(blueprint->systems);

    // Generate a map from the System pointer to its index in the registered
    // order.
    for (int i = 0; i < num_subsystems(); ++i) {
      system_index_map_[registered_systems_[i].get()] = i;
      registered_systems_[i]->set_parent(this);
    }

    // Generate constraints for the diagram from the constraints on the
    // subsystems.
    for (int i = 0; i < num_subsystems(); ++i) {
      const auto sys = registered_systems_[i].get();
      for (SystemConstraintIndex j(0); j < sys->get_num_constraints(); ++j) {
        const auto c = &(sys->get_constraint(j));
        typename SystemConstraint<T>::CalcCallback diagram_calc =
            [this, sys, c](const Context<T>& context, VectorX<T>* value) {
              c->Calc(this->GetSubsystemContext(*sys, context), value);
            };
        this->AddConstraint(std::make_unique<SystemConstraint<T>>(
            diagram_calc, c->size(), c->type(),
            sys->get_name() + ":" + c->description()));
      }
    }

    // Every system must appear exactly once.
    DRAKE_DEMAND(registered_systems_.size() == system_index_map_.size());
    // Every port named in the dependency_graph_ must actually exist.
    DRAKE_ASSERT(PortsAreValid());
    // Every subsystem must have a unique name.
    DRAKE_THROW_UNLESS(NamesAreUniqueAndNonEmpty());

    // Add the inputs to the Diagram topology, and check their invariants.
    for (const PortIdentifier& id : input_port_ids_) {
      ExportInput(id);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      ExportOutput(id);
    }

    // Identify the intersection of the subsystems' scalar conversion support.
    // Remove all conversions that at least one subsystem did not support.
    SystemScalarConverter& this_scalar_converter =
        SystemImpl::get_mutable_system_scalar_converter(this);
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
  }

  // Exposes the given port as an input of the Diagram.
  void ExportInput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the diagram.
    GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = sys->get_input_port(port_index);
    this->DeclareInputPort(subsystem_descriptor.get_data_type(),
                           subsystem_descriptor.size(),
                           subsystem_descriptor.get_random_type());
  }

  // Exposes the given subsystem output port as an output of the Diagram.
  void ExportOutput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    const auto& source_output_port = sys->get_output_port(port_index);
    auto diagram_port = std::make_unique<internal::DiagramOutputPort<T>>(
        *this, &source_output_port);
    this->CreateOutputPort(std::move(diagram_port));
  }

  // Evaluates the value of the output port with the given @p id in the given
  // @p context.
  //
  // TODO(david-german-tri): Add Diagram-level cache entries to keep track of
  // whether a given output port has already been evaluated.  Right now, we
  // are recomputing every intermediate output to satisfy every system that
  // depends on it, recursively. This is O(N^2 * M), where M is the number of
  // output ports the Diagram exposes, and N is the number of intermediate
  // output ports the Diagram contains.
  void EvaluateOutputPort(const DiagramContext<T>& context,
                          const PortIdentifier& id) const {
    const System<T>* const system = id.first;
    const OutputPortIndex port_index(id.second);
    const OutputPort<T>& port = system->get_output_port(port_index);
    const int i = GetSystemIndexOrAbort(system);
    SPDLOG_TRACE(log(), "Evaluating output for subsystem {}, port {}",
                 system->GetPath(), port_index);
    const Context<T>& subsystem_context = context.GetSubsystemContext(i);
    SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(i);
    AbstractValue* port_output = subsystem_output->GetMutableData(port_index);
    port.Calc(subsystem_context, port_output);
  }

  // Converts a PortIdentifier to a DiagramContext::PortIdentifier.
  // The DiagramContext::PortIdentifier contains the index of the System in the
  // the diagram, instead of an actual pointer to the System.
  typename DiagramContext<T>::PortIdentifier ConvertToContextPortIdentifier(
      const PortIdentifier& id) const {
    typename DiagramContext<T>::PortIdentifier output;
    output.first = GetSystemIndexOrAbort(id.first);
    output.second = id.second;
    return output;
  }

  // Sets up the OutputPortValue pointers in @p output to point to the subsystem
  // output values, found in @p context, that are the outputs of this Diagram.
  void ExposeSubsystemOutputs(const DiagramContext<T>& context,
                              internal::DiagramOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const int num_ports = static_cast<int>(output_port_ids_.size());
    DRAKE_DEMAND(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
      const PortIdentifier& id = output_port_ids_[i];
      // For each configured output port ID, obtain from the DiagramContext the
      // actual OutputPortValue that supplies its value.
      const int sys_index = GetSystemIndexOrAbort(id.first);
      const int port_index = id.second;
      SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(sys_index);
      OutputPortValue* output_port_value =
          subsystem_output->get_mutable_port_value(port_index);

      // Then, put a pointer to that OutputPortValue in the DiagramOutput.
      (*output->get_mutable_port_values())[i] = output_port_value;
    }
  }

  // Returns true if every port mentioned in the dependency_graph_ exists.
  bool PortsAreValid() const {
    for (const auto& entry : dependency_graph_) {
      const PortIdentifier& dest = entry.first;
      const PortIdentifier& src = entry.second;
      if (dest.second < 0 || dest.second >= dest.first->get_num_input_ports()) {
        return false;
      }
      if (src.second < 0 || src.second >= src.first->get_num_output_ports()) {
        return false;
      }
    }
    return true;
  }

  // Returns true if every subsystem has a unique, non-empty name.
  // O(N * log(N)) in the number of subsystems.
  bool NamesAreUniqueAndNonEmpty() const {
    std::set<std::string> names;
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

  int num_subsystems() const {
    return static_cast<int>(registered_systems_.size());
  }

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The Systems in this Diagram, which are owned by this Diagram, in the order
  // they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // Map to quickly satisify "What is the subsytem index of the child system?"
  std::map<const System<T>*, int> system_index_map_;

  // The ordered inputs and outputs of this Diagram.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For all T, Diagram<T> considers DiagramBuilder<T> a friend, so that the
  // builder can set the internal state correctly.
  friend class DiagramBuilder<T>;

  // For any T1 & T2, Diagram<T1> considers Diagram<T2> a friend, so that
  // Diagram can provide transmogrification methods across scalar types.
  // See Diagram<T>::ConvertScalarType.
  template <typename> friend class Diagram;
};

}  // namespace systems
}  // namespace drake
