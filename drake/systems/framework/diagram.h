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
#include "drake/common/symbolic_expression.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"

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

/// A vector of pair of subsystem id and its DiscreteEvent.
template <typename T>
using SubsystemIdAndEventPairs = std::vector<std::pair<int, DiscreteEvent<T>>>;

/// For a subsystem identified by @p subsystem_id, sorts all its discrete events
/// @p subsystem_events based on their event types. The results are appended to
/// @p all_publish, @p all_discrete_update and @p all_unrestricted_update.
template <typename T>
void FilterSubsystemEventsByType(int subsystem_id,
    const std::vector<DiscreteEvent<T>>& subsystem_events,
    SubsystemIdAndEventPairs<T>* all_publish,
    SubsystemIdAndEventPairs<T>* all_discrete_update,
    SubsystemIdAndEventPairs<T>* all_unrestricted_update) {
  DRAKE_ASSERT(all_publish != nullptr);
  DRAKE_ASSERT(all_discrete_update != nullptr);
  DRAKE_ASSERT(all_unrestricted_update != nullptr);
  for (const auto& event : subsystem_events) {
    switch (event.action) {
      case DiscreteEvent<T>::kPublishAction:
        all_publish->emplace_back(subsystem_id, event);
        break;
      case DiscreteEvent<T>::kDiscreteUpdateAction:
        all_discrete_update->emplace_back(subsystem_id, event);
        break;
      case DiscreteEvent<T>::kUnrestrictedUpdateAction:
        all_unrestricted_update->emplace_back(subsystem_id, event);
        break;
      default:
        DRAKE_ABORT_MSG("Unknown ActionType.");
        break;
    }
  }
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
    return *diagram_context->GetSubsystemContext(subsystem_index_);
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

  int num_subdifferences() const {
    return static_cast<int>(subdiscretes_.size());
  }

  DiscreteValues<T>* get_mutable_subdifference(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_subdifferences());
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

  /// Returns true if any output of the Diagram might have direct-feedthrough
  /// from any input of the Diagram. The implementation is quite conservative:
  /// it will return true if there is any path on the directed acyclic graph
  /// of subsystems that begins at any input port to the Diagram, and ends at
  /// any System producing an output port of the Diagram, such that every System
  /// in that path HasAnyDirectFeedthrough.
  bool HasAnyDirectFeedthrough() const final {
    for (int i = 0; i < this->get_num_output_ports(); i++) {
      if (HasDirectFeedthrough(i)) {
        return true;
      }
    }
    return false;
  }

  /// Returns true if the given @p output_port of the Diagram might have
  /// direct-feedthrough from any input of the Diagram. The implementation is
  /// quite conservative: it will return true if there is any path on the
  /// directed acyclic graph of subsystems that begins at any input port to
  /// the Diagram, and ends at the system producing the given @p output_port,
  /// such that every System in that path HasAnyDirectFeedthrough.
  bool HasDirectFeedthrough(int output_port) const final {
    DRAKE_ASSERT(output_port >= 0);
    DRAKE_ASSERT(output_port < this->get_num_output_ports());
    for (int i = 0; i < this->get_num_input_ports(); i++) {
      if (HasDirectFeedthrough(i, output_port)) {
        return true;
      }
    }
    return false;
  }

  /// Returns true if there might be direct feedthrough from the given
  /// @p input_port of the Diagram to the given @p output_port of the Diagram.
  bool HasDirectFeedthrough(int input_port, int output_port) const final {
    DRAKE_ASSERT(input_port >= 0);
    DRAKE_ASSERT(input_port < this->get_num_input_ports());
    DRAKE_ASSERT(output_port >= 0);
    DRAKE_ASSERT(output_port < this->get_num_output_ports());

    const PortIdentifier& target_id = input_port_ids_[input_port];

    // Search the graph for a direct-feedthrough connection from the output_port
    // back to the input_port. Maintain a set of the output port identifiers
    // that are known to have a direct-feedthrough path to the output_port.
    std::set<PortIdentifier> active_set;
    active_set.insert(output_port_ids_[output_port]);
    while (!active_set.empty()) {
      const PortIdentifier current_id = *active_set.begin();
      active_set.erase(current_id);
      const System<T>* sys = current_id.first;
      for (int i = 0; i < sys->get_num_input_ports(); ++i) {
        if (sys->HasDirectFeedthrough(i, current_id.second)) {
          if (sys == target_id.first) {
            // We've found a direct-feedthrough path to the input_port.
            return true;
          } else {
            // We've found an intermediate input port has a direct-feedthrough
            // path to the output_port. Add the upstream output port (if there
            // is one) to the active set.
            const PortIdentifier pos(sys, i);
            auto it = dependency_graph_.find(pos);
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

  std::unique_ptr<Context<T>> AllocateContext() const override {
    const int num_systems = num_subsystems();
    // Reserve inputs as specified during Diagram initialization.
    auto context = std::make_unique<DiagramContext<T>>(num_systems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_systems; ++i) {
      const System<T>* const sys = sorted_systems_[i];
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
      auto subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      auto substate = diagram_state->get_mutable_substate(i);
      DRAKE_DEMAND(substate != nullptr);
      sorted_systems_[i]->SetDefaultState(*subcontext, substate);
    }
  }

  void SetDefaults(Context<T>* context) const final {
    auto diagram_context = dynamic_cast<DiagramContext<T>*>(context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Set defaults of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto subcontext = diagram_context->GetMutableSubsystemContext(i);
      sorted_systems_[i]->SetDefaults(subcontext);
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

  /// Aggregates the time derivatives from each subsystem into a
  /// DiagramTimeDerivatives.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const System<T>* const system : sorted_systems_) {
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    std::vector<std::unique_ptr<DiscreteValues<T>>> sub_differences;
    for (const System<T>* const system : sorted_systems_) {
      sub_differences.push_back(system->AllocateDiscreteVariables());
    }
    return std::unique_ptr<DiscreteValues<T>>(
        new internal::DiagramDiscreteVariables<T>(std::move(sub_differences)));
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
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      sorted_systems_[i]->CalcTimeDerivatives(*subcontext, subderivatives);
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
  const Context<T>& GetSubsystemContext(const Context<T>& context,
                                        const System<T>* subsystem) const {
    auto ret = DoGetTargetSystemContext(subsystem, &context);
    DRAKE_DEMAND(ret != nullptr);
    return *ret;
  }

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>* GetMutableSubsystemContext(Context<T>* context,
                                         const System<T>* subsystem) const {
    auto ret = DoGetMutableTargetSystemContext(subsystem, context);
    DRAKE_DEMAND(ret != nullptr);
    return ret;
  }

  /// Retrieves the state for a particular subsystem from the context for the
  /// entire diagram. Invalidates all entries in that subsystem's cache that
  /// depend on State. Aborts if @p subsystem is not actually a subsystem of
  /// this diagram.
  ///
  /// TODO(david-german-tri): Provide finer-grained accessors for finer-grained
  /// invalidation.
  State<T>* GetMutableSubsystemState(Context<T>* context,
                                     const System<T>* subsystem) const {
    Context<T>* subcontext = GetMutableSubsystemContext(context, subsystem);
    return subcontext->get_mutable_state();
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  State<T>* GetMutableSubsystemState(State<T>* state,
                                     const System<T>* subsystem) const {
    auto ret = DoGetMutableTargetSystemState(subsystem, state);
    DRAKE_DEMAND(ret != nullptr);
    return ret;
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  const State<T>& GetSubsystemState(
      const State<T>& state, const System<T>* subsystem) const {
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
    for (const auto& subsystem : sorted_systems_) {
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

  /// Returns the index of the given @p sys in the sorted order of this diagram,
  /// or aborts if @p sys is not a member of the diagram.
  int GetSystemIndexOrAbort(const System<T>* sys) const {
    auto it = sorted_systems_map_.find(sys);
    DRAKE_DEMAND(it != sorted_systems_map_.end());
    return it->second;
  }

 protected:
  /// Constructs an uninitialized Diagram. Subclasses that use this constructor
  /// are obligated to call DiagramBuilder::BuildInto(this).
  Diagram() {}

  /// Returns a pointer to mutable context if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  Context<T>* DoGetMutableTargetSystemContext(
      const System<T>* target_system, Context<T>* context) const final {
    if (target_system == this)
      return context;

    return GetSubsystemStuff<Context<T>*, DiagramContext<T>*>(
        target_system, context,
        &System<T>::DoGetMutableTargetSystemContext,
        &DiagramContext<T>::GetMutableSubsystemContext);
  }

  /// Returns a pointer to const context if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  const Context<T>* DoGetTargetSystemContext(
      const System<T>* target_system, const Context<T>* context) const final {
    if (target_system == this)
      return context;

    return GetSubsystemStuff<const Context<T>*, const DiagramContext<T>*>(
        target_system, context,
        &System<T>::DoGetTargetSystemContext,
        &DiagramContext<T>::GetSubsystemContext);
  }

  /// Returns a pointer to mutable state if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  State<T>* DoGetMutableTargetSystemState(
      const System<T>* target_system, State<T>* state) const final {
    if (target_system == this)
      return state;

    return GetSubsystemStuff<State<T>*, DiagramState<T>*>(
        target_system, state,
        &System<T>::DoGetMutableTargetSystemState,
        &DiagramState<T>::get_mutable_substate);
  }

  /// Returns a pointer to const state if @p target_system is a sub system
  /// of this, nullptr is returned otherwise.
  const State<T>* DoGetTargetSystemState(
      const System<T>* target_system, const State<T>* state) const final {
    if (target_system == this)
      return state;

    return GetSubsystemStuff<const State<T>*, const DiagramState<T>*>(
        target_system, state,
        &System<T>::DoGetTargetSystemState,
        &DiagramState<T>::get_substate);
  }

  void DoPublish(const Context<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    for (const System<T>* const system : sorted_systems_) {
      const int i = GetSystemIndexOrAbort(system);
      system->Publish(*diagram_context->GetSubsystemContext(i));
    }
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
    const ContinuousState<T>* xc = context.get_continuous_state();
    DRAKE_DEMAND(xc != nullptr);
    const int nq = xc->get_generalized_position().size();
    const int nv = xc->get_generalized_velocity().size();
    DRAKE_DEMAND(nq == qdot->size());
    DRAKE_DEMAND(nv == generalized_velocity.size());

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, asking each subsystem to
    // map its subslice of velocity to configuration derivatives. This approach
    // is valid because the DiagramContinuousState guarantees that the subsystem
    // states are concatenated in sorted order.
    int v_index = 0;  // The next index to read in generalized_velocity.
    int q_index = 0;  // The next index to write in qdot.
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const ContinuousState<T>* sub_xc = subcontext->get_continuous_state();
      // If subsystem i is stateless, skip it.
      if (sub_xc == nullptr) continue;

      // Select the chunk of generalized_velocity belonging to subsystem i.
      const int num_v = sub_xc->get_generalized_velocity().size();
      const Eigen::Ref<const VectorX<T>>& v_slice =
          generalized_velocity.segment(v_index, num_v);

      // Select the chunk of qdot belonging to subsystem i.
      const int num_q = sub_xc->get_generalized_position().size();
      Subvector<T> dq_slice(qdot, q_index, num_q);

      // Delegate the actual mapping to subsystem i itself.
      sorted_systems_[i]->MapVelocityToQDot(*subcontext, v_slice, &dq_slice);

      // Advance the indices.
      v_index += num_v;
      q_index += num_q;
    }
  }

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapQDotToVelocity(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      VectorBase<T>* generalized_velocity) const override {
    // Check that the dimensions of the continuous state in the context match
    // the dimensions of the provided generalized velocity and configuration
    // derivatives.
    const ContinuousState<T>* xc = context.get_continuous_state();
    DRAKE_DEMAND(xc != nullptr);
    const int nq = xc->get_generalized_position().size();
    const int nv = xc->get_generalized_velocity().size();
    DRAKE_DEMAND(nq == qdot.size());
    DRAKE_DEMAND(nv == generalized_velocity->size());

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, asking each subsystem to
    // map its subslice of configuration derivatives to velocity. This approach
    // is valid because the DiagramContinuousState guarantees that the subsystem
    // states are concatenated in sorted order.
    int q_index = 0;  // The next index to read in qdot.
    int v_index = 0;  // The next index to write in generalized_velocity.
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const ContinuousState<T>* sub_xc = subcontext->get_continuous_state();
      // If subsystem i is stateless, skip it.
      if (sub_xc == nullptr) continue;

      // Select the chunk of qdot belonging to subsystem i.
      const int num_q = sub_xc->get_generalized_position().size();
      const Eigen::Ref<const VectorX<T>>& dq_slice =
        qdot.segment(q_index, num_q);

      // Select the chunk of generalized_velocity belonging to subsystem i.
      const int num_v = sub_xc->get_generalized_velocity().size();
      Subvector<T> v_slice(generalized_velocity, v_index, num_v);

      // Delegate the actual mapping to subsystem i itself.
      sorted_systems_[i]->MapQDotToVelocity(*subcontext, dq_slice, &v_slice);

      // Advance the indices.
      v_index += num_v;
      q_index += num_q;
    }
  }

  /// Computes the next update time based on the configured actions, for scalar
  /// types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* actions) const override {
    DoCalcNextUpdateTimeImpl(context, actions);
  }

  /// Populates a vector of events that need to be handled before the integrator
  /// can take a step.
  void DoGetPerStepEvents(const Context<T>& context,
      std::vector<DiscreteEvent<T>>* events) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, and harvest all their per
    // step actions.
    std::vector<std::vector<DiscreteEvent<T>>> sub_events(num_subsystems());

    bool no_events = true;
    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      sorted_systems_[i]->GetPerStepEvents(*subcontext, &sub_events[i]);
      no_events = no_events && sub_events[i].empty();
    }

    // If no actions are needed, bail early.
    if (no_events) return;

    internal::SubsystemIdAndEventPairs<T> publishers;
    internal::SubsystemIdAndEventPairs<T> updaters;
    internal::SubsystemIdAndEventPairs<T> unrestricted_updaters;

    for (int i = 0; i < num_subsystems(); i++) {
      internal::FilterSubsystemEventsByType<T>(i, sub_events[i],
          &publishers, &updaters, &unrestricted_updaters);
    }

    DRAKE_ASSERT(!publishers.empty() || !updaters.empty() ||
                 !unrestricted_updaters.empty());

    // Request a publish event, if our subsystems want it.
    RequestPublishIfAny<T>(publishers, events);

    // Request an update event, if our subsystems want it.
    RequestDiscreteUpdateIfAny<T>(updaters, events);

    // Request an unrestricted update event, if our subsystems want it.
    RequestUnrestrictedUpdateIfAny<T>(unrestricted_updaters, events);
  }

  /// Creates a deep copy of this Diagram<double>, converting the scalar type
  /// to AutoDiffXd, and preserving all internal structure. Subclasses
  /// may wish to override to initialize additional member data, or to return a
  /// more specific covariant type.
  /// This is the NVI implementation of ToAutoDiffXd.
  Diagram<AutoDiffXd>* DoToAutoDiffXd() const override {
    using FromType = System<double>;
    using ToType = std::unique_ptr<System<AutoDiffXd>>;
    std::function<ToType(const FromType&)> subsystem_converter{
      [](const FromType& subsystem) {
        return subsystem.ToAutoDiffXd();
      }};
    return ConvertScalarType<AutoDiffXd>(subsystem_converter).release();
  }

  /// Creates a deep copy of this Diagram<double>, converting the scalar type
  /// to symbolic::Expression, and preserving all internal structure. Subclasses
  /// may wish to override to initialize additional member data, or to return a
  /// more specific covariant type.
  /// This is the NVI implementation of ToSymbolic.
  Diagram<symbolic::Expression>* DoToSymbolic() const override {
    using FromType = System<double>;
    using ToType = std::unique_ptr<System<symbolic::Expression>>;
    std::function<ToType(const FromType&)> subsystem_converter{
        [](const FromType& subsystem) {
          return subsystem.ToSymbolic();
        }};
    return ConvertScalarType<symbolic::Expression>(
        subsystem_converter).release();
  }

  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputVector(
        subsystem->get_input_port(subindex)).release();
  }

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputAbstract(
        subsystem->get_input_port(subindex)).release();
  }

 private:
  /// Tries to recursively find @p target_system's BaseStuffPtr
  /// (context / state / etc). nullptr is returned if @p target_system is not
  /// a sub system of this diagram. This template function should only be used
  /// to reduce code repetition for DoGetMutableTargetSystemContext(),
  /// DoGetTargetSystemContext(), DoGetMutableTargetSystemState(), and
  /// DoGetTargetSystemState().
  /// @param target_system The sub system of interest.
  /// @param my_stuff BaseStuffPtr that's associated with this diagram.
  /// @param recursive_getter A member function of System that returns sub
  /// context or state. Should be one of the four functions listed above.
  /// @param get_child_stuff A member function of DiagramContext or DiagramState
  /// that returns context or state given the index of the sub system.
  ///
  /// @tparam BaseStuffPtr Can be Context<T>*, const Context<T>*, State<T>* and
  /// const State<T>*.
  /// @tparam DerivedStuffPtr Can be DiagramContext<T>*,
  /// const DiagramContext<T>*, DiagramState<T>* and const DiagramState<T>*.
  ///
  /// @pre @p target_system cannot be `this`. The caller should check for this
  /// edge case.
  template <typename BaseStuffPtr, typename DerivedStuffPtr>
  BaseStuffPtr GetSubsystemStuff(
      const System<T>* target_system, BaseStuffPtr my_stuff,
      std::function<BaseStuffPtr(const System<T>*, const System<T>*,
                                 BaseStuffPtr)>
          recursive_getter,
      std::function<BaseStuffPtr(DerivedStuffPtr, int)> get_child_stuff) const {
    DerivedStuffPtr const my_stuff_as_derived =
        dynamic_cast<DerivedStuffPtr>(my_stuff);
    DRAKE_DEMAND(my_stuff_as_derived != nullptr);
    DRAKE_DEMAND(target_system != nullptr);
    DRAKE_DEMAND(target_system != this);

    int index = 0;
    for (const System<T>* child : sorted_systems_) {
      BaseStuffPtr const child_stuff =
          get_child_stuff(my_stuff_as_derived, index);
      BaseStuffPtr const target_stuff =
          recursive_getter(child, target_system, child_stuff);

      if (target_stuff != nullptr) {
        return target_stuff;
      }
      index++;
    }

    return nullptr;
  }

  /// Uses this Diagram<double> to manufacture a Diagram<NewType>, given a
  /// @p converter for subsystems from System<double> to System<NewType>.
  /// SFINAE overload for std::is_same<T, double>.
  ///
  /// @tparam NewType The scalar type to which to convert.
  /// @tparam T1 SFINAE boilerplate.
  template <typename NewType, typename T1 = T>
  std::unique_ptr<Diagram<NewType>> ConvertScalarType(
      std::function<std::unique_ptr<System<NewType>>(
          const System<
              std::enable_if_t<std::is_same<T1, double>::value, double>>&)>
          converter) const {
    std::vector<std::unique_ptr<System<NewType>>> new_systems;
    // Recursively convert all the subsystems.
    std::map<const System<T1>*, const System<NewType>*> old_to_new_map;
    for (const auto& old_system : registered_systems_) {
      new_systems.push_back(converter(*old_system));
      old_to_new_map[old_system.get()] = new_systems.back().get();
    }

    // Set up the blueprint.
    typename Diagram<NewType>::Blueprint blueprint;
    // Make all the inputs and outputs.
    for (const PortIdentifier& id : input_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint.input_port_ids.emplace_back(new_system, port);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint.output_port_ids.emplace_back(new_system, port);
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

      blueprint.dependency_graph[new_dest] = new_src;
    }
    // Preserve the sort order.
    for (const System<T1>* system : sorted_systems_) {
      blueprint.sorted_systems.push_back(old_to_new_map[system]);
    }

    // Construct a new Diagram of type NewType from the blueprint.
    std::unique_ptr<Diagram<NewType>> new_diagram(
        new Diagram<NewType>(blueprint));
    new_diagram->Own(std::move(new_systems));
    return std::move(new_diagram);
  }

  /// Aborts at runtime.
  /// SFINAE overload for !std::is_same<T, double>.
  ///
  /// @tparam NewType The scalar type to which to convert.
  /// @tparam T1 SFINAE boilerplate.
  template <typename NewType, typename T1 = T>
  std::unique_ptr<Diagram<NewType>> ConvertScalarType(
      std::function<std::unique_ptr<System<NewType>>(
          const System<
              std::enable_if_t<!std::is_same<T1, double>::value,
                               double>>&)>) const {
    DRAKE_ABORT_MSG(
        "Scalar type conversion is only supported from Diagram<double>.");
  }

  // Adds a Diagram<T1>::HandlePublish callback to handle diagram level
  // publish in @p my_events if @p sub_events is not empty.
  template <typename T1 = T>
  void RequestPublishIfAny(
      const internal::SubsystemIdAndEventPairs<T1>& sub_events,
      std::vector<DiscreteEvent<T1>>* my_events) const {
    if (!sub_events.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kPublishAction;
      event.do_publish = std::bind(&Diagram<T1>::HandlePublish, this,
                                   std::placeholders::_1, /* context */
                                   sub_events);
      DRAKE_ASSERT(my_events != nullptr);
      my_events->push_back(event);
    }
  }

  // Adds a Diagram<T1>::HandleUnrestrictedUpdate callback to handle
  // diagram level unrestricted update to @p my_events if @p sub_events is
  // not empty.
  template <typename T1 = T>
  void RequestUnrestrictedUpdateIfAny(
      const internal::SubsystemIdAndEventPairs<T1>& sub_events,
      std::vector<DiscreteEvent<T1>>* my_events) const {
    if (!sub_events.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kUnrestrictedUpdateAction;
      event.do_unrestricted_update = std::bind(
                                  &Diagram<T1>::HandleUnrestrictedUpdate,
                                  this,
                                  std::placeholders::_1, /* context */
                                  std::placeholders::_2, /* state */
                                  sub_events);
      DRAKE_ASSERT(my_events != nullptr);
      my_events->push_back(event);
    }
  }

  // Adds a Diagram<T1>::HandleUpdate callback to handle diagram level
  // discrete state update to @p my_events if @p sub_events is not empty.
  template <typename T1 = T>
  void RequestDiscreteUpdateIfAny(
      const internal::SubsystemIdAndEventPairs<T1>& sub_events,
      std::vector<DiscreteEvent<T1>>* my_events) const {
    if (!sub_events.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kDiscreteUpdateAction;
      event.do_calc_discrete_variable_update = std::bind(
                                  &Diagram<T1>::HandleUpdate,
                                  this,
                                  std::placeholders::_1, /* context */
                                  std::placeholders::_2, /* difference state */
                                  sub_events);
      DRAKE_ASSERT(my_events != nullptr);
      my_events->push_back(event);
    }
  }

  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>&, UpdateActions<T1>*) const {
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
      const Context<T1>& context, UpdateActions<T1>* actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T1>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    actions->time = std::numeric_limits<T1>::infinity();

    // Iterate over the subsystems in sorted order, and harvest the most
    // imminent updates.
    std::vector<UpdateActions<T1>> sub_actions(num_subsystems());
    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T1>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const T1 time =
          sorted_systems_[i]->CalcNextUpdateTime(*subcontext, &sub_actions[i]);
      if (time < actions->time) {
        actions->time = time;
      }
    }

    // If no discrete actions are needed, bail early.
    if (actions->time == std::numeric_limits<T1>::infinity()) {
      return;
    }

    internal::SubsystemIdAndEventPairs<T1> publishers;
    internal::SubsystemIdAndEventPairs<T1> updaters;
    internal::SubsystemIdAndEventPairs<T1> unrestricted_updaters;

    for (int i = 0; i < num_subsystems(); i++) {
      if (sub_actions[i].time > actions->time) continue;

      internal::FilterSubsystemEventsByType(i, sub_actions[i].events,
          &publishers, &updaters, &unrestricted_updaters);
    }

    DRAKE_ASSERT(!publishers.empty() || !updaters.empty() ||
                 !unrestricted_updaters.empty());

    // Request a publish event, if our subsystems want it.
    RequestPublishIfAny<T1>(publishers, &(actions->events));

    // Request an update event, if our subsystems want it.
    RequestDiscreteUpdateIfAny<T1>(updaters, &(actions->events));

    // Request an unrestricted update event, if our subsystems want it.
    RequestUnrestrictedUpdateIfAny<T1>(
        unrestricted_updaters, &(actions->events));
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
    // A list of the systems in the dependency graph in a valid, sorted
    // execution order, such that if EvalOutput is called on each system in
    // succession, every system will have valid inputs by the time its turn
    // comes.
    std::vector<const System<T>*> sorted_systems;
  };

  // Constructs a Diagram from the Blueprint that a DiagramBuilder produces.
  // This constructor is private because only DiagramBuilder calls it.
  explicit Diagram(const Blueprint& blueprint) { Initialize(blueprint); }

  // Validates the given @p blueprint and sets up the Diagram accordingly.
  void Initialize(const Blueprint& blueprint) {
    // The Diagram must not already be initialized.
    DRAKE_DEMAND(sorted_systems_.empty());
    // The initialization must be nontrivial.
    DRAKE_DEMAND(!blueprint.sorted_systems.empty());

    // Copy the data from the blueprint into private member variables.
    dependency_graph_ = blueprint.dependency_graph;
    sorted_systems_ = blueprint.sorted_systems;
    input_port_ids_ = blueprint.input_port_ids;
    output_port_ids_ = blueprint.output_port_ids;

    // Generate a map from the System pointer to its index in the sort order.
    for (int i = 0; i < num_subsystems(); ++i) {
      sorted_systems_map_[sorted_systems_[i]] = i;
    }

    // Every system must appear in the sort order exactly once.
    DRAKE_DEMAND(sorted_systems_.size() == sorted_systems_map_.size());
    // Every port named in the dependency_graph_ must actually exist.
    DRAKE_ASSERT(PortsAreValid());
    // The sort order must square with the dependency_graph_.
    DRAKE_ASSERT(SortOrderIsCorrect());
    // Every subsystem must have a unique name.
    DRAKE_THROW_UNLESS(NamesAreUniqueAndNonEmpty());

    // Add the inputs to the Diagram topology, and check their invariants.
    for (const PortIdentifier& id : input_port_ids_) {
      ExportInput(id);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      ExportOutput(id);
    }
  }

  // Takes ownership of the @p registered_systems from DiagramBuilder.
  void Own(std::vector<std::unique_ptr<System<T>>> registered_systems) {
    // We must be given something to own.
    DRAKE_DEMAND(!registered_systems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(registered_systems_.empty());
    // The subsystems we are being given to own must be exactly the set of
    // subsystems for which we have an execution order.
    DRAKE_DEMAND(registered_systems.size() == sorted_systems_.size());
    for (const auto& system : registered_systems) {
      const auto it = sorted_systems_map_.find(system.get());
      DRAKE_DEMAND(it != sorted_systems_map_.end());
    }
    // All of those checks having passed, take ownership of the subsystems.
    registered_systems_ = std::move(registered_systems);
    // Inform the constituent system it's bound to this Diagram.
    for (auto& system : registered_systems_) {
      system->set_parent(this);
    }
  }

  // Exposes the given port as an input of the Diagram.
  void ExportInput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the sort order.
    GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = sys->get_input_port(port_index);
    this->DeclareInputPort(subsystem_descriptor.get_data_type(),
                           subsystem_descriptor.size());
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
    const Context<T>* subsystem_context = context.GetSubsystemContext(i);
    SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(i);
    AbstractValue* port_output = subsystem_output->GetMutableData(port_index);
    port.Calc(*subsystem_context, port_output);
  }

  // Converts a PortIdentifier to a DiagramContext::PortIdentifier.
  // The DiagramContext::PortIdentifier contains the index of the System in the
  // sorted order of the diagram, instead of an actual pointer to the System.
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

  // Returns true if every System precedes all its dependents in
  // sorted_systems_.
  bool SortOrderIsCorrect() const {
    for (const auto& entry : dependency_graph_) {
      const System<T>* const dest = entry.first.first;
      const System<T>* const src = entry.second.first;
      // If the destination system has no direct feedthrough, it does not
      // matter whether it is sorted before or after the systems on which
      // it depends.
      if (!dest->HasAnyDirectFeedthrough()) {
        continue;
      }
      if (GetSystemIndexOrAbort(dest) <= GetSystemIndexOrAbort(src)) {
        return false;
      }
    }
    return true;
  }

  // Returns true if every subsystem has a unique, non-empty name.
  // O(N * log(N)) in the number of subsystems.
  bool NamesAreUniqueAndNonEmpty() const {
    std::set<std::string> names;
    for (const auto& system : sorted_systems_) {
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
        log()->error("Non-unique name \"{}\" for subsystem of type {}",
                     name, NiceTypeName::Get(*system));
      }
      names.insert(name);
    }
    return names.size() == sorted_systems_.size();
  }

  /// Handles Publish callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandlePublish(
      const Context<T>& context,
      const internal::SubsystemIdAndEventPairs<T>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    for (const auto& action : sub_actions) {
      const int index = action.first;
      const DiscreteEvent<T>& event = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);

      DRAKE_ASSERT(event.action == DiscreteEvent<T>::kPublishAction);
      sorted_systems_[index]->Publish(*subcontext, event);
    }
  }

  /// Handles Update callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandleUpdate(
      const Context<T>& context, DiscreteValues<T>* update,
      const internal::SubsystemIdAndEventPairs<T>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_differences =
        dynamic_cast<internal::DiagramDiscreteVariables<T>*>(update);
    DRAKE_DEMAND(diagram_differences != nullptr);

    // As a baseline, initialize all the difference variables to their
    // current values.
    for (int i = 0; i < diagram_differences->num_groups(); ++i) {
      diagram_differences->get_mutable_vector(i)->set_value(
          context.get_discrete_state(i)->get_value());
    }

    // Then, allow the systems that wanted to update a difference variable
    // to do so.
    for (const auto& action : sub_actions) {
      const int index = action.first;
      const DiscreteEvent<T>& event = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      // Get the context and the difference state for the specified system.
      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);
      DiscreteValues<T>* subdifference =
          diagram_differences->get_mutable_subdifference(index);
      DRAKE_DEMAND(subdifference != nullptr);

      // Do that system's update actions.
      DRAKE_ASSERT(event.action == DiscreteEvent<T>::kDiscreteUpdateAction);
      sorted_systems_[index]->CalcDiscreteVariableUpdates(*subcontext,
                                                          event,
                                                          subdifference);
    }
  }

  /// Handles Update callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the UnrestrictedUpdate events to the subsystems that requested
  /// them.
  void HandleUnrestrictedUpdate(
      const Context<T>& context, State<T>* state,
      const internal::SubsystemIdAndEventPairs<T>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // No need to set state to context's state, since it has already been done
    // in System::CalcUnrestrictedUpdate().

    for (const auto& action : sub_actions) {
      const int index = action.first;
      const DiscreteEvent<T>& event = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      // Get the context and the state for the specified system.
      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);
      State<T>* substate = diagram_state->get_mutable_substate(index);
      DRAKE_DEMAND(substate != nullptr);

      // Do that system's update actions.
      DRAKE_ASSERT(event.action == DiscreteEvent<T>::kUnrestrictedUpdateAction);
      sorted_systems_[index]->CalcUnrestrictedUpdate(*subcontext,
                                                     event,
                                                     substate);
    }
  }

  int num_subsystems() const {
    return static_cast<int>(sorted_systems_.size());
  }

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The topologically sorted list of Systems in this Diagram.
  std::vector<const System<T>*> sorted_systems_;

  // The Systems in this Diagram, which are owned by this Diagram, in the order
  // they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // For fast conversion queries: what is the index of this System in the
  // sorted order?
  std::map<const System<T>*, int> sorted_systems_map_;

  // The ordered inputs and outputs of this Diagram.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For all T, Diagram<T> considers DiagramBuilder<T> a friend, so that the
  // builder can set the internal state correctly.
  friend class DiagramBuilder<T>;

  // For all T, Diagram<T> considers Diagram<double> a friend, so that
  // Diagram<double> can provide transmogrification methods to more flavorful
  // scalar types.  See Diagram<T>::ConvertScalarType.
  friend class Diagram<double>;
};

}  // namespace systems
}  // namespace drake
