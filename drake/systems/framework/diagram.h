#pragma once

#include <map>
#include <set>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

/// DiagramOutput is an implementation of SystemOutputInterface that holds
/// unowned OutputPort pointers. It is used to expose the outputs of
/// constituent systems as outputs of a Diagram.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
struct DiagramOutput : public SystemOutputInterface<T> {
  int get_num_ports() const override { return ports_.size(); }

  OutputPort<T>* get_mutable_port(int index) override { return ports_[index]; }

  const OutputPort<T>& get_port(int index) const override {
    return *ports_[index];
  }

  std::vector<OutputPort<T>*>* get_mutable_ports() { return &ports_; }

 private:
  std::vector<OutputPort<T>*> ports_;
};

/// Diagram is a System composed of one or more constituent Systems, arranged
/// in a directed graph where the vertices are the constituent Systems
/// themselves, and the edges connect the output of one constituent System
/// to the input of another.
///
/// There are two phases in the lifecycle of a Diagram: the configuration phase
/// (before finalization) and the execution phase (after finalization). The
/// Diagram transitions irrevocably from configuration to execution when
/// `Finalize` is called.
///
/// During configuration, Systems and connections can be added to the Diagram,
/// but the context and output structures for the Diagram cannot be allocated,
/// and thus the outputs and state derivatives cannot be computed.
///
/// During execution, Systems and connections may no longer be added to the
/// Diagram, but computations can be performed on it as with any other
/// system.
template <typename T>
class Diagram : public ContinuousSystemInterface<T> {
 public:
  Diagram(const std::string& name) : name_(name) {}
  virtual ~Diagram() {}

  std::string get_name() const override { return name_; }

  void Connect(const SystemInterface<T>* src, int src_port_index,
               const SystemInterface<T>* dest, int dest_port_index) {
    ThrowIfFinal();
    Register(src);
    Register(dest);
    PortIdentifier<T> dest_id{dest, dest_port_index};
    PortIdentifier<T> src_id{src, src_port_index};
    ThrowIfInputAlreadyWired(dest_id);
    dependency_graph_[dest_id] = src_id;
  }

  void AddInput(const SystemInterface<T>* sys, int port_index) {
    ThrowIfFinal();
    Register(sys);
    PortIdentifier<T> id{sys, port_index};
    ThrowIfInputAlreadyWired(id);
    input_port_ids_.push_back(id);
    diagram_input_set_.insert(id);
    systems_.insert(sys);
  }

  void AddOutput(const SystemInterface<T>* sys, int port_index) {
    ThrowIfFinal();
    Register(sys);
    output_port_ids_.push_back(PortIdentifier<T>{sys, port_index});
  }

  void Finalize() {
    ThrowIfFinal();
    sorted_systems_ = SortSystems();
  }

  std::unique_ptr<AbstractContext<T>> CreateDefaultContext() const override {
    ThrowIfNotFinal();

    // Reserve inputs as specified during Diagram initialization.
    std::unique_ptr<DiagramContext<T>> context(
        new DiagramContext<T>(input_port_ids_));

    // Add each constituent system to the Context, and wire up the
    // inputs to outputs.
    for (const auto& system : sorted_systems_) {
      context->AddConstituentSystem(system);
    }

    for (const auto& connection : dependency_graph_) {
      const PortIdentifier<T>& src = connection.second;
      const PortIdentifier<T>& dest = connection.first;
      context->Connect(src, dest);
    }

    // TODO: Create a state supervector over the constituent systems.
    return std::unique_ptr<AbstractContext<T>>(context.release());
  }

  std::unique_ptr<SystemOutputInterface<T>> AllocateOutput(
      const AbstractContext<T>& context) const override {
    ThrowIfNotFinal();
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    if (diagram_context == nullptr) {
      throw std::runtime_error(
          "Diagram::AllocateOutput was not given its own context.");
    }

    // Some of the output ports of this Diagram's constituent systems are also
    // output ports of this Diagram. Wrap those ports in a DiagramOutput.
    std::unique_ptr<DiagramOutput<T>> output(new DiagramOutput<T>);
    std::vector<OutputPort<T>*>* ports = output->get_mutable_ports();
    for (const PortIdentifier<T>& id : output_port_ids_) {
      ports->emplace_back(diagram_context->GetSubsystemOutput(id.first)
                              ->get_mutable_port(id.second));
    }

    return std::unique_ptr<SystemOutputInterface<T>>(output.release());
  }

  void EvalOutput(const AbstractContext<T>& context,
                  SystemOutputInterface<T>* output) const override {
    ThrowIfNotFinal();

    // Down-cast the context and output to DiagramContext and DiagramOutput.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_ASSERT(diagram_context != nullptr);
    auto diagram_output = dynamic_cast<DiagramOutput<T>*>(output);
    DRAKE_ASSERT(diagram_output != nullptr);

    // Populate the output with pointers to the appropriate subsystem outputs
    // in the DiagramContext. We do this on every call to EvalOutput, even
    // though it was also done in AllocateOutput, just in case the caller gave
    // us an output that was allocated with reference to some context other
    // than the one provided here.
    DRAKE_ASSERT(diagram_output->get_num_ports() == output_port_ids_.size());
    for (size_t i = 0; i < output_port_ids_.size(); ++i) {
      const PortIdentifier<T>& id = output_port_ids_[i];
      (*diagram_output->get_mutable_ports())[i] =
          diagram_context->GetSubsystemOutput(id.first)
              ->get_mutable_port(id.second);
    }

    // Since the diagram output simply contains pointers to the subsystem
    // outputs, all we need to do is compute all the subsystem outputs in
    // sorted order.
    for (const SystemInterface<T>* system : sorted_systems_) {
      const AbstractContext<T>* subsystem_context =
          diagram_context->GetSubsystemContext(system);
      SystemOutputInterface<T>* subsystem_output =
          diagram_context->GetSubsystemOutput(system);
      system->EvalOutput(*subsystem_context, subsystem_output);
    }
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    ThrowIfNotFinal();
    // TODO(david-german-tri): Actually allocate derivatives.
    return nullptr;
  }

  void EvalTimeDerivatives(const AbstractContext<T>& context,
                           ContinuousState<T>* derivatives) const override {
    ThrowIfNotFinal();
    // TODO(david-german-tri): Actually compute derivatives.
  }

  void MapVelocityToConfigurationDerivatives(
      const AbstractContext<T>& context,
      const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const override {
    ThrowIfNotFinal();
    // TODO(david-german-tri): Actually map velocity to derivatives.
  }

 private:
  void Register(const SystemInterface<T>* sys) {
    if (systems_.find(sys) != systems_.end()) {
      // This system is already registered.
      return;
    }
    systems_.insert(sys);
  };

  void ThrowIfFinal() const {
    if (!sorted_systems_.empty()) {
      throw std::runtime_error("Diagram is already finalized.");
    }
  }

  void ThrowIfNotFinal() const {
    if (sorted_systems_.empty()) {
      throw std::runtime_error("Diagram is not finalized.");
    }
  }

  void ThrowIfInputAlreadyWired(const PortIdentifier<T>& id) {
    if (dependency_graph_.find(id) != dependency_graph_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::runtime_error("Input port is already wired.");
    }
  }

  // Runs Kahn's algorithm to compute the topological sort order of the
  // Systems in the graph. If EvalOutput is called on each System in
  // the order that is returned, each System's inputs will be valid by
  // the time its EvalOutput is called.
  //
  // TODO(david-german-tri, bradking): Consider using functional form to
  // produce a separate execution order for each output of the Diagram.
  std::vector<const SystemInterface<T>*> SortSystems() const {
    std::vector<const SystemInterface<T>*> output;

    // Build two maps:
    // A map from each system, to every system that depends on it.
    std::map<const SystemInterface<T>*, std::set<const SystemInterface<T>*>>
        dependents;
    // A map from each system, to every system on which it depends.
    std::map<const SystemInterface<T>*, std::set<const SystemInterface<T>*>>
        dependencies;

    for (const auto& connection : dependency_graph_) {
      const SystemInterface<T>* src = connection.second.first;
      const SystemInterface<T>* dest = connection.first.first;
      dependents[src].insert(dest);
      dependencies[dest].insert(src);
    }

    // Find the systems that have no inputs within the Diagram.
    std::set<const SystemInterface<T>*> nodes_with_in_degree_zero;
    for (const SystemInterface<T>* system : systems_) {
      if (dependencies.find(system) == dependencies.end()) {
        nodes_with_in_degree_zero.insert(system);
      }
    }

    while (!nodes_with_in_degree_zero.empty()) {
      // Pop a node with in-degree zero.
      auto it = nodes_with_in_degree_zero.begin();
      const SystemInterface<T>* node = *it;
      nodes_with_in_degree_zero.erase(it);

      // Push the node onto the sorted output.
      output.push_back(node);

      for (const SystemInterface<T>* dependent : dependents[node]) {
        dependencies[dependent].erase(node);
        if (dependencies[dependent].empty()) {
          nodes_with_in_degree_zero.insert(dependent);
        }
      }
    }

    if (!nodes_with_in_degree_zero.empty()) {
      // TODO(david-german-tri): Attempt to break cycles using
      // the direct-feedthrough configuration of a System.
      throw new std::runtime_error("Cycle detected in Diagram.");
    }
    return output;
  }

  // Diagram objects are neither copyable nor moveable.
  Diagram(const Diagram<T>& other) = delete;
  Diagram& operator=(const Diagram<T>& other) = delete;
  Diagram(Diagram<T>&& other) = delete;
  Diagram& operator=(Diagram<T>&& other) = delete;

  std::string name_;

  // The ordered inputs and outputs of this Diagram.
  std::vector<PortIdentifier<T>> input_port_ids_;
  std::vector<PortIdentifier<T>> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier<T>> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier<T>, PortIdentifier<T>> dependency_graph_;

  // The unsorted set of Systems in this Diagram.
  std::set<const SystemInterface<T>*> systems_;

  // The topologically sorted list of Systems in this Diagram. Only computed
  // when Finalize is called.
  std::vector<const SystemInterface<T>*> sorted_systems_;
};

}  // namespace systems
}  // namespace drake
