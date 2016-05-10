#pragma once

#include <map>
#include <set>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

template <typename T> class Diagram
    : public ContinuousSystemInterface<T> {
 public:
  Diagram(const std::string& name) : name_(name) {}
  virtual ~Diagram() {}

  std::string get_name() const override { return name_; }

  void Connect(const SystemInterface<T>* src,
               int src_port_index,
               const SystemInterface<T>* dest,
               int dest_port_index) {
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
    diagram_inputs_.push_back(id);
    diagram_input_set_.insert(id);
    systems_.insert(sys);
  }

  void AddOutput(const SystemInterface<T>* sys, int port_index) {
    ThrowIfFinal();
    Register(sys);
    diagram_outputs_.push_back(PortIdentifier<T>{sys, port_index});
  }

  void Finalize() {
    ThrowIfFinal();
    sorted_systems_ = SortSystems();
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    ThrowIfNotFinal();
    std::unique_ptr<DiagramContext<T>> context(new DiagramContext<T>);

    // Reserve inputs as specified during Diagram initialization.
    context->SetNumInputPorts(diagram_inputs_.size());

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
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput() const override {
    ThrowIfNotFinal();
    std::unique_ptr<SystemOutput<T>> output(new SystemOutput<T>);
    // TODO: something.
    return output;
  }

  void EvalOutput(const Context<T>& context, SystemOutput<T>* output) const override {
    ThrowIfNotFinal();
    // TODO: something
  }


  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    ThrowIfNotFinal();
    // TODO: something
    return nullptr;
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    ThrowIfNotFinal();
    // TODO: something
  }

  void MapVelocityToConfigurationDerivatives(
        const Context<T>& context, const StateVector<T>& generalized_velocity,
        StateVector<T>* configuration_derivatives) const override {
    ThrowIfNotFinal();
    // TODO: something
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

  void ThrowIfOutputsInvalid() {
    for (const PortIdentifier<T>& output_port : diagram_outputs_) {
      // TODO(david-german-tri): Actually validate something.
    }
  }

  std::vector<const SystemInterface<T>*> SortSystems() const {
    // Kahn's Algorithm.
    std::vector<const SystemInterface<T>*> output;

    // Build two maps:
    // A map from each system, to every system that depends on it.
    std::map<const SystemInterface<T>*, std::set<const SystemInterface<T>*>> dependents;
    // A map from each system, to every system on which it depends.
    std::map<const SystemInterface<T>*, std::set<const SystemInterface<T>*>> dependencies;

    for (const auto& edge : dependency_graph_) {
      dependents[edge.first.first].insert(edge.second.first);
      dependencies[edge.second.first].insert(edge.first.first);
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
      throw new std::runtime_error("Cycle detected.");
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
  std::vector<PortIdentifier<T>> diagram_inputs_;
  std::vector<PortIdentifier<T>> diagram_outputs_;

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
