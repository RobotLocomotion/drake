#pragma once

#include <map>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram. It collects the dependency
/// graph of constituent systems, and topologically sorts them.
template <typename T>
class DiagramBuilder {
 public:
  DiagramBuilder() {}
  virtual ~DiagramBuilder() {}

  /// Declares that input port @p dest is connected to output port @p src.
  void Connect(const SystemPortDescriptor<T>& src,
               const SystemPortDescriptor<T>& dest) {
    Register(src.get_system());
    Register(dest.get_system());
    PortIdentifier dest_id{dest.get_system(), dest.get_index()};
    PortIdentifier src_id{src.get_system(), src.get_index()};
    ThrowIfInputAlreadyWired(dest_id);
    dependency_graph_[dest_id] = src_id;
  }

  /// Declares that the given @p input port of a constituent system is an input
  /// to the entire Diagram.
  void ExportInput(const SystemPortDescriptor<T>& input) {
    DRAKE_ASSERT(input.get_face() == kInputPort);
    Register(input.get_system());
    PortIdentifier id{input.get_system(), input.get_index()};
    ThrowIfInputAlreadyWired(id);
    input_port_ids_.push_back(id);
    diagram_input_set_.insert(id);
    systems_.insert(input.get_system());
  }

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.
  void ExportOutput(const SystemPortDescriptor<T>& output) {
    DRAKE_ASSERT(output.get_face() == kOutputPort);
    Register(output.get_system());
    output_port_ids_.push_back(
        PortIdentifier{output.get_system(), output.get_index()});
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput. Throws std::logic_error if the graph is
  /// not buildable.
  std::unique_ptr<Diagram<T>> Build() {
    if (systems_.size() == 0) {
      throw std::logic_error("Cannot Build with an empty DiagramBuilder.");
    }
    return std::make_unique<Diagram<T>>(dependency_graph_, SortSystems(),
                                        input_port_ids_, output_port_ids_);
  }

 private:
  typedef typename Diagram<T>::PortIdentifier PortIdentifier;

  void Register(const System<T>* sys) {
    if (systems_.find(sys) != systems_.end()) {
      // This system is already registered.
      return;
    }
    systems_.insert(sys);
  }

  void ThrowIfInputAlreadyWired(const PortIdentifier& id) const {
    if (dependency_graph_.find(id) != dependency_graph_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::logic_error("Input port is already wired.");
    }
  }

  // Runs Kahn's algorithm to compute the topological sort order of the
  // Systems in the graph. If EvalOutput is called on each System in
  // the order that is returned, each System's inputs will be valid by
  // the time its EvalOutput is called.
  //
  // TODO(david-german-tri, bradking): Consider using functional form to
  // produce a separate execution order for each output of the DiagramBuilder.
  std::vector<const System<T>*> SortSystems() const {
    std::vector<const System<T>*> output;

    // Build two maps:
    // A map from each system, to every system that depends on it.
    std::map<const System<T>*, std::set<const System<T>*>> dependents;
    // A map from each system, to every system on which it depends.
    std::map<const System<T>*, std::set<const System<T>*>> dependencies;

    for (const auto& connection : dependency_graph_) {
      const System<T>* src = connection.second.first;
      const System<T>* dest = connection.first.first;
      dependents[src].insert(dest);
      dependencies[dest].insert(src);
    }

    // Find the systems that have no inputs within the DiagramBuilder.
    std::set<const System<T>*> nodes_with_in_degree_zero;
    for (const System<T>* system : systems_) {
      if (dependencies.find(system) == dependencies.end()) {
        nodes_with_in_degree_zero.insert(system);
      }
    }

    while (!nodes_with_in_degree_zero.empty()) {
      // Pop a node with in-degree zero.
      auto it = nodes_with_in_degree_zero.begin();
      const System<T>* node = *it;
      nodes_with_in_degree_zero.erase(it);

      // Push the node onto the sorted output.
      output.push_back(node);

      for (const System<T>* dependent : dependents[node]) {
        dependencies[dependent].erase(node);
        if (dependencies[dependent].empty()) {
          nodes_with_in_degree_zero.insert(dependent);
        }
      }
    }

    if (output.size() != systems_.size()) {
      // TODO(david-german-tri): Attempt to break cycles using
      // the direct-feedthrough configuration of a System.
      throw std::logic_error("Cycle detected in DiagramBuilder.");
    }
    return output;
  }

  // DiagramBuilder objects are neither copyable nor moveable.
  DiagramBuilder(const DiagramBuilder<T>& other) = delete;
  DiagramBuilder& operator=(const DiagramBuilder<T>& other) = delete;
  DiagramBuilder(DiagramBuilder<T>&& other) = delete;
  DiagramBuilder& operator=(DiagramBuilder<T>&& other) = delete;

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The unsorted set of Systems in this DiagramBuilder.
  std::set<const System<T>*> systems_;
};

}  // namespace systems
}  // namespace drake
