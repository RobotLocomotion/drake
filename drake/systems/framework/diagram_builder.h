#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram. It is single
/// use: after calling Build or BuildInto, DiagramBuilder gives up ownership
/// of the constituent systems, and should therefore be discarded.
///
/// A system must be added to the DiagramBuilder with AddSystem before it can
/// be wired up in any way. Every system must have a unique, non-empty name.
template <typename T>
class DiagramBuilder {
 public:
  // DiagramBuilder objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilder)

  DiagramBuilder() {}
  virtual ~DiagramBuilder() {}

  /// Takes ownership of @p system and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// If the system's name is unset, sets it to System::GetMemoryObjectName()
  /// as a default in order to have unique names within the diagram.
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  S* AddSystem(std::unique_ptr<S> system) {
    if (system->get_name().empty()) {
      system->set_name(system->GetMemoryObjectName());
    }
    S* raw_sys_ptr = system.get();
    systems_.insert(raw_sys_ptr);
    registered_systems_.push_back(std::move(system));
    return raw_sys_ptr;
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   auto foo = builder.AddSystem<Foo<double>>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template<class S, typename... Args>
  S* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   // Foo must be a template.
  ///   auto foo = builder.AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template<template<typename Scalar> class S, typename... Args>
  S<T>* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  /// Returns whether any Systems have been added yet.
  bool empty() const { return registered_systems_.empty(); }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Declares that input port @p dest is connected to output port @p src.
  void Connect(const OutputPort<T>& src,
               const InputPortDescriptor<T>& dest) {
    DRAKE_DEMAND(src.size() == dest.size());
    PortIdentifier dest_id{dest.get_system(), dest.get_index()};
    PortIdentifier src_id{&src.get_system(), src.get_index()};
    ThrowIfInputAlreadyWired(dest_id);
    ThrowIfSystemNotRegistered(&src.get_system());
    ThrowIfSystemNotRegistered(dest.get_system());
    dependency_graph_[dest_id] = src_id;
  }

  /// Declares that sole input port on the @p dest system is connected to sole
  /// output port on the @p src system.  Throws an exception if the sole-port
  /// precondition is not met (i.e., if @p dest has no input ports, or @p dest
  /// has more than one input port, or @p src has no output ports, or @p src
  /// has more than one output port).
  void Connect(const System<T>& src, const System<T>& dest) {
    DRAKE_THROW_UNLESS(src.get_num_output_ports() == 1);
    DRAKE_THROW_UNLESS(dest.get_num_input_ports() == 1);
    Connect(src.get_output_port(0), dest.get_input_port(0));
  }

  /// Cascades @p src and @p dest.  The sole input port on the @p dest system
  /// is connected to sole output port on the @p src system.  Throws an
  /// exception if the sole-port precondition is not met (i.e., if @p dest has
  /// no input ports, or @p dest has more than one input port, or @p src has no
  /// output ports, or @p src has more than one output port).
  void Cascade(const System<T>& src, const System<T>& dest) {
    Connect(src, dest);
  }

  /// Declares that the given @p input port of a constituent system is an input
  /// to the entire Diagram.
  /// @return The index of the exported input port of the entire diagram.
  int ExportInput(const InputPortDescriptor<T>& input) {
    PortIdentifier id{input.get_system(), input.get_index()};
    ThrowIfInputAlreadyWired(id);
    ThrowIfSystemNotRegistered(input.get_system());
    int return_id = static_cast<int>(input_port_ids_.size());
    input_port_ids_.push_back(id);
    diagram_input_set_.insert(id);
    return return_id;
  }

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.
  /// @return The index of the exported output port of the entire diagram.
  int ExportOutput(const OutputPort<T>& output) {
    ThrowIfSystemNotRegistered(&output.get_system());
    int return_id = static_cast<int>(output_port_ids_.size());
    output_port_ids_.push_back(
        PortIdentifier{&output.get_system(), output.get_index()});
    return return_id;
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput. Throws std::logic_error if the graph is
  /// not buildable.
  std::unique_ptr<Diagram<T>> Build() {
    std::unique_ptr<Diagram<T>> diagram(new Diagram<T>(Compile()));
    return std::move(diagram);
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(Diagram<T>* target) {
    target->Initialize(Compile());
  }

 private:
  typedef typename Diagram<T>::PortIdentifier PortIdentifier;

  void ThrowIfInputAlreadyWired(const PortIdentifier& id) const {
    if (dependency_graph_.find(id) != dependency_graph_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::logic_error("Input port is already wired.");
    }
  }

  void ThrowIfSystemNotRegistered(const System<T>* system) const {
    DRAKE_THROW_UNLESS(systems_.find(system) != systems_.end());
  }

  // Helper method to do the algebraic loop test. It recursively performs the
  // depth-first search on the graph to find cycles. The "stack" is really just
  // a set because we need to do relatively "cheap" lookups for membership in
  // the stack. The stack-like property of the set is maintained by this method.
  static bool HasCycleRecurse(
      const PortIdentifier& n,
      const std::map<PortIdentifier, std::set<PortIdentifier>>& edges,
      std::set<PortIdentifier>* visited, std::set<PortIdentifier>* stack) {
    DRAKE_ASSERT(visited->count(n) == 0);
    visited->insert(n);

    auto edge_iter = edges.find(n);
    if (edge_iter != edges.end()) {
      DRAKE_ASSERT(stack->count(n) == 0);
      stack->insert(n);  // Push onto the stack.
      for (const auto& target : edge_iter->second) {
        if (visited->count(target) == 0 &&
            HasCycleRecurse(target, edges, visited, stack)) {
          return true;
        } else if (stack->count(target) > 0) {
          return true;
        }
      }
      stack->erase(n);  // Pop from the stack.
    }
    return false;
  }

  // Evaluates the graph of port dependencies -- including *connections* between
  // output ports and input ports and direct feedthrough connections between
  // input ports and output ports. If an algebraic loop is detected, throws
  // a std::logic_error.
  void ThrowIfAlgebraicLoopsExist() const {
    // Each port in the diagram is a node in a graph.
    // An edge exists from node u to node v if:
    //  1. output u is connected to input v (via Connect(u, v) method), or
    //  2. a direct feedthrough from input u to output v is reported.
    // A depth-first search of the graph should produce a forest of valid trees
    // if there are no algebraic loops. Otherwise, at least one link moving
    // *up* the tree will exist.

    // Build the graph.
    // Generally, the nodes of the graph would be the set of all defined ports
    // (input and output) of each subsystem. However, we only need to
    // consider the input/output ports that have a diagram level output-to-input
    // connection (ports that are not connected in this manner cannot contribute
    // to an algebraic loop).

    // Track *all* of the nodes involved in a diagram-level connection as
    // described above.
    std::set<PortIdentifier> nodes;
    // A map from node u, to the set of edges { (u, v_i) }. In normal cases,
    // not every node in `nodes` will serve as a key in `edges` (as that is a
    // necessary condition for there to be no algebraic loop).
    std::map<PortIdentifier, std::set<PortIdentifier>> edges;

    // In order to store PortIdentifiers for both input and output ports in the
    // same set, I need to encode the ports. The identifier for the first input
    // port and output port look identical (same system pointer, same port
    // id 0). So, to distinguish them, I'll modify the output ports to use the
    // negative half of the int space. The function below provides a utility for
    // encoding an output port id.
    auto output_to_key = [](int port_id) { return -(port_id + 1); };

    // Populate the node set from the connections (and define the edges implied
    // by those connections).
    for (const auto& connection : dependency_graph_) {
      // Dependency graph is a mapping from the destination of the connection
      // to what it *depends on* (the source).
      const PortIdentifier& src = connection.second;
      const PortIdentifier& dest = connection.first;
      PortIdentifier encoded_src{src.first, output_to_key(src.second)};
      nodes.insert(encoded_src);
      nodes.insert(dest);
      edges[encoded_src].insert(dest);
    }

    // Populate more edges based on direct feedthrough.
    for (const auto& system : registered_systems_) {
      for (const auto& pair : system->GetDirectFeedthroughs()) {
        PortIdentifier src_port{system.get(), pair.first};
        PortIdentifier dest_port{system.get(), output_to_key(pair.second)};
        if (nodes.count(src_port) > 0 && nodes.count(dest_port) > 0) {
          // Track direct feedthrough only on port pairs where *both* ports are
          // connected to other ports at the diagram level.
          edges[src_port].insert(dest_port);
        }
      }
    }

    // Evaluate the graph for cycles.
    std::set<PortIdentifier> visited;
    std::set<PortIdentifier> stack;
    for (const auto& node : nodes) {
      if (visited.count(node) == 0) {
        if (HasCycleRecurse(node, edges, &visited, &stack)) {
          throw std::logic_error("Algebraic loop detected in DiagramBuilder.");
        }
      }
    }
  }

  // TODO(russt): Implement AddRandomSources method to wire up all dangling
  // random input ports with a compatible RandomSource system.

  // Produces the Blueprint that has been described by the calls to
  // Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  // graph is empty or contains algebraic loops.
  // The DiagramBuilder passes ownership of the registered systems to the
  // blueprint.
  std::unique_ptr<typename Diagram<T>::Blueprint> Compile() {
    if (registered_systems_.size() == 0) {
      throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
    }
    ThrowIfAlgebraicLoopsExist();

    auto blueprint = std::make_unique<typename Diagram<T>::Blueprint>();
    blueprint->input_port_ids = input_port_ids_;
    blueprint->output_port_ids = output_port_ids_;
    blueprint->dependency_graph = dependency_graph_;
    blueprint->systems = std::move(registered_systems_);

    return blueprint;
  }

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // A mirror on the systems in the diagram. Should have the same values as
  // registered_systems_. Used for fast membership queries.
  std::unordered_set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  friend int AddRandomInputs(double, systems::DiagramBuilder<double>*);
};

}  // namespace systems
}  // namespace drake
