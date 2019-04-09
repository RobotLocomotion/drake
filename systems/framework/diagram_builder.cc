#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

// This generic port identifier is used only for cycle detection below
// because the algorithm treats both input & output ports as nodes.
using PortIdentifier = std::pair<const SystemBase*, int>;

// Helper to do the algebraic loop test. It recursively performs the
// depth-first search on the graph to find cycles.
bool HasCycleRecurse(
    const PortIdentifier& n,
    const std::map<PortIdentifier, std::set<PortIdentifier>>& edges,
    std::set<PortIdentifier>* visited,
    std::vector<PortIdentifier>* stack) {
  DRAKE_ASSERT(visited->count(n) == 0);
  visited->insert(n);

  auto edge_iter = edges.find(n);
  if (edge_iter != edges.end()) {
    DRAKE_ASSERT(std::find(stack->begin(), stack->end(), n) == stack->end());
    stack->push_back(n);
    for (const auto& target : edge_iter->second) {
      if (visited->count(target) == 0 &&
          HasCycleRecurse(target, edges, visited, stack)) {
        return true;
      } else if (std::find(stack->begin(), stack->end(), target) !=
                 stack->end()) {
        return true;
      }
    }
    stack->pop_back();
  }
  return false;
}

}  // namespace

void DiagramBuilderImpl::ThrowIfAlgebraicLoopsExist(
    const std::unordered_set<const SystemBase*>& systems,
    const std::map<
        std::pair<const SystemBase*, InputPortIndex>,
        std::pair<const SystemBase*, OutputPortIndex>>& connection_map) {
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
  for (const auto& connection : connection_map) {
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
  for (const auto& system : systems) {
    for (const auto& pair : system->GetDirectFeedthroughs()) {
      PortIdentifier src_port{system, pair.first};
      PortIdentifier dest_port{system, output_to_key(pair.second)};
      if (nodes.count(src_port) > 0 && nodes.count(dest_port) > 0) {
        // Track direct feedthrough only on port pairs where *both* ports are
        // connected to other ports at the diagram level.
        edges[src_port].insert(dest_port);
      }
    }
  }

  // Evaluate the graph for cycles.
  std::set<PortIdentifier> visited;
  std::vector<PortIdentifier> stack;
  for (const auto& node : nodes) {
    if (visited.count(node) == 0) {
      if (HasCycleRecurse(node, edges, &visited, &stack)) {
        std::stringstream ss;

        auto port_to_stream = [&ss](const auto& id) {
          ss << "  " << id.first->get_name() << ":";
          if (id.second < 0)
            ss << "Out(";
          else
            ss << "In(";
          ss << (id.second >= 0 ? id.second : -id.second - 1) << ")";
        };

        ss << "Algebraic loop detected in DiagramBuilder:\n";
        for (size_t i = 0; i < stack.size() - 1; ++i) {
          port_to_stream(stack[i]);
          ss << " depends on\n";
        }
        port_to_stream(stack.back());

        throw std::runtime_error(ss.str());
      }
    }
  }
}

}  // namespace internal
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder)
