#pragma once

#include <map>
#include <string>
#include <vector>

#include <common_robotics_utilities/serialization.hpp>
#include <common_robotics_utilities/simple_graph.hpp>

#include "planning/roadmap.h"

namespace anzu {
namespace planning {
namespace internal {
// Convenience typedefs for roadmap graph, graph edge, and graph node types.
template<typename StateType>
using RoadmapGraph = common_robotics_utilities::simple_graph::Graph<StateType>;
using RoadmapGraphEdge = common_robotics_utilities::simple_graph::GraphEdge;
template<typename StateType>
using RoadmapGraphNode =
    common_robotics_utilities::simple_graph::GraphNode<StateType>;

// Helper to retrieve the mutable internal roadmap graph from a Roadmap.
template<typename StateType>
RoadmapGraph<StateType>& GetMutableRoadmapInternalGraph(
    // NOLINTNEXTLINE(runtime/references)
    Roadmap<StateType>& roadmap) {
  return *reinterpret_cast<RoadmapGraph<StateType>*>(
      roadmap.mutable_internal_representation());
}

// Helper to retrieve the internal roadmap graph from a Roadmap.
template<typename StateType>
const RoadmapGraph<StateType>& GetRoadmapInternalGraph(
    const Roadmap<StateType>& roadmap) {
  return *reinterpret_cast<const RoadmapGraph<StateType>*>(
      roadmap.internal_representation());
}

template<typename StateType>
using DeserializedRoadmap =
    common_robotics_utilities::serialization::Deserialized<Roadmap<StateType>>;

template<typename StateType>
using DeserializedNamedRoadmaps =
    common_robotics_utilities::serialization
        ::Deserialized<std::map<std::string, Roadmap<StateType>>>;

template<typename StateType>
class RoadmapSerializer {
 public:
  // Mutable reference used for API compatibility with CRU serialization.
  static uint64_t SerializeRoadmap(
      const Roadmap<StateType>& roadmap,
      // NOLINTNEXTLINE(runtime/references)
      std::vector<uint8_t>& buffer);

  static DeserializedRoadmap<StateType> DeserializeRoadmap(
      const std::vector<uint8_t>& buffer, uint64_t starting_offset);

  // Mutable reference used for API compatibility with CRU serialization.
  static uint64_t SerializeNamedRoadmaps(
      const std::map<std::string, Roadmap<StateType>>& named_roadmaps,
      // NOLINTNEXTLINE(runtime/references)
      std::vector<uint8_t>& buffer);

  static DeserializedNamedRoadmaps<StateType> DeserializeNamedRoadmaps(
      const std::vector<uint8_t>& buffer, uint64_t starting_offset);
};
}  // namespace internal
}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::internal::RoadmapSerializer)
