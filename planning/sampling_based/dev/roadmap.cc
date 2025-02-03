#include "planning/roadmap.h"

#include <utility>
#include <vector>

#include <common_robotics_utilities/zlib_helpers.hpp>

#include "planning/roadmap_internal.h"

using common_robotics_utilities::zlib_helpers::CompressAndWriteToFile;
using common_robotics_utilities::zlib_helpers::LoadFromFileAndDecompress;

namespace anzu {
namespace planning {
namespace {
// Helper to copy the internal roadmap graph from a Roadmap.
template<typename StateType>
std::shared_ptr<void> CopyInternalRepresentation(
    const Roadmap<StateType>& roadmap) {
  auto copied_graph = std::make_shared<internal::RoadmapGraph<StateType>>(
      internal::GetRoadmapInternalGraph(roadmap));
  return std::shared_ptr<void>(copied_graph, copied_graph.get());
}
}  // namespace

template<typename StateType>
void Roadmap<StateType>::SaveRoadmapToFile(
    const Roadmap<StateType>& roadmap, const std::filesystem::path& file) {
  std::vector<uint8_t> buffer;
  internal::RoadmapSerializer<StateType>::SerializeRoadmap(roadmap, buffer);
  CompressAndWriteToFile(buffer, file.string());
}

template<typename StateType>
Roadmap<StateType> Roadmap<StateType>::LoadRoadmapFromFile(
    const std::filesystem::path& file) {
  const std::vector<uint8_t> decompressed_serialized_roadmap =
      LoadFromFileAndDecompress(file.string());
  const uint64_t starting_offset = 0;
  return internal::RoadmapSerializer<StateType>::DeserializeRoadmap(
      decompressed_serialized_roadmap, starting_offset).Value();
}

template<typename StateType>
void Roadmap<StateType>::SaveNamedRoadmapsToFile(
    const std::map<std::string, Roadmap<StateType>>& named_roadmaps,
    const std::filesystem::path& file) {
  std::vector<uint8_t> buffer;
  internal::RoadmapSerializer<StateType>::SerializeNamedRoadmaps(
      named_roadmaps, buffer);
  CompressAndWriteToFile(buffer, file.string());
}

template<typename StateType>
std::map<std::string, Roadmap<StateType>>
Roadmap<StateType>::LoadNamedRoadmapsFromFile(
    const std::filesystem::path& file) {
  const std::vector<uint8_t> decompressed_serialized_roadmaps =
      LoadFromFileAndDecompress(file.string());
  const uint64_t starting_offset = 0;
  return internal::RoadmapSerializer<StateType>::DeserializeNamedRoadmaps(
      decompressed_serialized_roadmaps, starting_offset).Value();
}

template<typename StateType>
Roadmap<StateType>::Roadmap(const int64_t initial_capacity) {
  Initialize(initial_capacity);
}

template<typename StateType>
Roadmap<StateType>::Roadmap(const Roadmap<StateType>& other) {
  internal_representation_ = CopyInternalRepresentation(other);
}

template<typename StateType>
Roadmap<StateType>::Roadmap(Roadmap<StateType>&& other) {
  internal_representation_ = std::move(other.internal_representation_);
  other.Initialize(0);
}

template<typename StateType>
Roadmap<StateType>& Roadmap<StateType>::operator=(
    const Roadmap<StateType>& other) {
  if (this != &other) {
    internal_representation_ = CopyInternalRepresentation(other);
  }
  return *this;
}

template<typename StateType>
Roadmap<StateType>& Roadmap<StateType>::operator=(Roadmap<StateType>&& other) {
  if (this != &other) {
    internal_representation_ = std::move(other.internal_representation_);
    other.Initialize(0);
  }
  return *this;
}

template<typename StateType>
int64_t Roadmap<StateType>::Size() const {
  return internal::GetRoadmapInternalGraph(*this).Size();
}

template<typename StateType>
void Roadmap<StateType>::Initialize(const int64_t initial_capacity) {
  auto internal_graph =
      std::make_shared<internal::RoadmapGraph<StateType>>(initial_capacity);
  internal_representation_ =
      std::shared_ptr<void>(internal_graph, internal_graph.get());
}
}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::Roadmap)
