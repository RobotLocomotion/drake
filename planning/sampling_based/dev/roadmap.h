#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "planning/default_state_types.h"

namespace anzu {
namespace planning {
/// Roadmap type for use in PRM planning. Note that the Roadmap type serves to
/// hide the details of the internal representation of the roadmap graph.
/// For internal-only access to the roadmap graph, see planning/roadmap.cc and
/// planning/roadmap_internal.h for accessors and more information.
/// Note that loading and saving roadmaps and "named roadmaps" are all provided
/// via static methods to provide a more consistent API.
template<typename StateType>
class Roadmap {
 public:
  /// Saves the provided roadmap to the specified file.
  /// @param roadmap Roadmap to save.
  /// @param file Path of file to save to.
  static void SaveRoadmapToFile(
      const Roadmap<StateType>& roadmap, const std::filesystem::path& file);

  /// Loads a roadmap from the specified file.
  /// @param file Path of file to load from.
  /// @return Loaded roadmap.
  static Roadmap<StateType> LoadRoadmapFromFile(
      const std::filesystem::path& file);

  /// Saves the provided named roadmaps to the specified file.
  /// @param named_roadmaps Named roadmaps to save.
  /// @param file Path of file to save to.
  static void SaveNamedRoadmapsToFile(
      const std::map<std::string, Roadmap<StateType>>& named_roadmaps,
      const std::filesystem::path& file);

  /// Loads named roadmaps from the specified file.
  /// @param file Path of file to load from.
  /// @return Loaded named roadmaps.
  static std::map<std::string, Roadmap<StateType>> LoadNamedRoadmapsFromFile(
      const std::filesystem::path& file);

  /// Default-constructs an empty Roadmap.
  Roadmap() : Roadmap<StateType>(0) {}

  /// Constructs a Roadmap with an initial expected capacity, like
  /// vector.reserve().
  explicit Roadmap(int64_t initial_capacity);

  /// Roadmap provides copy, move, and assignment.
  Roadmap(const Roadmap<StateType>& other);
  Roadmap(Roadmap<StateType>&& other);
  Roadmap& operator=(const Roadmap<StateType>& other);
  Roadmap<StateType>& operator=(Roadmap<StateType>&& other);

  /// Gets the Roadmap size, i.e. the number of nodes in the roadmap graph.
  int64_t Size() const;

  // Internal-only, access to the internal roadmap graph.
  const void* internal_representation() const {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

  // Internal-only, mutable access to the internal roadmap graph.
  void* mutable_internal_representation() {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

 private:
  void Initialize(int64_t initial_capacity);

  // Opaque handle to the internal roadmap graph.
  // Note that while a shared_ptr is used here for type erasure, the internal
  // representation is never shared between Roadmaps.
  std::shared_ptr<void> internal_representation_;
};
}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::Roadmap)
