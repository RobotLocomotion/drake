#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;

std::unique_ptr<RenderEngine> RenderEngine::Clone() const {
  return std::unique_ptr<RenderEngine>(DoClone());
}

optional<RenderIndex> RenderEngine::RegisterVisual(
    GeometryIndex index, const drake::geometry::Shape& shape,
    const PerceptionProperties& properties,
    const RigidTransformd& X_WG, bool needs_updates) {
  optional<RenderIndex> render_index =
      DoRegisterVisual(shape, properties, X_WG);
  if (render_index) {
    if (needs_updates) {
      update_indices_.insert({*render_index, index});
    } else {
      anchored_indices_.insert({*render_index, index});
    }
  }
  return render_index;
}

optional<GeometryIndex> RenderEngine::RemoveGeometry(RenderIndex index) {
  // The underlying engine doesn't know if the geometry to remove or the
  // geometry that gets moved requires updates or not. As such, the removed
  // index and the moved index can arbitrarily come from either mapping.
  // Possible scenarios:
  // Remove index in map A
  //   Case 1: nothing moved
  //     1. Remove the (remove index, remove GeometryIndex) pair from A.
  //     2. return nullopt.
  //   Case 2: moved geometry in A (i.e., _same_ map)
  //     1. Remove the (moved index, moved GeometryIndex) pair from A.
  //     2. Add the (removed index, moved GeometryIndex) pair into A.
  //     3. Return moved GeometryIndex.
  //   Case 3: moved geometry in B (i.e., _different_ map)
  //     1. Remove the (removed index, removed GI) pair from A.
  //     2. Remove the (moved index, moved GI) pair from B.
  //     3. Add the (removed index, moved GI) pair to B.
  //     4. Return moved GeometryIndex.

  // Given an index, return a pointer to the anchored/update index map in which
  // this index belongs.
  auto get_index_map = [this](const RenderIndex r_index) {
    auto iter = update_indices_.find(r_index);
    if (iter != update_indices_.end()) return &update_indices_;
    iter = anchored_indices_.find(r_index);
    if (iter != anchored_indices_.end()) return &anchored_indices_;
    throw std::logic_error(fmt::format(
        "Error finding RenderIndex ({}) as either dynamic or anchored",
        r_index));
  };

  // Determine map A.
  std::unordered_map<RenderIndex, GeometryIndex>* map_A = get_index_map(index);

  GeometryIndex moved_internal_index;
  optional<RenderIndex> moved_index = DoRemoveGeometry(index);

  // Determine map B.
  std::unordered_map<RenderIndex, GeometryIndex>* map_B = nullptr;
  if (moved_index) {
    map_B = get_index_map(*moved_index);
    moved_internal_index = (*map_B)[*moved_index];
  }

  // Examine cases:
  map_A->erase(index);
  if (map_B == nullptr) {                        // Case 1.
    return nullopt;
  } else if (map_A == map_B) {                   // Case 2.
    map_A->erase(*moved_index);
    (*map_A)[index] = moved_internal_index;
  } else {                                       // Case 3.
    map_B->erase(*moved_index);
    (*map_B)[index] = moved_internal_index;
  }
  return moved_internal_index;
}

RenderLabel RenderEngine::GetRenderLabelOrThrow(
    const PerceptionProperties& properties) const {
  RenderLabel label =
      properties.GetPropertyOrDefault("label", "id", default_render_label_);
  if (label == RenderLabel::kUnspecified || label == RenderLabel::kEmpty) {
    throw std::logic_error(
        "Cannot register a geometry with the 'unspecified' or 'empty' render "
        "labels. The bad label may have come from a default-constructed "
        "RenderLabel or the RenderEngine may have provided it as a default for "
        "missing render labels in the properties.");
  }
  return label;
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
