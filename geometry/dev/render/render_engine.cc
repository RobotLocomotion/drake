#include "drake/geometry/dev/render/render_engine.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {

std::unique_ptr<RenderEngine> RenderEngine::Clone() const {
  return std::unique_ptr<RenderEngine>(DoClone());
}

optional<RenderIndex> RenderEngine::RegisterVisual(
    InternalIndex index, const drake::geometry::Shape& shape,
    const PerceptionProperties& properties, const Isometry3<double>& X_WG,
    bool needs_updates) {
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

optional<InternalIndex> RenderEngine::RemoveGeometry(RenderIndex index) {
  // The underlying engine doesn't know if the geometry to remove or the
  // geometry that gets moved requires updates or not. As such, the removed
  // index and the moved index can arbitrarily come from either mapping.
  // Possible scenarios:
  // Remove index in map A
  //   Case 1: nothing moved
  //     1. Remove the (remove index, remove InternalIndex) pair from A.
  //     2. return nullopt.
  //   Case 2: moved geometry in A (i.e., _same_ map)
  //     1. Remove the (moved index, moved InternalIndex) pair from A.
  //     2. Add the (removed index, moved InternalIndex) pair into A.
  //     3. Return moved InternalIndex.
  //   Case 3: moved geometry in B (i.e., _different_ map)
  //     1. Remove the (removed index, removed GI) pair from A.
  //     2. Remove the (moved index, moved GI) pair from B.
  //     3. Add the (removed index, moved GI) pair to B.
  //     4. Return moved InternalIndex.

  // Determine map A.
  auto* map_A = &update_indices_;
  {
    auto iter = update_indices_.find(index);
    if (iter == update_indices_.end()) {
      iter = anchored_indices_.find(index);
      map_A = &anchored_indices_;
      if (iter == anchored_indices_.end()) {
        throw std::logic_error(
            "Attempting to remove unrecognized render index from renderer");
      }
    }
  }

  std::unordered_map<RenderIndex, InternalIndex>* map_B = nullptr;
  InternalIndex moved_internal_index;
  optional<RenderIndex> moved_index = DoRemoveGeometry(index);
  // Determine map B.
  {
    if (moved_index) {
      auto iter = update_indices_.find(*moved_index);
      if (iter != update_indices_.end()) {
        map_B = &update_indices_;
      } else {
        iter = anchored_indices_.find(*moved_index);
        if (iter != anchored_indices_.end()) {
          map_B = &anchored_indices_;
        } else {
          // TODO(SeanCurtis-TRI): Make this DRAKE_UNREACHABLE().
          throw std::logic_error(
              "Implementation removed unrecognized render index");
        }
      }
      moved_internal_index = iter->second;
    }
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

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
