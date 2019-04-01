#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render/render_label_class.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** The class responsible for allocating render labels and tracking those that
 have been allocated.  */
class RenderLabelManager {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabelManager)

  /** Constructs the manager attributing ownership of the reserved render label
   classes to the source with the given `id` (typically the owning SceneGraph).
   */
  explicit RenderLabelManager(SourceId id);

  /** Implementation of SceneGraph::GetRenderLabel().  */
  RenderLabel GetRenderLabel(SourceId source_id, std::string class_name);

  /** Returns a mapping from a render label class name to one or more pairs
   consisting of 1. the name of the source that requested that label class and
   2. the render label value for that source.
   @param source_names A map from SourceId to the corresponding source name.
   @throws std::logic_error if a source that has requested a render label is not
                            included in the map.  */
  std::unordered_map<std::string,
                     std::vector<std::pair<std::string, RenderLabel>>>
  GetLabelsByName(
      const std::unordered_map<SourceId, std::string>& source_names) const;

 private:
  friend class RenderLabelManagerTester;

  // A map from the name of the label to each of the render labels associated
  // with that name (they *must* all have different source ids).
  std::unordered_map<std::string, std::vector<RenderLabelClass>>
      name_label_map_;

  using ValueType = RenderLabel::ValueType;

  // The next value to provide when allocating a RenderLabel.
  ValueType next_value_{0};

  // The maximum allowable render label value. This is mostly a convenience
  // lever to test logic.
  ValueType maximum_value_{-1};
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
