#pragma once

// Exclude internal class from doxygen's view.
#if !defined(DRAKE_DOXYGEN_CXX)

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
 have been allocated by SceneGraph.

 A new RenderLabel is allocated for every unique semantic class (name and source
 id pair). If GetRenderLabel() is called multiple times with the same name and
 source id, the same RenderLabel value will be returned each time.

 The %RenderLabelManager is copyable to facilitate SceneGraph cloning. Each copy
 contains the same record of allocated labels up to the point of copying, but
 subsequent allocations will be independent.

 @see RenderLabel  */
class RenderLabelManager {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabelManager)

  /** Constructs the manager attributing ownership of the reserved semantic
   classes to the source with the given `id` (expected to be the id of the
   owning SceneGraph).
   */
  explicit RenderLabelManager(SourceId id);

  /** Implementation of SceneGraph::GetRenderLabel().  */
  RenderLabel GetRenderLabel(SourceId source_id, std::string class_name);

  /** Provides a report of all the allocated RenderLabel values and their
   semantic classes (in no particular order).  */
  std::vector<RenderLabelClass> GetRenderLabelClasses() const;

 private:
  friend class RenderLabelManagerTester;

  // A map from the name of the label to each of the semantic classes associated
  // with that name (they *must* all have different source ids).
  std::unordered_multimap<std::string, RenderLabelClass> name_label_map_;

  using ValueType = RenderLabel::ValueType;

  // The next value to provide when allocating a RenderLabel.
  ValueType next_value_{0};

  // The maximum allowable render label value. This is mostly a convenience
  // lever to test logic.
  ValueType maximum_value_{RenderLabel::kMaxUnreserved};
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
