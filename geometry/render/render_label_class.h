#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

}  // namespace internal

/** The definition of a render label class. This includes the class name, the
 associated RenderLabel, and the id of the geometry source that requested the
 render label.  */
struct RenderLabelClass {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabelClass)

  RenderLabelClass(std::string name_in, SourceId id_in, RenderLabel label_in) :
      name(std::move(name_in)), source_id(id_in), label(label_in) {}

  /** The source-local unique name of the class. The name may be shared by
   multiple sources.  */
  std::string name;

  // TODO(SeanCurtis-TRI): I shouldn't leak a source id here. It should be the
  // source name, otherwise one could infer the source id and hijack the result.
  /** The source id that requested the render label.  */
  SourceId source_id;

  /** The label associated with the class.  */
  RenderLabel label;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
