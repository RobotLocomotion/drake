#pragma once

// Exclude internal class from doxygen's view.
#if !defined(DRAKE_DOXYGEN_CXX)

#include <iostream>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** The definition of a semantic render class. As documented in RenderLabel,
 it is the association of a semantic class (a (source id, name) pair) with the
 allocated RenderLabel value.  */
struct RenderLabelClass {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabelClass)

  RenderLabelClass(std::string name_in, SourceId id_in, RenderLabel label_in) :
      name(std::move(name_in)), source_id(id_in), label(label_in) {}

  /** The source-local unique name of the class. The name may be shared by
   multiple sources.  */
  std::string name;

  // TODO(SeanCurtis-TRI): This is a potential leak of the SourceId. This class
  // is internal and isn't part of the public API, so it's not a leak today. If
  // it ever becomes part of the public API, it *cannot* contain a SourceId to
  // avoid becoming a leak.
  /** The source id that requested the render label.  */
  SourceId source_id;

  /** The label associated with the class.  */
  RenderLabel label;

  bool operator==(const RenderLabelClass& c2) const {
    return label == c2.label && source_id == c2.source_id && name == c2.name;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const RenderLabelClass& c) {
    out << "(" << c.name << ", Source(" << c.source_id << ")) --> " << c.label;
    return out;
  }
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
