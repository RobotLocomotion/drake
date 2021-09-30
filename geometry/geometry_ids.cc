#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// N.B. This definition appears in the cc file because get_new_id should never
// be called from a header file.
GeometryId GeometryId::get_new_id() {
  auto base_id = Base::get_new_id();
  return GeometryId(base_id.get_value());
}

}  // namespace geometry

// Add explicit template instantiations to ensure that pydrake can link.
template class Identifier<geometry::FilterTag>;
template class Identifier<geometry::SourceTag>;
template class Identifier<geometry::FrameTag>;
template class Identifier<geometry::GeometryTag>;

}  // namespace drake
