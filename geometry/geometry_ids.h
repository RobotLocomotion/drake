#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/identifier.h"

namespace drake {
namespace geometry {

namespace internal {
class EncodedData;
}  // namespace internal

/** Type used to identify geometry sources in SceneGraph. */
using SourceId = drake::Identifier<class SourceTag>;

/** Type used to identify geometry frames in SceneGraph.*/
using FrameId = drake::Identifier<class FrameTag>;

/** Type used to identify geometry instances in SceneGraph. */
class GeometryId : public drake::Identifier<class GeometryTag> {
  using Base = drake::Identifier<class GeometryTag>;
 public:
  GeometryId() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryId);

  static GeometryId get_new_id() {
    auto base_id = Base::get_new_id();
    return GeometryId(base_id.get_value());
  }

 private:
  explicit GeometryId(int64_t value) : Base(value) {}

  // EncodedData needs to be able to create GeometryId from FCL-encoded data.
  // This gives it access to the otherwise inaccessible constructor.
  friend class internal::EncodedData;
};

}  // namespace geometry
}  // namespace drake

namespace std {

/** Enables use of the identifier to serve as a key in STL containers.
 @relates GeometryId
 */
template <>
struct hash<drake::geometry::GeometryId> {
  size_t operator()(const drake::geometry::GeometryId& id) const {
    return std::hash<int64_t>()(id.get_value());
  }
};

}  // namespace std
