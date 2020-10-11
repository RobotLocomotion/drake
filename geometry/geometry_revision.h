#pragma once

#include <cstdint>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** A revision number class to keep track of geometry changes in GeometryState
that is owned by GeometryState and can be queried by SceneGraphInspector, but
can only be incremented by GeometryState.

SceneGraphInspector provides a public API that return a copy of
the GeometryRevision owned by GeometryState that downstream systems hold in
order to detect (non-pose) changes in GeometryState.

Two different context can report the same revision number, but that does not
imply the contents are the same. These revision numbers are just the number of
revisions of the GeometryState that owns them.

Note that this class is not thread-safe. */

class GeometryRevision {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryRevision);

  int64_t proximity_revision() const { return proximity_revision_; }

  int64_t perception_revision() const { return perception_revision_; }

  int64_t illustration_revision() const { return illustration_revision_; }

 private:
  // Only GeometryState can update the revision numbers.
  template <typename T>
  friend class GeometryState;

  // Facilitates testing.
  friend class GeometryRevisionTest;

  // Only GeometryState can construct GeometryRevision. Downstream systems
  // should obtain a copy through the API provided by SceneGraphInspector if
  // they wish to inquire the revision numbers.
  GeometryRevision() = default;

  void increment_proximity_revision() { ++proximity_revision_; }

  void increment_perception_revision() { ++perception_revision_; }

  void increment_illustration_revision() { ++illustration_revision_; }

  int64_t proximity_revision_{0};
  int64_t perception_revision_{0};
  int64_t illustration_revision_{0};
};
}  // namespace geometry
}  // namespace drake
