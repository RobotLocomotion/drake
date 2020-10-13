#pragma once

#include <cstdint>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/**
A version numbering class that reports revisions of SceneGraph's geometric data.

Other Systems can use this version number to perform updates when they detect
changes to the geometric data they consume. The version of the geometry data is
made available through SceneGraphInspector.

The geometry data is partitioned by geometric role and have independent version
number values. Some SceneGraph's API (as variously documented) will cause one or
more to be incremented. A higher version number always represents a later
version of the data.

SceneGraph's internal model, and every allocated context have independent copies
of their own version number values. When allocating a context, the context's
geometry version will initially match SceneGraph's model version. Subsequent
modifications to either one will cause its version number value to move
independently.

This class is not thread-safe.
*/
class GeometryVersion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryVersion);

  int64_t proximity() const { return proximity_version_; }

  int64_t perception() const { return perception_version_; }

  int64_t illustration() const { return illustration_version_; }

 private:
  // Only GeometryState can update the version numbers.
  template <typename T>
  friend class GeometryState;

  // Facilitates testing.
  friend class GeometryVersionTest;

  // Only GeometryState can construct GeometryVersion. Downstream systems
  // should obtain a copy through the API provided by SceneGraphInspector if
  // they wish to inquire the version numbers.
  GeometryVersion() = default;

  void increment_proximity() { ++proximity_version_; }

  void increment_perception() { ++perception_version_; }

  void increment_illustration() { ++illustration_version_; }

  int64_t proximity_version_{0};
  int64_t perception_version_{0};
  int64_t illustration_version_{0};
};
}  // namespace geometry
}  // namespace drake
