#pragma once

#include <cstdint>

#include "drake/common/drake_copyable.h"
#include "drake/common/identifier.h"

namespace drake {
namespace geometry {

/**
@anchor geometry_version
A version numbering class that reports revisions of SceneGraph's geometric data.

Other Systems can use this version number to perform updates when they detect
changes to the geometric data they consume. The version of the geometry data is
made available through SceneGraphInspector.

The geometry data is partitioned by geometric role and have independent version
number values. Some SceneGraph's API (as variously documented) will cause one or
more to change. This class provides API `SameFooAs` to help detect whether
the "Foo" role of the geometries may have changed.

This class is not thread-safe.
*/
class GeometryVersion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryVersion);

  /// Returns true if `this` GeometryVersion has the same proximity properties
  /// as the `other` GeometryVersion. If this method returns false, 'this'
  /// GeometryVersion may or may not have the same proximity properties as the
  /// 'other' GeometryVersion.
  bool SameProximityAs(const GeometryVersion& other) const {
    return proximity_version_ == other.proximity_version_ &&
           state_id_ == other.state_id_;
  }

  /// Returns true if `this` GeometryVersion has the same perception properties
  /// as the `other` GeometryVersion. If this method returns false, 'this'
  /// GeometryVersion may or may not have the same perception properties as the
  /// 'other' GeometryVersion.
  bool SamePerceptionAs(const GeometryVersion& other) const {
    return perception_version_ == other.perception_version_ &&
           state_id_ == other.state_id_;
  }

  /// Returns true if `this` GeometryVersion has the same illustration
  /// properties as the `other` GeometryVersion. If this method returns false,
  /// 'this' GeometryVersion may or may not have the same illustration
  /// properties as the 'other' GeometryVersion.
  bool SameIllustrationAs(const GeometryVersion& other) const {
    return illustration_version_ == other.illustration_version_ &&
           state_id_ == other.state_id_;
  }

 private:
  // Only GeometryState can update the version numbers.
  template <typename T>
  friend class GeometryState;
  // Allow Clone() in GeometryStateValue to modify the state_id_.
  template <typename T>
  friend class GeometryStateValue;

  // Facilitates testing.
  friend class GeometryVersionTest;

  // Only GeometryState can construct GeometryVersion. Downstream systems
  // should obtain a reference through the API provided by SceneGraphInspector
  // if they wish to inquire the version numbers. They then may choose to retain
  // a copy if needed.
  using GeometryVersionId = Identifier<class GeometryVersionTag>;
  explicit GeometryVersion(GeometryVersionId state_id)
      : proximity_version_(0),
        perception_version_(0),
        illustration_version_(0),
        state_id_(state_id) {}

  void increment_proximity() { ++proximity_version_; }

  void increment_perception() { ++perception_version_; }

  void increment_illustration() { ++illustration_version_; }

  void increment_state_id() { state_id_ = GeometryVersionId::get_new_id(); }

  int64_t proximity_version_{0};
  int64_t perception_version_{0};
  int64_t illustration_version_{0};
  GeometryVersionId state_id_;
};
}  // namespace geometry
}  // namespace drake
