#pragma once

#include <cstdint>

#include "drake/common/drake_copyable.h"
#include "drake/common/identifier.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
/**
A version numbering class that reports revisions of SceneGraph's geometric data.

Other Systems can use this version number to perform updates when they detect
changes to the geometric data they consume. The version of the geometry data is
made available through SceneGraphInspector.

The geometry data is partitioned by geometric role and have independent role
version values. Some of SceneGraph's API (as variously documented) will cause
one or more role versions to change. This class provides the API `SameVersionAs`
that takes another GeometryVersion as well as a Role to help detect whether the
provided role of the geometries may have changed. For example:

 @code
 // Downstream system holds an instance of GeometryVersion `old_version` as a
 reference to compare against.
 // Get the version under test from SceneGraphInspector.
 const GeometryVersion& test_version =
 scene_graph_inspector.GetGeometryVersion();
 // Determine if two versions have the same proximity data.
 bool same_proximity = old_version.SameVersionAs(test_version,
 Role::kProximity);
 @endcode
*/
class GeometryVersion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryVersion);

  /** Returns true if `this` GeometryVersion has the same `role`
   version as the `other` GeometryVersion. */
  bool SameVersionAs(const GeometryVersion& other, Role role) const {
    switch (role) {
      case Role::kUnassigned:
        throw std::logic_error(
            "Trying to compare the version of unassigned roles.");
      case Role::kProximity:
        return proximity_version_id_ == other.proximity_version_id_;
      case Role::kIllustration:
        return illustration_version_id_ == other.illustration_version_id_;
      case Role::kPerception:
        return perception_version_id_ == other.perception_version_id_;
    }
    DRAKE_UNREACHABLE();
  }

 private:
  using RoleVersionId = Identifier<class RoleVersionTag>;
  /* Only GeometryState can update the role versions and construct
   GeometryVersion from scratch. Downstream systems should obtain a reference
   through the API provided by SceneGraphInspector if they wish to inquire the
   version. They then may choose to retain a copy if needed. */
  template <typename T>
  friend class GeometryState;

  /* Facilitates testing. */
  friend class GeometryVersionTest;

  /* Create unique version id for all roles so that the new geometry version
   unique. */
  GeometryVersion()
      : proximity_version_id_(RoleVersionId::get_new_id()),
        perception_version_id_(RoleVersionId::get_new_id()),
        illustration_version_id_(RoleVersionId::get_new_id()) {}

  void increment_proximity() {
    proximity_version_id_ = RoleVersionId::get_new_id();
  }

  void increment_perception() {
    perception_version_id_ = RoleVersionId::get_new_id();
  }

  void increment_illustration() {
    illustration_version_id_ = RoleVersionId::get_new_id();
  }

 private:
  RoleVersionId proximity_version_id_;
  RoleVersionId perception_version_id_;
  RoleVersionId illustration_version_id_;
};
}  // namespace geometry
}  // namespace drake
