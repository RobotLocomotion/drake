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

The geometry data is partitioned by geometric role and have independent version
number values. Some SceneGraph's API (as variously documented) will cause one or
more to change. This class provides the API `SameVersionAs` that takes another
GeometryVersion as well as a Role to help detect whether the provided role of
the geometries may have changed.

This class is not thread-safe.
*/

// GeometryVersion contains three members to help detect equivalence and
// differences among geometry data.
//
//     `self_data` records the number of revisions since the geometries are
//     created.
//
//     `parent_data` records the number of revisions of the data in its the
//     original source at the time the geometries
//
//     `version_id` records the id of the original source for each data as well
//     as the id of `this` geometry data.
//
// The goal is for the parent to define and represent a unique equivalence class
// of initial values. If at the time a context clones itself, the source has not
// yet modified the data since its creation, then the copy belongs to the same
// equivalence class as the source because they share the same initial value.
//
// When comparing two geometry data, it is easy to compare two revisions of the
// geometries from the same context. It is also easy to compare geometries from
// different contexts if they both have been modified since creation: they
// cannot be of the same version. the subtlety emerges when comparing geometries
// from different contexts where one set or both sets of geometries are
// unmodified since creation. In this case, we rely on the equivalence class
// built with the `parent_data` and `version_id` to track the chain of
// transitivity.
class GeometryVersion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryVersion);
  /// Returns true if `this` GeometryVersion has the same `role`
  /// version as the `other` GeometryVersion, and returns false if `this`
  /// GeometryVersion has a different `role` version as the `other`
  /// GeometryVersion.
  bool SameVersionAs(const GeometryVersion& other, Role role) const {
    if (version_id_.self_id == other.version_id_.self_id) {
      return self_data_.get_value(role) == other.self_data_.get_value(role);
    }
    // Find the id of origin of the data from `this` and `other`.
    // The origin is data is itself if it has been modified since create,
    // otherwise, the id of the origin is stored in the parent_id.
    const auto& this_proxy_id = self_data_.get_value(role) == 0
                                    ? version_id_.get_parent_id(role)
                                    : version_id_.self_id;
    const auto& other_proxy_id = other.self_data_.get_value(role) == 0
                                     ? other.version_id_.get_parent_id(role)
                                     : other.version_id_.self_id;
    // If `this` and `other` point to different origins, it doesn't matter what
    // value they hold, they can't be the same version.
    if (this_proxy_id != other_proxy_id) {
      return false;
    }
    // Now that we know they stem from the same origin, find the value of
    // `this` and `other` at the origin.
    const auto& this_proxy_value = self_data_.get_value(role) == 0
                                       ? parent_data_.get_value(role)
                                       : self_data_.get_value(role);
    const auto& other_proxy_value = other.self_data_.get_value(role) == 0
                                        ? other.parent_data_.get_value(role)
                                        : other.self_data_.get_value(role);
    return this_proxy_value == other_proxy_value;
  }

 private:
  struct VersionData {
    int64_t get_value(Role role) const {
      switch (role) {
        case Role::kUnassigned:
          throw std::logic_error(
              "Trying to get the version number of an unassigned role.");
        case Role::kProximity:
          return proximity;
        case Role::kIllustration:
          return illustration;
        case Role::kPerception:
          return perception;
      }
      DRAKE_UNREACHABLE();
    }
    int64_t proximity{0};
    int64_t perception{0};
    int64_t illustration{0};
  };

  struct VersionId {
    using GeometryVersionId = Identifier<class GeometryVersionTag>;
    VersionId()
        : proximity_parent_id(GeometryVersionId::get_new_id()),
          perception_parent_id(proximity_parent_id),
          illustration_parent_id(proximity_parent_id),
          self_id(GeometryVersionId::get_new_id()) {}
    const GeometryVersionId& get_parent_id(Role role) const {
      switch (role) {
        case Role::kUnassigned:
          throw std::logic_error(
              "Trying to get the parent version id of an unassigned role.");
        case Role::kProximity:
          return proximity_parent_id;
        case Role::kIllustration:
          return illustration_parent_id;
        case Role::kPerception:
          return perception_parent_id;
      }
      DRAKE_UNREACHABLE();
    }
    GeometryVersionId proximity_parent_id;
    GeometryVersionId perception_parent_id;
    GeometryVersionId illustration_parent_id;
    GeometryVersionId self_id;
  };

  // Only GeometryState can update the version numbers.
  template <typename T>
  friend class GeometryState;

  // Facilitates testing.
  friend class GeometryVersionTest;

  using GeometryVersionId = Identifier<class GeometryVersionTag>;

  // Only GeometryState can construct GeometryVersion. Downstream systems
  // should obtain a reference through the API provided by SceneGraphInspector
  // if they wish to inquire the version numbers. They then may choose to retain
  // a copy if needed.

  // Default constructor sets all version data to zero, sets all parent id to be
  // the same and sets a unique self id.
  GeometryVersion() = default;

  // Alternative constructor that sets self version data to zero, sets parent
  // version data/id to either the source's data/id or the source's parent's
  // data/id depending on whether its source's data is zero.
  GeometryVersion(const VersionData& source_parent_data,
                  const VersionData& source_data, const VersionId& source_id) {
    // Proximity
    if (source_data.proximity == 0) {
      parent_data_.proximity = source_parent_data.proximity;
      version_id_.proximity_parent_id = source_id.proximity_parent_id;
    } else {
      parent_data_.proximity = source_data.proximity;
      version_id_.proximity_parent_id = source_id.self_id;
    }

    // Perception
    if (source_data.perception == 0) {
      parent_data_.perception = source_parent_data.perception;
      version_id_.perception_parent_id = source_id.perception_parent_id;
    } else {
      parent_data_.perception = source_data.perception;
      version_id_.perception_parent_id = source_id.self_id;
    }

    // Illustration
    if (source_data.illustration == 0) {
      parent_data_.illustration = source_parent_data.illustration;
      version_id_.illustration_parent_id = source_id.illustration_parent_id;
    } else {
      parent_data_.illustration = source_data.illustration;
      version_id_.illustration_parent_id = source_id.self_id;
    }
  }

  void increment_proximity() { ++self_data_.proximity; }
  void increment_perception() { ++self_data_.perception; }
  void increment_illustration() { ++self_data_.illustration; }

 private:
  VersionData parent_data_;
  VersionData self_data_;
  VersionId version_id_;
};
}  // namespace geometry
}  // namespace drake
