#pragma once

#include <unordered_set>

#include "drake/geometry/geometry_set.h"

namespace drake {
namespace geometry {

// Utility class for testing the implementation details of the GeometrySet.
class GeometrySetTester {
 public:
  explicit GeometrySetTester(const GeometrySet* set) : set_(*set) {}

  const std::unordered_set<FrameId> frames() const {
    return set_.frames();
  }

  int num_frames() const { return set_.num_frames(); }

  const std::unordered_set<GeometryId> geometries() const {
    return set_.geometries();
  }

  int num_geometries() const { return set_.num_geometries(); }

  bool contains(FrameId frame_id) const {
    return set_.contains(frame_id); }

  bool contains(GeometryId geometry_id) const {
    return set_.contains(geometry_id);;
  }

 private:
  const GeometrySet& set_;
};

}  // namespace geometry
}  // namespace drake
