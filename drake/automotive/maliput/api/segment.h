#pragma once

#include <string>

namespace drake {
namespace maliput {
namespace api {

class Junction;
class Lane;


/// Persistent identifier for a Segment element.
struct SegmentId {
  std::string id;
};


/// A Segment represents a bundle of adjacent Lanes which share a
/// continuously traversable road surface.  Every LanePosition on a
/// given Lane of a Segment has a corresponding LanePosition on each
/// other Lane, all with the same height-above-surface h, that all
/// map to the same GeoPoint in 3-space.
///
/// Segments are grouped by Junction.
class Segment {
 public:
  virtual ~Segment() {}

  /// Returns the persistent identifier.
  const SegmentId id() const { return do_id(); }

  /// Returns the Junction to which this Segment belongs.
  const Junction* junction() const { return do_junction(); }

  /// Returns the number of Lanes contained in this Segment.
  ///
  /// Return value is non-negative.
  int num_lanes() const { return do_num_lanes(); }

  /// Returns the Lane indexed by @p index.
  /// The indexing order is meaningful; numerically adjacent indices correspond
  /// to geometrically adjacent Lanes.
  ///
  /// @pre @p index must be >= 0 and < num_lanes().
  // TODO(maddog@tri.global) Does increasing index value mean "to left" or
  //                         "to right"?  Resolve this by the first multilane
  //                         implementation.
  const Lane* lane(int index) const { return do_lane(index); }

  /// @name Deleted Copy/Move Operations
  /// Segment is neither copyable nor moveable.
  ///@{
  explicit Segment(const Segment&) = delete;
  Segment& operator=(const Segment&) = delete;
  ///@}

 protected:
  Segment() {}

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual const SegmentId do_id() const = 0;

  virtual const Junction* do_junction() const = 0;

  virtual int do_num_lanes() const = 0;

  virtual const Lane* do_lane(int index) const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
