#pragma once

#include <string>

namespace drake {
namespace maliput {
namespace api {

class RoadGeometry;
class Segment;


/// Persistent identifier for a Junction element.
struct JunctionId {
  std::string id;
};


/// A Junction is a closed set of Segments which have physically
/// coplanar road surfaces, in the sense that RoadPositions with the
/// same h value (height above surface) in the domains of two Segments
/// map to the same GeoPosition.  The Segments need not be directly
/// connected to one another in the network topology.
///
/// Junctions are grouped by RoadGeometry.
class Junction {
 public:
  virtual ~Junction() {}

  /// Returns the persistent identifier.
  const JunctionId id() const { return do_id(); }

  /// Returns the RoadGeometry to which this Junction belongs.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  /// Returns the number of Segments in the Junction.
  ///
  /// Return value is non-negative.
  int num_segments() const { return do_num_segments(); }

  /// Returns the Segment indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_segments().
  const Segment* segment(int index) const { return do_segment(index); }

  /// @name Deleted Copy/Move Operations
  /// Junction is neither copyable nor moveable.
  ///@{
  explicit Junction(const Junction&) = delete;
  Junction& operator=(const Junction&) = delete;
  ///@}

 protected:
  Junction() {}

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual const JunctionId do_id() const = 0;

  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual int do_num_segments() const = 0;

  virtual const Segment* do_segment(int index) const = 0;
  ///@}
};


}  // namespace api
}  // namespace maliput
}  // namespace drake
