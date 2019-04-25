#pragma once

#include <string>

#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {

class RoadGeometry;
class Segment;


/// Persistent identifier for a Junction element.
using JunctionId = TypeSpecificIdentifier<class Junction>;


/// A Junction is a closed set of Segments which have physically
/// coplanar road surfaces, in the sense that RoadPositions with the
/// same h value (height above surface) in the domains of two Segments
/// map to the same GeoPosition.  The Segments need not be directly
/// connected to one another in the network topology.
///
/// Junctions are grouped by RoadGeometry.
class Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  virtual ~Junction() = default;

  /// Returns the persistent identifier.
  JunctionId id() const { return do_id(); }

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

 protected:
  Junction() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual JunctionId do_id() const = 0;

  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual int do_num_segments() const = 0;

  virtual const Segment* do_segment(int index) const = 0;
  ///@}
};


}  // namespace api
}  // namespace maliput
}  // namespace drake
