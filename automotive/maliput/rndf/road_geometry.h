#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// An api::RoadGeometry implementation for RNDF specification.
/// Use the Builder interface to actually assemble a sensible road network.
/// Further information on RNDF specification can be found:
/// https://www.grandchallenge.org/grandchallenge/docs/RNDF_MDF_Formats_031407.pdf
///
/// api::RoadGeometry::ToRoadPosition should not be called as DoToRoadPosition
/// is not implemented.
class RoadGeometry : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs an empty RoadGeometry with the specified tolerances.
  ///
  /// @param id to set a name to the object.
  /// @param linear_tolerance is the tolerance in the linear sense for the
  /// api::RoadGeometry::CheckInvariants constraints checks.
  /// @param angular_tolerance is the tolerance in the angular sense for the
  /// api::RoadGeometry::CheckInvariants constraints checks.
  RoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
               double angular_tolerance)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  /// Creates and adds a new Junction with the specified @p id.
  /// @param id is the ID of the junction.
  /// @return A pointer to a Junction
  Junction* NewJunction(api::JunctionId id);

  /// Creates and adds a new BranchPoint with the specified @p id.
  /// @param id is the ID of the branch point.
  /// @return A pointer to a BranchPoint
  BranchPoint* NewBranchPoint(api::BranchPointId id);

  ~RoadGeometry() override = default;

 private:
  const api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  // This function will throw an exception if index is not bounded between 0
  // and the current maximum available index of junctions_ vector.
  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  // This function will throw an exception if index is not bounded between 0
  // and the current maximum available index of branch_points_ vector.
  const api::BranchPoint* do_branch_point(int index) const override;

  // TODO(maddog@tri.global) Implement when someone needs it.
  const IdIndex& DoById() const override { DRAKE_ABORT(); }

  // This function will abort as it's not implemented and should not be called.
  api::RoadPosition DoToRoadPosition(const api::GeoPosition& geo_pos,
                                     const api::RoadPosition* hint,
                                     api::GeoPosition* nearest_position,
                                     double* distance) const override;

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
