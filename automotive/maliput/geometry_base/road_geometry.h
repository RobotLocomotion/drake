#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/basic_id_index.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/geometry_base/branch_point.h"
#include "drake/automotive/maliput/geometry_base/junction.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace geometry_base {

/// @namespace drake::maliput::geometry_base
///
/// geometry_base provides basic implementations for a subset of the
/// interfaces of maliput's geometry API (api::RoadGeometry, etc) that
/// can be shared by most "leaf" backends.  It is suitable for use as
/// base classes for a complete backend implementation, or for mock
/// implementations for unit tests (such as
/// test_utilities/mock_geometry.h).
///
/// geometry_base implements all the virtual methods involved in
/// managing the object graph of the road network.  It does not
/// implement any of the fundamental geometric methods that define the
/// immersion of lane-frame into world-frame; that is the job of each
/// specific backend.
// TODO(maddog@tri.global) Provide a basic naive implementation of
//                         RoadGeometry::DoToRoadPosition() which
//                         only requires generic calls to
//                         Lane::ToLanePosition().

/// geometry_base's implementation of api::RoadGeometry.
class RoadGeometry : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry);

  /// Constructs an empty RoadGeometry with the specified tolerances.
  ///
  /// @param id the ID of the RoadGeometry
  /// @param linear_tolerance the linear tolerance
  /// @param angular_tolerance the angular tolerance
  ///
  /// @throws std::exception if either `linear_tolerance` or
  ///         `angular_tolerance` or `scale_length` is non-positive.
  RoadGeometry(const api::RoadGeometryId& id,
               double linear_tolerance, double angular_tolerance,
               double scale_length)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance),
        scale_length_(scale_length) {
    DRAKE_THROW_UNLESS(linear_tolerance_ > 0.);
    DRAKE_THROW_UNLESS(angular_tolerance_ > 0.);
    DRAKE_THROW_UNLESS(scale_length_ > 0.);
  }

  /// Adds @p junction to this RoadGeometry.
  ///
  /// This RoadGeometry will take ownership of `junction` and will be assigned
  /// as its parent.
  ///
  /// @returns `junction`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Junction.
  ///
  /// @throws std::exception if `junction` is empty.
  template <class T>
  T* AddJunction(std::unique_ptr<T> junction) {
    static_assert(std::is_base_of<Junction, T>::value,
                  "T is not derived from geometry_base::Junction");
    T* const raw_pointer = junction.get();
    AddJunctionPrivate(std::move(junction));
    return raw_pointer;
  }

  /// Adds @p branch_point to this RoadGeometry.
  ///
  /// This RoadGeometry will take ownership of `branch_point` and will be
  /// assigned as its parent.
  ///
  /// @returns `branch_point`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::BranchPoint.
  ///
  /// @throws std::exception if `branch_point` is empty.
  template <class T>
  T* AddBranchPoint(std::unique_ptr<T> branch_point) {
    static_assert(std::is_base_of<BranchPoint, T>::value,
                  "T is not derived from geometry_base::BranchPoint");
    T* const raw_pointer = branch_point.get();
    AddBranchPointPrivate(std::move(branch_point));
    return raw_pointer;
  }

  ~RoadGeometry() override = default;

 private:
  // The non-template implementation of AddJunction<T>()
  void AddJunctionPrivate(std::unique_ptr<Junction> junction);

  // The non-template implementation of AddBranchPoint<T>()
  void AddBranchPointPrivate(std::unique_ptr<BranchPoint> branch_point);

  api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const override;

  const IdIndex& DoById() const override { return id_index_; }

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  double do_scale_length() const override { return scale_length_; }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
  api::BasicIdIndex id_index_;
};


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
