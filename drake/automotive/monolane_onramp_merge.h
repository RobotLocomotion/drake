#pragma once

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

namespace drake {
namespace maliput {
namespace monolane {

enum ArcDirection {kCW, kCCW};

/// Stores uniform characteristics of the road network; i.e. bounds on the lane
/// width and driveable width. Default settings are stored if no others are
/// specified.
struct RoadCharacteristics {
  /// Constructor for using default road geometries.
  RoadCharacteristics() = default;

  /// Constructor for custom-set road geometries.
  RoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  // Default parameters.
  const double lane_width{4.};
  const double driveable_width{8.};

  const api::RBounds lane_bounds{-lane_width / 2., lane_width / 2.};
  const api::RBounds driveable_bounds{-driveable_width / 2.,
        driveable_width / 2.};
};

/// Allows recursive addition of lane primitives to sequentially pave an
/// unbranching road from some specified starting configuration.  If the
/// starting configuration is not provided, the road begins at the origin.
/// Upon construction, RoadSection takes owership of the builder, releasing it
/// upon finalizing.

// **** Can this be reversed??
// ****** Can groups be composed??
template <typename T>
class RoadSection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadSection)

  RoadSection(std::unique_ptr<Builder> builder, const std::string& name,
              const RoadCharacteristics& road)
  : name_(name), road_(road), b_(std::move(builder)) {}

  RoadSection(std::unique_ptr<Builder> builder, const std::string& name,
              const RoadCharacteristics& road, Endpoint starting_endpt)
      : name_(name), road_(road), last_endpoint_(starting_endpt),
        b_(std::move(builder)) {}

  std::unique_ptr<Group> own_group() {
    return std::make_unique<Group>(b_->MakeGroup(name_).get());
  };

  //int get_num_connections() { return group_.size(); }

  void AddArcSegment(const T& arc_length, const T& arc_radius,
                     const ArcDirection& dir, const EndpointZ& end_z);

  // Vanilla version without any height info.
  void AddArcSegment(const T& arc_length, const T& arc_radius,
                     const ArcDirection& dir);

  void AddStraightSegment(const T& segment_length, const EndpointZ& end_z);

  // Vanilla version without any height info.
  void AddStraightSegment(const T& segment_length);

  const Endpoint& get_last_endpoint() { return last_endpoint_; }

  std::unique_ptr<Builder> Finalize() { return std::move(b_); }

 private:
  const std::string name_;
  const RoadCharacteristics road_;
  // const Group* group_;

  const EndpointXy origin_xy_{0., 0., 0.};
  const EndpointZ flat_z_{0., 0., 0., 0.};

  Endpoint last_endpoint_{origin_xy_, flat_z_};
  int id_{0};

  std::unique_ptr<Builder> b_;
};

///

// **** Make this abstract, then derive from it a class in another file?
template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  MonolaneOnrampMerge() {
    b_.reset(new Builder(road_.lane_bounds, road_.driveable_bounds,
                         kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  }

  MonolaneOnrampMerge(const RoadCharacteristics& rc) : road_(rc) {
    b_.reset(new Builder(road_.lane_bounds, road_.driveable_bounds,
                         kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  };

  std::unique_ptr<const api::RoadGeometry> own_road_geometry() {
    return std::move(rg_);
  };

 private:
  void BuildOnramp();

  // ***** Is there a better place for this stuff???
  const double kLinearTolerance_ = 0.01;
  const double kAngularTolerance_ = 0.01 * M_PI;

  const RoadCharacteristics road_{};
  std::unique_ptr<const api::RoadGeometry> rg_;
  std::unique_ptr<Builder> b_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
