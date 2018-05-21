#pragma once

#include <ostream>
#include <string>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

/// Defines the direction of an Endpoint or EndpointZ.
enum class Direction { kForward, kReverse };

/// Provides methods to build an StartReference::Spec.
class StartReference {
 public:
  /// Defines how a Connection's reference curve starts.
  ///
  /// Objects of this class should be created using StartReference::at()
  /// methods.
  class Spec {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Spec)

    const Endpoint& endpoint() const { return endpoint_; }

   private:
    // Allows StartReference factory to build objects of this class.
    friend class StartReference;

    // Constructs a Spec that specifies with `endpoint` how a Connection's
    // reference curve starts.
    explicit Spec(const Endpoint& endpoint) : endpoint_(endpoint) {}

    // Describes the connection's reference curve start-point.
    Endpoint endpoint_{};
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StartReference)

  StartReference() = default;

  /// Builds a Spec at `endpoint` with `direction` direction. When
  /// `direction` == `Direction::kReverse`, `endpoint` is reversed.
  Spec at(const Endpoint& endpoint, Direction direction) const {
    return direction == Direction::kForward ? Spec(endpoint)
                                            : Spec(endpoint.reverse());
  }

  /// Builds a Spec at `connection`'s `end` side with `direction` direction.
  /// When `direction` == `Direction::kReverse`, `endpoint` is reversed.
  Spec at(const Connection& connection, api::LaneEnd::Which end,
          Direction direction) const {
    const Endpoint endpoint = end == api::LaneEnd::Which::kStart
                                  ? connection.start()
                                  : connection.end();
    return direction == Direction::kForward ? Spec(endpoint)
                                            : Spec(endpoint.reverse());
  }
};

/// Streams a string representation of `start_spec` into `out`. Returns `out`.
/// This method is provided for the purposes of debugging or text-logging.
/// It is not intended for serialization.
std::ostream& operator<<(std::ostream& out,
                         const StartReference::Spec& start_spec);

/// Provides methods to build an EndReference::Spec.
class EndReference {
 public:
  /// Defines how a Connection's reference curve ends.
  ///
  /// Objects of this class should be created using EndReference::z_at()
  /// methods.
  class Spec {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Spec)

    const EndpointZ& endpoint_z() const { return endpoint_z_; }

   private:
    // Allows EndReference factory to build objects of this class.
    friend class EndReference;

    /// Constructs a Spec at that specifies with `endpoint_z` how a
    /// Connection's reference curve ends.
    explicit Spec(const EndpointZ& endpoint_z) : endpoint_z_(endpoint_z) {}

    // Describes the connection's reference curve end-point.
    EndpointZ endpoint_z_;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EndReference)

  EndReference() = default;

  /// Builds a Spec at `connection`'s `end` side with `direction` direction.
  /// When `direction` == `Direction::kReverse`, `end`-side endpoint's
  /// EndpointZ is reversed.
  Spec z_at(const Connection& connection, api::LaneEnd::Which end,
            Direction direction) const {
    const EndpointZ endpoint_z = end == api::LaneEnd::Which::kStart
                                     ? connection.start().z()
                                     : connection.end().z();
    return direction == Direction::kForward ? Spec(endpoint_z)
                                            : Spec(endpoint_z.reverse());
  }

  /// Builds an Spec at `endpoint_z` with `direction` direction.
  /// When `direction` == `Direction::kReverse`, `endpoint_z` is reversed.
  Spec z_at(const EndpointZ& endpoint_z, Direction direction) const {
    return direction == Direction::kForward ? Spec(endpoint_z)
                                            : Spec(endpoint_z.reverse());
  }
};

/// Streams a string representation of `end_spec` into `out`. Returns `out`.
/// This method is provided for the purposes of debugging or text-logging.
/// It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const EndReference::Spec& end_spec);

/// Wraps all the lane-related specifications in a Connection.
class LaneLayout {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneLayout)

  /// Constructs a the lane layout of a connection.
  ///
  /// Lane reference paths (which are offsets of parent Segment reference curve)
  /// are centered within the Lane. Lane spacing will be road geometry fixed
  /// lane's width. Segment extents will be derived from the composition of
  /// `left_shoulder` and `right_shoulder` shoulders, number of lanes and lane
  /// spacing. `ref_lane` lane's centerline will be placed at `ref_r0` distance
  /// from connection's reference curve.
  ///
  /// `left_shoulder` and `right_shoulder` must be nonnegative.
  /// `num_lanes` must be positive and `ref_lane` must be nonnegative and
  /// smaller than `num_lanes`.
  LaneLayout(double left_shoulder, double right_shoulder, int num_lanes,
             int ref_lane, double ref_r0)
      : left_shoulder_(left_shoulder),
        right_shoulder_(right_shoulder),
        num_lanes_(num_lanes),
        ref_lane_(ref_lane),
        ref_r0_(ref_r0) {
    DRAKE_DEMAND(left_shoulder_ >= 0.);
    DRAKE_DEMAND(right_shoulder_ >= 0.);
    DRAKE_DEMAND(num_lanes_ > 0);
    DRAKE_DEMAND(ref_lane_ >= 0 && ref_lane_ < num_lanes_);
  }

  double left_shoulder() const { return left_shoulder_; }

  double right_shoulder() const { return right_shoulder_; }

  int num_lanes() const { return num_lanes_; }

  int ref_lane() const { return ref_lane_; }

  double ref_r0() const { return ref_r0_; }

 private:
  // Extra space added to the right of the first lane.
  double left_shoulder_{};
  // Extra space added to the left of the last lane.
  double right_shoulder_{};
  // Number of lanes.
  int num_lanes_{};
  // Index of the lane from which `ref_r0_` is defined.
  int ref_lane_{};
  // Distance from `ref_lane_` lane's centerline to reference curve.
  double ref_r0_{};
};

/// Streams a string representation of `lane_layout` into `out`. Returns `out`.
/// This method is provided for the purposes of debugging or text-logging.
/// It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LaneLayout& lane_layout);


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
