#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/connection.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {
class RoadGeometry;

/// Base class to define a type that specifies a Connection endpoint. It could
/// be either at start or at the end of the reference curve or a lane curve.
class ConnectionSpec {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConnectionSpec)

  ConnectionSpec() = default;

  virtual ~ConnectionSpec() = default;

  /// Defines the direction of an Endpoint or EndpointZ.
  enum Direction { kForward, kReverse };
};

/// Interface to specify how a Connection's curve starts.
class StartConnectionSpec : public ConnectionSpec {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StartConnectionSpec)

  StartConnectionSpec() = default;

  virtual ~StartConnectionSpec() = default;

  /// @return An Endpoint that defines how the Connection's curve starts.
  virtual const Endpoint& endpoint() const = 0;
};

/// Defines how a Connection's reference curve starts.
///
/// Objects of this class should be created using StartReferenceSpecBuilder
/// methods.
class StartReferenceSpec : public StartConnectionSpec {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StartReferenceSpec)

  /// Constructs a ConnectionSpec at that specifies with `endpoint` how a
  /// Connection's reference curve starts.
  explicit StartReferenceSpec(const Endpoint& endpoint)
      : StartConnectionSpec(), endpoint_(endpoint) {}

  const Endpoint& endpoint() const override { return endpoint_; }

 private:
  // Describes the connection's reference curve start-point.
  Endpoint endpoint_{};
};

/// Provides methods to build an StartReferenceSpec.
///
/// Objects of this class should be created using
/// ConnectionSpecFabric::StartReference method.
class StartReferenceSpecBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StartReferenceSpecBuilder)

  StartReferenceSpecBuilder() = default;

  /// Builds a StartReferenceSpec at `endpoint` with `direction` direction. When
  /// `direction` == `ConnectionSpec::Direction::kReverse`, `endpoint` is
  /// reversed.
  StartReferenceSpec at(const Endpoint& endpoint,
                        ConnectionSpec::Direction direction) const {
    return direction == ConnectionSpec::Direction::kForward
               ? StartReferenceSpec(endpoint)
               : StartReferenceSpec(endpoint.reverse());
  }

  /// Builds a StartReferenceSpec at `connection`'s `end` side with `direction`
  /// direction. When `direction` == `ConnectionSpec::Direction::kReverse`,
  /// `endpoint` is reversed.
  StartReferenceSpec at(const Connection& connection, api::LaneEnd::Which end,
                        ConnectionSpec::Direction direction) const {
    const Endpoint endpoint = end == api::LaneEnd::Which::kStart
                                  ? connection.start()
                                  : connection.end();
    if (direction == ConnectionSpec::Direction::kReverse) {
      return StartReferenceSpec(endpoint.reverse());
    }
    return StartReferenceSpec(endpoint);
  }
};

/// Interface to specify how a Connection's curve ends.
class EndConnectionSpec : public ConnectionSpec {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndConnectionSpec)

  EndConnectionSpec() = default;

  virtual ~EndConnectionSpec() = default;

  /// @return An EndpointZ that defines how the Connection's curve ends.
  virtual const EndpointZ& endpoint_z() const = 0;
};

/// Defines how a Connection's reference curve ends.
///
/// Objects of this class should be created using EndReferenceSpecBuilder
/// methods.
class EndReferenceSpec : public EndConnectionSpec {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndReferenceSpec)

  /// Constructs a ConnectionSpec at that specifies with `endpoint_z` how a
  /// Connection's reference curve ends.
  explicit EndReferenceSpec(const EndpointZ& endpoint_z)
      : EndConnectionSpec(), endpoint_z_(endpoint_z) {}

  const EndpointZ& endpoint_z() const override { return endpoint_z_; }

 private:
  // Describes the connection's reference curve end-point.
  EndpointZ endpoint_z_;
};

/// Provides methods to build an EndReferenceSpec.
///
/// Objects of this class should be created using
/// ConnectionSpecFabric::EndReference method.
class EndReferenceSpecBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndReferenceSpecBuilder)

  EndReferenceSpecBuilder() = default;

  /// Builds an EndReferenceSpec at `endpoint` with `direction` direction. When
  /// `direction` == `ConnectionSpec::Direction::kReverse`, `endpoint.z()` is
  /// reversed.
  EndReferenceSpec at(const Endpoint& endpoint,
                      ConnectionSpec::Direction direction) const {
    return direction == ConnectionSpec::Direction::kForward
               ? EndReferenceSpec(endpoint.z())
               : EndReferenceSpec(endpoint.z().reverse());
  }

  /// Builds a EndReferenceSpec at `connection`'s `end` side with `direction`
  /// direction. When `direction` == `ConnectionSpec::Direction::kReverse`,
  /// `end`-side endpoint's EndpointZ is reversed.
  EndReferenceSpec at(const Connection& connection, api::LaneEnd::Which end,
                      ConnectionSpec::Direction direction) const {
    const EndpointZ endpoint_z = end == api::LaneEnd::Which::kStart
                                     ? connection.start().z()
                                     : connection.end().z();
    if (direction == ConnectionSpec::Direction::kReverse) {
      return EndReferenceSpec(endpoint_z.reverse());
    }
    return EndReferenceSpec(endpoint_z);
  }

  /// Builds an EndReferenceSpec at `endpoint_z` with `direction` direction.
  /// When `direction` == `ConnectionSpec::Direction::kReverse`, `endpoint_z` is
  /// reversed.
  EndReferenceSpec z_at(const EndpointZ& endpoint_z,
                        ConnectionSpec::Direction direction) const {
    return direction == ConnectionSpec::Direction::kForward
               ? EndReferenceSpec(endpoint_z)
               : EndReferenceSpec(endpoint_z.reverse());
  }
};

/// Fabric to create ConnectionSpec builders.
///
/// No objects of this class should be instantiated.
class ConnectionSpecFabric {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConnectionSpecFabric)

  ConnectionSpecFabric() = default;

  ~ConnectionSpecFabric() = default;

  /// @return A StartReferenceSpecBuilder instance.
  static StartReferenceSpecBuilder StartReference();

  /// @return A EndReferenceSpecBuilder instance.
  static EndReferenceSpecBuilder EndReference();
};

/// Wraps all the lane-related specifications in a Connection.
class LaneLayout {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneLayout)

  /// Constructs a the lane layout of a connection.
  ///
  /// `lane_width` is the width assigned to all connection's lanes. Lane
  /// reference path (which are offsets of parent Segment reference curve) are
  /// centered within the Lane. Lane spacing will be `lane_width` too. Segment
  /// extents will be derived from the composition of `left_shoulder` and
  /// `right_shoulder` shoulders, number of lanes and lane spacing.
  /// `ref_lane` lane's centerline will be placed at `ref_r0` distance from
  /// connection's reference curve.
  ///
  /// `lane_width` must be nonnegative.
  /// `left_shoulder` and `right_shoulder` must be nonnegative.
  /// `num_lanes` must be positive and `ref_lane` must be nonnegative and
  /// smaller than `num_lanes`.
  LaneLayout(double lane_width, double left_shoulder, double right_shoulder,
             int num_lanes, int ref_lane, double ref_r0)
      : lane_width_(lane_width),
        left_shoulder_(left_shoulder),
        right_shoulder_(right_shoulder),
        num_lanes_(num_lanes),
        ref_lane_(ref_lane),
        ref_r0_(ref_r0) {
    DRAKE_DEMAND(lane_width_ >= 0.);
    DRAKE_DEMAND(left_shoulder_ >= 0.);
    DRAKE_DEMAND(right_shoulder_ >= 0.);
    DRAKE_DEMAND(num_lanes_ > 0);
    DRAKE_DEMAND(ref_lane_ >= 0 && ref_lane_ < num_lanes_);
  }

  double lane_width() const { return lane_width_; }

  double left_shoulder() const { return left_shoulder_; }

  double right_shoulder() const { return right_shoulder_; }

  int num_lanes() const { return num_lanes_; }

  int ref_lane() const { return ref_lane_; }

  double ref_r0() const { return ref_r0_; }

 private:
  // Lanes' width.
  double lane_width_{};
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

/// Convenient builder class which makes it easy to construct a multilane road
/// network.
///
/// multilane is a simple road-network implementation:
///  - multiple lanes per segment;
///  - constant elevation_bounds, right and left shoulders are the same for all
///    lanes;
///  - only linear and constant-curvature-arc primitives in XY-plane;
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation;
///  - superelevation (bank of road) rotates around the reference line (r = 0)
///    of the path.
///
/// The Builder class simplifies the assembly of multilane road network
/// components into a valid RoadGeometry.  In the Builder model, an Endpoint
/// specifies a point in world coordinates (along with a direction, slope,
/// and superelevation parameters).  A Connection is a path from an explicit
/// start Endpoint to an end Endpoint calculated via a linear (LineOffset) or
/// arc displacement (ArcOffset).  A Group is a collection of Connections.
///
/// Builder::Build() constructs a RoadGeometry. Each Connection yields a
/// Segment bearing multiple Lanes. Each Group yields a Junction containing
/// the Segments associated with the grouped Connections; ungrouped
/// Connections each receive their own Junction.
///
/// Specific suffixes are used to name Maliput entities. The following list
/// explains the naming convention:
///
///  - Junctions: "j:" + Group::id(), or "j" + Connection::id() for an
///    ungrouped Connection.
///  - Segments: "s:" + Connection::id()
///  - Lanes: "l:" + Connection::id() + "_" + lane_index
///  - BranchPoints: "bp:" + branch_point_index
///
/// Note: 'lane_index' is the index in the Segment, and 'branch_point_index' is
/// is the index in the RoadGeometry.
class Builder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Builder)

  /// Constructs a Builder which can be used to specify and assemble a
  /// multilane implementation of an api::RoadGeometry.
  ///
  /// Lane reference path (which are offsets of parent Segment reference curve)
  /// are centered within the Lane. Lane spacing will be lane's width too.
  /// Segment extents will be derived from the composition of left and right
  /// shoulders, number of lanes and lane spacing. The `elevation_bounds` is
  /// applied uniformly to all lanes of every segment. `linear_tolerance` and
  /// `angular_tolerance` specify the respective tolerances for the resulting
  /// RoadGeometry.
  Builder(const api::HBounds& elevation_bounds, double linear_tolerance,
          double angular_tolerance);

  /// Connects `start_spec`'s Endpoint to an end-point linearly displaced from
  /// `start_spec`'s Endpoint.
  ///
  /// `line_offset` specifies the length of displacement (in the direction of
  /// the heading of `start_spec`'s Endpoint). `end_spec` specifies the
  /// elevation characteristics at the end-point.
  /// `lane_layout` defines the number of lanes, their width, extra shoulder
  /// asphalt extensions and placing with respect to connection's reference
  /// curve.
  const Connection* Connect(const std::string& id,
                            const LaneLayout& lane_layout,
                            const StartReferenceSpec& start_spec,
                            const LineOffset& line_offset,
                            const EndReferenceSpec& end_spec);

  /// Connects `start_spec`'s Endpoint to an end-point displaced from
  /// `start_spec`'s Endpoint via an arc.
  ///
  /// `arc_offset` specifies the shape of the arc. `end_spec` specifies the
  /// elevation characteristics at the end-point.
  /// `r0` is the distance from the reference curve to the first Lane
  /// centerline. `left_shoulder` and `right_shoulder` are extra lateral
  /// distances added to the extents of the Segment after the first and last
  /// Lanes positions are determined.
  /// `lane_layout` defines the number of lanes, their width, extra shoulder
  /// asphalt extensions and placing with respect to connection's reference
  /// curve.
  const Connection* Connect(const std::string& id,
                            const LaneLayout& lane_layout,
                            const StartReferenceSpec& start_spec,
                            const ArcOffset& arc_offset,
                            const EndReferenceSpec& end_spec);

  /// Sets the default branch for one end of a connection.
  ///
  /// The default branch for the `in_end` of connection `in` at Lane
  /// `in_lane_index`will set to be `out_end` of connection `out` at Lane
  /// `out_lane_index`. The specified connections must actually be joined at the
  /// specified ends (i.e., the Endpoint's for those ends must be coincident and
  /// (anti)parallel within the tolerances for the Builder).
  void SetDefaultBranch(const Connection* in, int in_lane_index,
                        const api::LaneEnd::Which in_end, const Connection* out,
                        int out_lane_index, const api::LaneEnd::Which out_end);

  /// Creates a new empty connection group with ID string `id`.
  Group* MakeGroup(const std::string& id);

  /// Creates a new connection group with ID `id`, populated with the
  /// given `connections`.
  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections);

  /// Produces a RoadGeometry, with the ID `id`.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const;

 private:
  // EndpointFuzzyOrder is an arbitrary strict complete ordering of Endpoints
  // useful for, e.g., std::map.  It provides a comparison operation that
  // treats two Endpoints within `linear_tolerance` of one another as
  // equivalent.
  //
  // This is used to match up the endpoints of Connections, to determine
  // how Connections are linked to one another.  Exact numeric equality
  // would not be robust given the use of floating-point values in Endpoints.
  class EndpointFuzzyOrder {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointFuzzyOrder)

    explicit EndpointFuzzyOrder(const double linear_tolerance)
        : lin_tol_(linear_tolerance) {}

    bool operator()(const Endpoint& lhs, const Endpoint& rhs) const {
      switch (fuzzy_compare(rhs.xy().x(), lhs.xy().x())) {
        case -1: { return true; }
        case 1: { return false; }
        case 0: {
          switch (fuzzy_compare(rhs.xy().y(), lhs.xy().y())) {
            case -1: { return true; }
            case 1: { return false; }
            case 0: {
              switch (fuzzy_compare(rhs.z().z(), lhs.z().z())) {
                case -1: { return true; }
                case 1: { return false; }
                case 0: { return false; }
                default: { DRAKE_ABORT(); }
              }
            }
            default: { DRAKE_ABORT(); }
          }
        }
        default: { DRAKE_ABORT(); }
      }
    }

   private:
    int fuzzy_compare(const double a, const double b) const {
      if (a < (b - lin_tol_)) {
        return -1;
      } else if (a > (b + lin_tol_)) {
        return 1;
      } else {
        return 0;
      }
    }

    double lin_tol_{};
  };

  struct DefaultBranch {
    DefaultBranch() = default;

    DefaultBranch(const Connection* ain, int ain_lane_index,
                  const api::LaneEnd::Which ain_end, const Connection* aout,
                  int aout_lane_index, const api::LaneEnd::Which aout_end)
        : in(ain),
          in_lane_index(ain_lane_index),
          in_end(ain_end),
          out(aout),
          out_lane_index(aout_lane_index),
          out_end(aout_end) {}

    const Connection* in{};
    const int in_lane_index{};
    api::LaneEnd::Which in_end{};
    const Connection* out{};
    const int out_lane_index{};
    api::LaneEnd::Which out_end{};
  };

  std::vector<Lane*> BuildConnection(
      const Connection* const cnx, Junction* const junction,
      RoadGeometry* const rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  BranchPoint* FindOrCreateBranchPoint(
      const Endpoint& point,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  void AttachBranchPoint(
      const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const;

  const api::HBounds elevation_bounds_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::vector<std::unique_ptr<Group>> groups_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
