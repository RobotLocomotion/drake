#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <tuple>
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
  /// Spec's theta_dot will be cleared so the Builder adjusts it to match
  /// continuity constraints.
  Spec at(const Connection& connection, api::LaneEnd::Which end,
          Direction direction) const {
    Endpoint endpoint = end == api::LaneEnd::Which::kStart ? connection.start()
                                                           : connection.end();
    endpoint.get_mutable_z().get_mutable_theta_dot() = {};
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
  /// Spec's theta_dot will be cleared so the Builder adjusts it to match
  /// continuity constraints.
  Spec z_at(const Connection& connection, api::LaneEnd::Which end,
            Direction direction) const {
    EndpointZ endpoint_z = end == api::LaneEnd::Which::kStart
                               ? connection.start().z()
                               : connection.end().z();
    endpoint_z.get_mutable_theta_dot() = {};
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

/// Defines a builder interface for multilane. It is used for testing purposes
/// only, and derived code should instantiate Builder objects.
class BuilderBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BuilderBase)

  BuilderBase() = default;

  virtual ~BuilderBase() = default;

  /// Gets `lane_width` value.
  virtual double get_lane_width() const = 0;

  /// Gets `elevation_bounds` value.
  virtual const api::HBounds& get_elevation_bounds() const = 0;

  /// Gets `linear_tolerance` value.
  virtual double get_linear_tolerance() const = 0;

  /// Gets `angular_tolerance` value.
  virtual double get_angular_tolerance() const = 0;

  /// Gets `scale_length` value.
  virtual double get_scale_length() const = 0;

  /// Gets `computation_policy` value.
  virtual ComputationPolicy get_computation_policy() const = 0;

  /// Connects `start_spec`'s Endpoint to an end-point linearly displaced from
  /// `start_spec`'s Endpoint.
  ///
  /// `line_offset` specifies the length of displacement (in the direction of
  /// the heading of `start_spec`'s Endpoint). `end_spec` specifies the
  /// elevation characteristics at the end-point.
  /// `lane_layout` defines the number of lanes, their width, extra shoulder
  /// asphalt extensions and placing with respect to connection's reference
  /// curve.
  virtual const Connection* Connect(const std::string& id,
                                    const LaneLayout& lane_layout,
                                    const StartReference::Spec& start_spec,
                                    const LineOffset& line_offset,
                                    const EndReference::Spec& end_spec) = 0;

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
  virtual const Connection* Connect(const std::string& id,
                                    const LaneLayout& lane_layout,
                                    const StartReference::Spec& start_spec,
                                    const ArcOffset& arc_offset,
                                    const EndReference::Spec& end_spec) = 0;

  /// Sets the default branch for one end of a connection.
  ///
  /// The default branch for the `in_end` of connection `in` at Lane
  /// `in_lane_index`will set to be `out_end` of connection `out` at Lane
  /// `out_lane_index`. The specified connections must actually be joined at the
  /// specified ends (i.e., the Endpoint's for those ends must be coincident and
  /// (anti)parallel within the tolerances for the Builder).
  virtual void SetDefaultBranch(const Connection* in, int in_lane_index,
                                api::LaneEnd::Which in_end,
                                const Connection* out, int out_lane_index,
                                api::LaneEnd::Which out_end) = 0;

  /// Creates a new empty connection group with ID string `id`.
  virtual Group* MakeGroup(const std::string& id) = 0;

  /// Creates a new connection group with ID `id`, populated with the
  /// given `connections`.
  virtual Group* MakeGroup(
      const std::string& id,
      const std::vector<const Connection*>& connections) = 0;

  /// Produces a RoadGeometry, with the ID `id`.
  virtual std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const = 0;
};

/// Factory interface to construct BuilderBase instances.
///
/// Defined for testing purposes, and production code must use BuilderFactory
/// objects.
class BuilderFactoryBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BuilderFactoryBase)

  BuilderFactoryBase() = default;

  virtual ~BuilderFactoryBase() = default;

  /// Creates a BuilderBase instance.
  ///
  /// `lane_width`, `elevation_bounds`, `linear_tolerance`,
  /// `angular_tolerance`, `scale_length` and `computation_policy` are
  /// BuilderBase properties.
  virtual std::unique_ptr<BuilderBase> Make(
      double lane_width, const api::HBounds& elevation_bounds,
      double linear_tolerance, double angular_tolerance,
      double scale_length, ComputationPolicy computation_policy) const = 0;
};

/// Convenient builder class which makes it easy to construct a multilane road
/// network.
///
/// multilane is a simple road-network implementation:
///  - multiple lanes per segment;
///  - constant lane width, lane_bounds, and elevation_bounds, same for all
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
class Builder : public BuilderBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Builder)

  /// Constructs a Builder which can be used to specify and assemble a
  /// multilane implementation of an api::RoadGeometry.
  ///
  /// `lane_width` is the width assigned to all Lanes. It must be greater or
  /// equal to zero. Lane reference path (which are offsets of parent Segment
  /// reference curve) are centered within the Lane. Lane spacing will be
  /// `lane_width` too. Segment extents will be derived from the composition of
  /// left and right shoulders, number of lanes and lane spacing. The
  /// `elevation_bounds` is applied uniformly to all lanes of every segment.
  /// `linear_tolerance` and `angular_tolerance` specify the respective
  /// tolerances for the resulting RoadGeometry. `scale_length` constrains
  /// the maximum level of detail captured by the resulting RoadGeometry.
  /// `computation_policy` sets the speed vs. accuracy balance for computations.
  Builder(double lane_width, const api::HBounds& elevation_bounds,
          double linear_tolerance, double angular_tolerance,
          double scale_length, ComputationPolicy computation_policy);

  /// Gets `lane_width` value.
  double get_lane_width() const override { return lane_width_; }

  /// Gets `elevation_bounds` value.
  const api::HBounds& get_elevation_bounds() const override {
    return elevation_bounds_;
  }

  /// Gets `linear_tolerance` value.
  double get_linear_tolerance() const override { return linear_tolerance_; }

  /// Gets `angular_tolerance` value.
  double get_angular_tolerance() const override { return angular_tolerance_; }

  double get_scale_length() const override { return scale_length_; }

  ComputationPolicy get_computation_policy() const override {
    return computation_policy_;
  }

  const Connection* Connect(const std::string& id,
                            const LaneLayout& lane_layout,
                            const StartReference::Spec& start_spec,
                            const LineOffset& line_offset,
                            const EndReference::Spec& end_spec) override;

  const Connection* Connect(const std::string& id,
                            const LaneLayout& lane_layout,
                            const StartReference::Spec& start_spec,
                            const ArcOffset& arc_offset,
                            const EndReference::Spec& end_spec) override;

  void SetDefaultBranch(const Connection* in, int in_lane_index,
                        const api::LaneEnd::Which in_end, const Connection* out,
                        int out_lane_index,
                        const api::LaneEnd::Which out_end) override;

  Group* MakeGroup(const std::string& id) override;

  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections) override;

  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const override;

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

  double lane_width_{};
  api::HBounds elevation_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  ComputationPolicy computation_policy_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::vector<std::unique_ptr<Group>> groups_;
};

/// Implements a BuilderFactoryBase to construct Builder objects.
class BuilderFactory : public BuilderFactoryBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BuilderFactory)

  BuilderFactory() = default;

  std::unique_ptr<BuilderBase> Make(
      double lane_width, const api::HBounds& elevation_bounds,
      double linear_tolerance, double angular_tolerance, double scale_length,
      ComputationPolicy computation_policy) const override {
    return std::make_unique<Builder>(lane_width, elevation_bounds,
                                     linear_tolerance, angular_tolerance,
                                     scale_length, computation_policy);
  }
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
