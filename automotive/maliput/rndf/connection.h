#pragma once

#include <string>
#include <vector>

#include "ignition/math/Vector3.hh"
#include "ignition/rndf/UniqueId.hh"

#include "drake/automotive/maliput/rndf/directed_waypoint.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

/// A container that holds the information needed by a Builder to construct one
/// or more Lane objects. This class will be created from a vector of
/// DirectedWaypoint objects (based on RNDF waypoints that belong to the same
/// RNDF lane).
///
/// A Connection object derives one or multiple Lane objects. Consequently the
/// list of waypoints must have at least two valid (related to the
/// DirectedWaypoint's ignition::rndf::UniqueId) waypoints to derive an
/// interpolated geometry from them. The order the waypoints are set is the
/// way they will be used so create an interpolated curve (base line of the
/// Lane). Lane's width is fixed along the complete path length.
/// Builder class will receive a group of Connections that belong to the same
/// RNDF segment. Note that RNDF, unlike Maliput, has no constraint on lane
/// direction inside a segment, so Connections should be marked (using
/// the constructor's inverse_direction parameter) so they can be grouped by
/// sense of direction. To define the direction, one of the Connections in the
/// group will be flagged as the reference, and an interpolated spline is
/// computed from the the DirectedWaypoints within the reference Connection to
/// obtain the tangent information.
class Connection {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Connection)
  /// Constructs a Connection.
  /// @param id Connection's ID.
  /// @param waypoints A DirectedWaypoints vector that determines the lane path.
  /// @param width The Lane's width.
  /// @param inverse_direction False if this Connection belongs to a group of
  /// Connections and the sense of direction of this Connection is the same as
  /// the first Connection in the group. The first Connection will be set False
  /// as default.
  /// @throws std::runtime_error When the number of @p waypoints items, whose
  /// ignition::rndf::UniqueId is valid, is less than 2.
  /// @throws std::runtime_error When the @p width is less or equal to 0.0.
  Connection(const std::string& id,
             const std::vector<DirectedWaypoint>& waypoints, double width,
             bool inverse_direction)
      : id_(id),
        waypoints_(waypoints),
        width_(width),
        inverse_direction_(inverse_direction) {
    ThrowIfInvalidWaypoints(waypoints);
    DRAKE_THROW_UNLESS(width > 0.0);
  }

  /// Getter of the ID.
  /// @return The ID.
  const std::string& id() const { return id_; }

  /// Getter of the first item in the vector returned by waypoints().
  /// @return A constant reference to the first item in the vector returned by
  /// waypoints().
  const DirectedWaypoint& start() const { return waypoints_.front(); }

  /// Getter of the last item in the vector returned by waypoints().
  /// @return A constant reference to the last item in the vector returned by
  /// waypoints().
  const DirectedWaypoint& end() const { return waypoints_.back(); }

  /// Getter of the DirectedWaypoint vector.
  /// @return The DirectedWaypoint vector.
  const std::vector<DirectedWaypoint>& waypoints() const { return waypoints_; }

  /// Setter of the DirectedWaypoint vector.
  /// @param waypoints A vector of DirectedWaypoints.
  /// @throws std::runtime_error When @p waypoints holds less than two valid
  /// DirectedWaypoints.
  void set_waypoints(const std::vector<DirectedWaypoint>& waypoints) {
    ThrowIfInvalidWaypoints(waypoints);
    waypoints_ = waypoints;
  }

  /// Adds a waypoint to the DirectedWaypoint vector at @p position index.
  /// @param waypoint A DirectedWaypoint to add.
  /// @param position The index in the DirectedWaypoint vector at which
  /// @p waypoint should be added.
  /// @throws std::runtime_error When position is less than 0.
  /// @throws std::runtime_error When position is greater than the size of the
  /// DirectedWaypoint vector.
  void AddWaypoint(const DirectedWaypoint& waypoint, int position);

  /// Getter for the width.
  /// @return The width.
  double width() const { return width_; }

  /// Setter for the width.
  /// @param width The absolute span from one lateral extent to the other of the
  /// api::Lane that the Builder will make from this Connection.
  /// @throws std::runtime_error When @p width is less or equal to zero.
  void set_width(double width) {
    DRAKE_THROW_UNLESS(width > 0.0);
    width_ = width;
  }

  /// Getter of inverse_direction.
  /// @return true when this Connection has inverse direction relative to the
  /// base Lane in the RNDF segment to which this Connection belongs, and false
  /// otherwise.
  bool inverse_direction() const { return inverse_direction_; }

  /// Setter of inverse_direction.
  /// @brief inverse_direction The relative direction regarding another
  /// Connection which will be considered as the reference direction for the
  /// RNDF segment.
  void set_inverse_direction(bool inverse_direction) {
    inverse_direction_ = inverse_direction;
  }

 private:
  // Throws an exception if @p waypoints doesn't hold at least two valid
  // waypoints.
  // @param waypoints A vector of DirectedWaypoints to check.
  // @throws std::runtime_error When @p waypoints holds less than two valid
  // DirectedWaypoints.
  void ThrowIfInvalidWaypoints(
      const std::vector<DirectedWaypoint>& waypoints) const {
    int num_valid = 0;
    for (const DirectedWaypoint& wp : waypoints) {
      if (wp.id().Valid()) {
        num_valid++;
        if (num_valid == 2) {
          return;
        }
      }
    }
    throw std::runtime_error("Insufficient valid waypoints.");
  }

  // TODO(@agalbachicar) Uniqueness of the id_, when it is valid, should be
  // verified and guaranteed.
  // The ID.
  std::string id_;
  // A vector that holds the sequence of the DirectedWaypoints.
  std::vector<DirectedWaypoint> waypoints_;
  // The width of the connection.
  double width_{};
  // A boolean that tells if this connection has an inverse direction relative
  // to another connection in the same RNDF segment and that
  // represents the first lane in the segment.
  bool inverse_direction_{false};
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
