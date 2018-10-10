#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace multilane {

/// Loads the `input` string as a maliput_multilane_builder document using the
/// provided `builder_factory`. See
/// @ref automotive/maliput/multilane/loader.h "loader.h" for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the @ref drake::maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> Load(
    const BuilderFactoryBase& builder_factory, const std::string& input);

/// Loads the named file as a maliput_multilane_builder document using the
/// provided `builder_factory`. See
/// @ref automotive/maliput/multilane/loader.h "loader.h" for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the @ref drake::maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> LoadFile(
    const BuilderFactoryBase& builder_factory, const std::string& filename);

/// @file
/// <h1>Multilane YAML Format Specification</h1>
///
///
/// <h2>Introduction</h2>
///
/// Multilane is a backend implementation of Maliput, an interface to describe
/// road geometries. Multilane provides two loader methods
/// ( @ref drake::maliput::multilane::Load() "Load()" and
/// @ref drake::maliput::multilane::LoadFile() "LoadFile()" ) that will parse a
/// YAML file or string by calling appropriate
/// @ref drake::maliput::multilane::Builder "Builder" methods to create a
/// `RoadGeometry`.
///
/// The serialization is a fairly straightforward mapping of the
/// @ref drake::maliput::multilane::Builder "Builder" interface onto YAML.
///
/// The basic idea is, however:
///
/// - general parameters (i.e., lane_width, elevation bounds, linear and
///   angular tolerance)
/// - a collection of named 'points', which are specifications of explicitly
///   named Endpoints
/// - a collection of named 'connections', whose start Endpoints are specified
///   by reference to either a named Endpoint or the start or end of
///   a named Connection
/// - a collection of named 'groups', specified by sequences of named
///   Connections
///
/// Parsing will fail if there is no way to concretely resolve all of the
/// Endpoint references, e.g., if a document specifies that Connection-A
/// is an arc starting at the end of Connection-B and that Connection-B
/// is an arc starting at the end of Connection-A.  All referential chains
/// must bottom out in explicitly-named Endpoints.
///
/// <h2>General considerations</h2>
///
/// All the road geometry information must be under a root node called
/// `maliput_multilane_builder`, otherwise it will not be parsed.
///
/// <h3>Units</h3>
///
/// The following list shows the expected units for floating-point quantities:
///
/// - Positions, distances and lengths: meters [m].
/// - Angles: degrees [°] (no minutes nor seconds, just degrees).
/// - Derivatives of positions: meters per meter [m/m] \(i.e., a unitless
///   slope).
/// - Derivatives of angles: degrees per meter [°/m].
///
/// All positions, distances, lengths, angles and derivatives are floating point
/// numbers. Other type of quantities will be integers.
///
/// <h3>Miscellaneous</h3>
///
/// Clarifications to better understand the nomenclature used within this
/// description:
///
/// - In code snippets, strings in capital letters represent values that the
///   YAML writer must choose and the others are keywords to be parsed.
/// - When referring to keywords in the YAML, `non-capitalized` strings will
///   be used.
/// - When referring to types within `maliput`, `Capitalized` strings will be
///   used.
///
/// <h3>Coordinates and frames</h3>
///
/// For points in space, a right handed, orthonormal and inertial ℝ³ frame is
/// used. Its basis is (x̂, ŷ, ẑ), where x̂, ŷ are coplanar with the ground and ẑ
/// points upwards, and positions are expressed as `(x, y, z)` triples.
/// Also, the Θ angle rotating around the ẑ axis is used to define
/// headings. These rotations are right handed and an angle of 0° points in
/// the x̂ direction. Angles with respect to a plane parallel to `z = 0` can
/// be defined. A heading vector pointing the direction of the lane at that
/// point is used as rotation axis and the angle is clockwise. Those will
/// express superelevation.
///
/// <h2>Example of General Structure</h2>
///
/// Below you can see a snippet with the general YAML structure.
///
/// @code{.yml}
///
/// maliput_multilane_builder:
///  id: "my_road_geometry"
///  lane_width: 3.2
///  left_shoulder: 1.25
///  right_shoulder: 2.47
///  elevation_bounds: [0., 7.6]
///  scale_length: 1.
///  linear_tolerance: 0.1
///  angular_tolerance: 0.1
///  computation_policy: prefer-accuracy
///  points:
///    point_a:
///      xypoint: [0, 0, 0]
///      zpoint: [0, 0, 0, 0]
///    point_b:
///      xypoint: [50, 5, 0]
///      zpoint: [0, 0, 0]
///    ...
///  connections:
///    conn_a:
///       left_shoulder: 1.3 # Optional
///       lanes: [3, 2, -5.3]
///       start: ["lane.1", "points.point_a.forward"]
///       arc: [30.25, -45]
///       z_end: ["lane.0", [0, 3, 30, 3.1]]
///    conn_b:
///    ...
///  groups:
///    group_A: [conn_a, conn_b]
///
/// @endcode
///
/// <h2>Entities</h2>
///
/// <h3>`maliput_multilane_builder`</h3>
///
/// `maliput_multilane_builder` holds all the common and default configurations
/// to build a `RoadGeometry`. All of them, except `groups`, must be defined
/// though some of them may be empty.
///
/// It will be represented as a mapping:
///
/// @code{.yml}
///
/// maliput_multilane_builder:
///  id: "ID"
///  lane_width: LW
///  left_shoulder: LS
///  right_shoulder: RS
///  elevation_bounds: [EB_MIN, EB_MAX]
///  linear_tolerance: LT
///  angular_tolerance: AT
///  scale_length: SL
///  computation_policy: CP
///  points:
///    ...
///  connections:
///    ...
///  groups:
///    ...
///
/// @endcode
///
/// Where:
///
/// - _ID_ is a string scalar that defines the _ID_ of the `RoadGeometry`.
/// - _LW_ is the width of the lanes. Lane’s centerline will be placed at
///   the same lane width distance one from the other. It must be non negative.
/// - _LS_ and _RS_ are default left and right shoulders are extra spaces
///   added to the right of the last lane and left to the first lane
///   respectively.
///   Their purpose is to increase driveable bounds. Both must be non negative.
/// - _EB\_MIN_ and _EB\_MAX_ define minimum and maximum height values of the
///   road’s volume. The minimum value must be non positive, thus the maximum
///   must be non negative.
/// - _LT_ and _AT_ are position and orientation tolerances which are non
///   negative numbers that define the error of mapping a world coordinate or
///   orientation in a custom lane-frame.
/// - _SL_ is the minimum spatial period of variation in `connections`'
///   reference curve.
/// - A _CP_ label, which could either be `prefer-accuracy` or `prefer-speed`
///   The former guides the computations to be as accurate as precision states.
///   The latter will be accurate whenever possible, but it's not guaranteed in
///   favor of faster computations.
/// - _points_ is a map of `endpoint`s to build `connection`s. At least one
///   point is required to anchor the connections to world-frame.
/// - _connections_ is a map that holds all the `connection` definitions. It
///   may be empty if no `Connection` is going to be defined.
/// - _groups_ is a map of `groups` where `connections` can be put together.
///   It may be empty or not defined if no group is going to be made.
///
///
/// <h3>`points`</h3>
///
/// A collection of points in 3D space. Each one will be under a tag (used to
/// reference it within `connection` description) and defined by an
/// `endpoint_xy` and a `endpoint_z`. Both sequences must be provided.
///
/// It will be represented as a mapping like:
///
/// @code{.yml}
///
/// endpoint:
///   xypoint: [X, Y, THETHA]
///   zpoint: [Z, Z_DOT, THETA, THETA_DOT]
///
/// @endcode
///
/// Where:
///
/// - `xypoint` is the `endpoint_xy` sequence.
/// - `zpoint` is the `endpoint_z` sequence.
///
/// <h3>`endpoint_xy`</h3>
///
/// A point in the plane `z = 0` expressed as `(x, y, Θ)`, where
/// `(x, y)` defines the position and Θ defines the heading angle of
/// the endpoint. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// endpoint_xy: [X, Y, THETA]
///
/// @endcode
///
/// Where:
///
/// - _X_ is the `x` coordinate.
/// - _Y_ is the `y` coordinate.
/// - _THETA_ is the Θ coordinate.
///
/// <h3>`endpoint_z`</h3>
///
/// Specifies elevation, slope, superelevation and its speed of change at a
/// point over the plane `z = 0` as `(z, z', Θ, Θ')`, where `z` and
/// Θ' are the elevation and superelevation of the road at the endpoint
/// and `z'` and Θ' are their respective derivatives with respect to
/// an arc-length coordinate `t` that travels along curve’s projection over
/// the `z = 0` plane. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// endpoint_z: [Z, Z_DOT, THETA, THETA_DOT]
///
/// @endcode
///
/// Where:
///
/// - _Z_ is the `z` coordinate.
/// - _Z\_DOT_ is the `z′` coordinate.
/// - _THETA_ is the Θ coordinate.
/// - _THETA\_DOT_ is the Θ′ coordinate. This parameter is optional, and
///   typically should be omitted. When omitted, this value will be
///   automatically calculated such that _G1_ continuity of the road surface is
///   preserved.
///
/// <h3>`connections`</h3>
///
/// A `connection` defines a `Segment` and must provide the number of lanes,
/// start `endpoint` and end `endpoint_z` information. Either line `length` or
/// `arc` must be provided to define the planar geometry of that `connection`.
/// Optional extra information can also provided and it will modify the way the
/// `connection` will be created. `connections` is a collection of `connection`s
/// and those will be identified by their tag. Each tag will name a
/// `connection`, can be referenced by other `connection`s and to create
/// `group`s, and will be used as `Segment`'s ID as well.
///
/// `start` `endpoint` and `end` `endpoint_z` can either refer to a reference
/// road curve or to the lane start and end `Endpoint`s. When only `start` and
/// `z_end` or `explicit_end` are provided, those will refer to the reference
/// road curve of the `Connection`. `r_ref` can be provided to state the offset
/// distance from the reference road curve to `ref_lane` `connection`’s lane.
/// Lanes are 0-indexed. In addition, left and right shoulder distances can be
/// provided and those will override default values. `left_shoulder` and
/// `right_shoulder` must be bigger or equal to zero if provided.
///
/// Sample line-connections mapping are shown below:
///
/// - Example 1: reference curve from points.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["ref", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["ref", [Z, Z_DOT, THETA, THETA_DOT]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["ref", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
/// Within `z_end`, _THETA\_DOT_ is optional, and typically should be omitted.
/// When omitted, this value will be automatically calculated such that _G1_
/// continuity of the road surface is preserved. Otherwise, provided
/// _THETA\_DOT_ will be used and the
/// @ref drake::maliput::multilane::Builder "Builder" will check whether or not
///  _G1_ is preserved.
///
/// When `explicit_end` is used, _THETA\_DOT_ will be set by
/// @ref drake::maliput::multilane::Builder "Builder" to preserve _G1_ road
/// surface continuity.
///
/// - Example 2: reference curve from connections.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: [
///     "ref",
///     "connections.CONN_NAME_1.(start|end).ref.(forward|reverse)"
///   ]
///   length: L
///   explicit_end: [
///     "ref",
///     "connections.CONN_NAME_2.(start|end).ref.(forward|reverse)"
///   ]
///   # The following can be used instead of explicit_end:
///   # z_end: ["ref", [Z, Z_DOT, THETA, THETA_DOT]]
///
/// @endcode
///
/// Within `z_end`, _THETA\_DOT_ is optional, and typically should be omitted.
/// When omitted, this value will be automatically calculated such that _G1_
/// continuity of the road surface is preserved. Otherwise, provided
/// _THETA\_DOT_ will be used and the
/// @ref drake::maliput::multilane::Builder "Builder" will check whether or not
/// _G1_ is preserved.
///
/// When `explicit_end` is used, _THETA\_DOT_ will be set by
/// @ref drake::maliput::multilane::Builder "Builder" to preserve _G1_
/// continuity of the road surface.
///
/// - Example 3: lane curve from points.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["lane.LN_1", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["lane.LN_2", [Z, Z_DOT, THETA]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["lane.LN_2", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
/// None of the lane-based flavors allow to have _THETA\_DOT_ at either
/// `start` or `z_end`. @ref drake::maliput::multilane::Builder "Builder" will
/// adjust them to preserve _G1_ continuity of the road surface.
///
/// - Example 4: lane curve from other connections' lane curves.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: [
///     "lane.LN_1",
///     "connections.CONN_NAME_1.(start|end).LN_2.(forward|reverse)"
///   ]
///   length: L
///   explicit_end: [
///     "lane.LN_2",
///     "connections.CONN_NAME_2.(start|end).LN_4.(forward|reverse)"
///   ]
///   # The following can be used instead of explicit_end:
///   # z_end: ["lane.LN_2", [Z, Z_DOT, THETA]]
///
/// @endcode
///
/// None of the lane-based flavors allow to have _THETA\_DOT_ at either
/// `start` or `z_end`. @ref drake::maliput::multilane::Builder "Builder" will
/// adjust them to preserve _G1_ continuity of the road surface.
///
/// From examples above:
///
/// - `lanes` holds number of lanes, reference lane and distance from the
///   reference lane to the reference curve.
/// - `left_shoulder` is the extra space at the right side of the last lane
///   of the connection. It will override default values.
/// - `right_shoulder` is the extra space at the left side of the first lane
///   of the `connection`. It will override default values.
/// - `start` is used to define the start `endpoint` of one the `connection`’s
///   curves. It may have multiple options. Those can be split into two
///   elements:
///   - The first element could be:
///     -# `ref` to point the reference curve.
///     -# `lane.LN` to point a specific lane.\n
///   - The second element is composed of one of the following options:
///     -# A reference to an `endpoint` in the points collection. Either
///        `forward` or `reverse` should be used to indicate the direction of
///        the `endpoint`.
///     -# The start or end `endpoint` of a `connection`’s reference curve or
///        lane. Either forward or reverse should be used to indicate the
///        direction of the `endpoint`. When using the forward the `endpoint`
///        will be used as is. Otherwise (using `reverse`) the `Endpoint` will
///        be reversed.
/// - `length` is the `connection`’s reference road curve planar line length.
/// - `arc` is the `connection`’s reference curve planar piece of arc.
/// - `z_end` is the `endpoint_z` information to end one of the `connection`’s
///    curves. It is composed of two elements too. The first one points to the
///    reference curve when `ref` is present. Otherwise, `lane.LN` must be
///    specified.
/// - `explicit_end` is a node similar to `start`. It is composed of two
///    parts. The first one points to the reference curve when `ref` is present.
///    Otherwise, `lane.LN` must be specified. The second part is used to point
///    the `endpoint` information which could be provided by a `connection` or
///    the `points` collection. When using a `connection`, two options are
///    available: the reference curve or a lane.
///
/// Possible combinations to define a `connection` node are:
///
/// - Each `connection` must have either `length` or `arc`.
/// - Each `connection` must have either `z_end` or `explicit_end`.
/// - `start` and `explicit_end` possible combinations:
///   -# This LANE from other LANE.
///   -# This REF from other REF.
///   -# This LANE from POINT.
///   -# This REF from POINT.
///
/// At least one connection must _start_ with "LANE from POINT" or "REF from
/// POINT" in order to anchor the road geometry in the world frame.
///
/// <h3>`arc`</h3>
///
/// Constant radius arcs are defined in terms of a radius and an angle span.
/// `arc`s are used to define planar curves on the the `z = 0` plane,
/// starting from an `endpoint_xy`. Radius must be positive, and arc's center
/// would be to the left (i.e. rotating +90° start `endpoint_xy`’s heading
/// vector) when theta is positive, otherwise it would be to the right (i.e.
/// rotating -90° start `endpoint_xy`’s heading).
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// arc: [RADIUS, THETA]
///
/// @endcode
///
/// Where:
///
/// - _RADIUS_ is the radius of the `arc`.
/// - _THETA_ is the angle span of the `arc`.
///
/// <h3>`groups`</h3>
///
/// A group specifies a set of connections whose Segments will be placed
/// together in the same `Junction`. A `connection` may only belong to a single
/// `group`. If a `connection` is not in any `group`, its `Segment` will receive
/// its own `Junction`.
///
/// It will be represented as a mapping:
///
/// @code{.yml}
///
/// GROUP_NAME: [C_1, C_2, C_3]
///
/// @endcode
///
/// Where:
///
/// - `C\_1`, `C\_2`, `C\_3` are `connections`’ IDs.

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
