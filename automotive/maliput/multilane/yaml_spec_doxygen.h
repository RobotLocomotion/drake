#pragma once

namespace drake {
namespace maliput {
namespace multilane {

/// @file
/// <h1>Multilane YAML Format Specification</h1>
///
///
/// <h2>Introduction</h2>
///
/// Multilane is a backend implementation of Maliput, an interface to describe
/// road geometries. Multilane provides two loader methods
/// ( Load() and LoadFile() ) that will parse a YAML file or string by
/// calling appropriate `Builder` methods to create a `RoadGeometry`. So as
/// to reduce the impact with a new base format, YAML was chosen taking
/// Monolane's decision on that format before as well there is no need to
/// deprecate it.
///
/// <h2>General considerations</h2>
///
/// All the map information must be under a root node called
/// `maliput_multilane_builder`, otherwise it will not be parsed.
///
/// <h3>Units</h3>
///
/// The following list shows the available units found in different entities:
///
///   - Positions, distances and lengths: meters [m].
///   - Angles: degrees [°] (no minutes nor seconds, just degrees).
///   - Derivatives of positions: meters per meter [m/m].
///   - Derivatives of angles: degrees per meter [°/m].
///
/// All positions, distances, lengths, angles and derivatives are floating point
/// numbers. Bare quantities are integers.
///
/// <h3>Coordinates and frames</h3>
///
/// For points in space, a right handed, orthonormal and inertial ℝ³ frame is
/// used. Its basis is \f$(\hat{x}, \hat{y}, \hat{z})\f$ , where \f$\hat{x}\f$,
/// \f$\hat{y}\f$ are coplanar with the ground and \f$\hat{z}\f$ points upwards,
/// and positions are expressed as \f$(x, y, z)\f$ triples. Also, the \f$Θ\f$
/// angle rotating around the \f$\hat{z}\f$ axis is used to define headings.
/// These rotations are right handed and an angle of 0° points in the
/// \f$\hat{x}\f$ direction. Angles wrt a plane parallel to \f$z = 0\f$ can be
/// defined. A heading vector pointing the direction of the lane at that point
/// is used as rotation axis and the angle is clockwise. Those will express
/// superelevation.
///
/// <h2>Entities</h2>
///
/// <h3>Maliput Multilane Builder</h3>
///
/// `maliput_multilane_builder` holds all the common and default configurations
/// to build a `RoadGeometry`. All of them, except `groups`, must be defined
/// though some of them may be empty. Its attributes are:
///
/// It will be represented as a mapping:
///
/// @code
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
///   - _ID_ is a string scalar that defines the _ID_ of the `RoadGeometry`.
///   - _LW_ is the width of the lanes. Lane’s centerline will be placed at
/// the same lane width distance one from the other. It must be non negative.
///   - _LS_ and _RS_ are default left and right shoulders are extra spaces
/// added to the right of the last lane and left to the first lane respectively.
///  Their purpose is to increase driveable bounds. Both must be non negative.
///   - _EB\_MIN_ and _EB\_MAX_ define minimum and maximum height values of the
/// road’s volume. The minimum value comes first and must be non positive, then
/// the maximum and must be non negative.
///   - _LT_ and _AT_ are position and orientation tolerances which are non
/// negative numbers that define the error of mapping a world coordinate or
/// orientation in a custom lane-frame.
///   - _SL_ is the minimum spatial period of variation in `connections`'
/// reference curve.
///   - A _CP_ label, which could either be `prefer-accuracy` or `prefer-speed`
/// The former guides the computations to be as accurate as precision states.
/// The latter will be accurate whenever possible, but it's not guaranteed in
/// favor of faster computations.
///   - _points_ is a map of `Enpoints` to build `connections` are provided
/// under the `points` mapping. If it is empty, no `connection` would be
/// defined.
///   - _connections_ is a map that holds all the `Connection` definitions. It
/// may be empty if no `Connection` is going to be defined.
///   - _groups_ is a map of `groups` where `connections` can be put together.
/// It may be empty or not defined if no group is going to be made.
///
/// <h3>`EndpointXy`</h3>
///
/// A point in the plane \f$z = 0\f$ expressed as \f$(x, y, Θ)\f$, where
/// \f$(x, y)\f$ defines the position and \f$Θ\f$ defines the heading angle of
/// the endpoint. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code
///
/// endpoint_xy: [x, y, theta]
///
/// @endcode
///
/// Where:
///   - `x` is the \f$x\f$ coordinate.
///   - `y` is the \f$y\f$ coordinate.
///   - `theta` is the \f$Θ\f$ coordinate.
///
/// <h3>`EndpointXy`</h3>
///
/// Specifies elevation, slope, superelevation and its speed of change at a
/// point over the plane \f$z = 0\f$ as \f$(z, z', Θ, Θ')\f$, where \f$z\f$ and
/// \f$Θ'\f$ are the elevation and superelevation of the road at the endpoint
/// and \f$z'\f$ and \f$Θ'\f$ are their respective derivatives with respect to
/// an arc-length coordinate \f$t\f$ that travels along curve’s projection over
/// the \f$z = 0\f$ plane. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code
///
/// endpoint_z: [z, z_dot, theta, theta_dot]
///
/// @endcode
///
/// Where:
///   - `z` is the \f$z\f$ coordinate.
///   - `z_dot` is the \f$z′\f$ coordinate.
///   - `theta` is the \f$Θ\f$ coordinate.
///   - `theta_dot` is the \f$Θ′\f$ coordinate. This parameter is optional.
/// The `Builder` may need to adjust it to comply with _G1_ constraints.
///
/// <h3>`Endpoint`</h3>
///
/// It is a point in 3D space defined by an `EndpointXy` and a `EndpointZ`
/// Both sequences must be provided.
///
/// It will be represented as a mapping like:
///
/// @code
///
/// endpoint:
///   xypoint: [x, y, theta]
///   zpoint: [z, z_dot, theta, theta_dot]
///
/// @endcode
///
/// Where:
///   - `xypoint` is the _EndpointXy_ sequence.
///   - `zpoint` is the `EnpointZ` sequence.
///
/// <h3>Arc</h3>
///
/// Constant radius arcs are defined in terms of a radius and an angle span.
/// _Arcs_ are used to define planar curves on the the \f$z = 0\f$ plane,
/// starting from an `EndpointXy`. Radius must be positive and its direction
/// is derived as rotating +90° start `EndpointXy`’s heading vector, and
/// angle span has the same properties as `EndpoinXy`’s `theta` coordinate.
///
/// It will be represented as a sequence:
///
/// @code
///
/// arc: [radius, theta]
///
/// @endcode
///
/// Where:
///   - `radius` is the radius of the `arc`.
///   - `theta` is the angle span of the `arc`.
///
/// <h3>`Connection`</h3>
///
/// A `Connection` defines a `Segment` and must provide the number of lanes,
/// start `Endpoint` and end `EndpointZ` information. Either line `length` or
/// `arc` must be provided to define the planar geometry of that connection.
/// Optional extra information can also provided and it will modify the way the
/// `Connection` will be created.
///
/// `start` `Endpoint` and `end` `EndpointZ` can either refer to a reference
/// road curve or to the lane start and end `Endpoints`. When only `start` and
/// `z_end` or `explicit_end` are provided, those will refer to the reference
/// road curve of the `Connection`. `r_ref` can be provided to state the offset
/// distance from the reference road curve to `ref_lane` `connection`’s lane.
/// Lanes are 0-indexed. In addition, left and right shoulder distances can be
/// provided and those will override default values. `left_shoulder` and
/// `right_shoulder` must be bigger or equal to zero if provided.
///
/// Sample line-connections mapping are shown below:
///
///   - Example 1: reference curve from points.
///
/// @code
///
/// connection:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["ref", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["ref", [z, z_dot, theta, theta_dot]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["ref", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
///   - Example 2: reference curve from connections.
///
/// @code
///
/// connection:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["ref", "connections.CONN_NAME_1.(start|end).ref.(forward|reverse)"]
///   length: L
///   explicit_end: ["ref", "connections.CONN_NAME_2.(start|end).ref.(forward|reverse)"]
///   # The following can be used instead of explicit_end:
///   # z_end: ["ref", [z, z_dot, theta, theta_dot]]
///
/// @endcode
///
///   - Example 3: lane curve from points.
///
/// @code
///
/// connection:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["lane.LN_1", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["lane.LN_2", [z, z_dot, theta, theta_dot]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["lane.LN_2", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
///   - Example 4: lane curve from other connections' lane curves.
///
/// @code
///
/// connection:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["lane.LN_1", connections.CONN_NAME_1.(start|end).LN_2.(forward|reverse)"]
///   length: L
///   explicit_end: ["lane.LN_3", "connections.CONN_NAME_2.(start|end).LN_4.(forward|reverse)"]
///   # The following can be used instead of explicit_end:
///   # z_end: ["ref", [z, z_dot, theta, theta_dot]]
///
/// @endcode
///
/// Where:
///   - `lanes` holds number of lanes, reference lane and distance from the
/// reference lane to the reference curve.
///   - `left_shoulder` is the extra space at the right side of the last lane
/// of the connection. It will override default values.
///   - `right_shoulder` is the extra space at the left side of the first lane
/// of the `connection`. It will override default values.
///   - `start` is used to define the start `Endpoint` of one the `connection`’s
/// curves. It may have multiple options. Those can be split into two parts.
/// The first item could be:
///     -# `ref` to point the reference curve.
///     -# `lane.LN` to point a specific lane.
/// The second part is composed of one of the following options:
///     -# A reference to an `Endpoint` in the points collection. Either
/// `forward` or `reverse` should be used to indicate the direction of the
/// `Endpoint`.
///    -# The start or end `Endpoint` of a `connection`’s reference curve or
/// lane. Either forward or reverse should be used to indicate the direction of
/// the `Endpoint`.
/// When using the forward the `Endpoint` will be used as is. Otherwise
/// (using `reverse`) the `Endpoint` will be reversed.
///   - `length` is the `connection`’s reference road curve planar line length.
///   - `arc` is the `connection`’s reference curve planar piece of arc.
///   - `z_end` is the `EnpointZ` information to end one of the `connection`’s
/// curves. It is composed of two elements too. The first one points to the
/// reference curve when `ref` is present. Otherwise, `lane.LN` must be
/// specified.
///   - `explicit_end` is a node similar to `start`. It is composed of two
/// parts. The first one points to the reference curve when `ref` is present.
/// Otherwise, `lane.LN` must be specified. The second part is used to point the
/// `Endpoint` information which could be provided by a `connection` or the
/// `points` collection. When using a `connection`, two options are available:
/// the reference curve or a lane.
///
/// Possible combinations to define a `connection` node are:
///   - Each `connection` must have either `length` or `arc`.
///   - Each `connection` must have either `z_end` or `explicit_end`.
///   - `start` and `explicit_end` possible combinations:
///     -# This LANE from other LANE: This combination is essential; it's the
/// fundamental "hook up segments so that the lanes line up as intended"
/// specification.
///     -# This REF from other REF: I'm not sure how useful this actually is,
/// but the existing `Builder` should be able to execute it.
///     -# This REF from lane POINT: We need something that allows using a
/// free point as the origin, and the existing `Builder` should be able to
/// execute it.
///     -# This LANE from lane POINT: This is probably more useful than
/// "REF from POINT".
///
/// <h3>Group</h3>
///
/// A collection of touching `Connections`. It will be a sequence of string
/// scalars that refer to connections’ name identifiers.
///
/// It will be represented as a mapping:
///
/// @code
///
/// group: [c_1, c_2, c_3]
///
/// @endcode
///
/// Where:
///   - `c_1`, `c_2`, `c_3` are `connections`’ _ID_s.
///
/// <h2>General structure</h2>
///
/// Below you can see a snippet with the general YAML structure.
///
/// @code
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

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
