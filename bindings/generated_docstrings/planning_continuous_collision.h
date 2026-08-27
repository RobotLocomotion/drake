#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/planning/continuous_collision/bounding_sphere.h"
// #include "drake/planning/continuous_collision/certificate.h"
// #include "drake/planning/continuous_collision/continuous_collision_checker.h"
// #include "drake/planning/continuous_collision/distance_oracle.h"
// #include "drake/planning/continuous_collision/motion_bound_table.h"
// #include "drake/planning/continuous_collision/numerics.h"
// #include "drake/planning/continuous_collision/options.h"
// #include "drake/planning/continuous_collision/piecewise_bezier_path.h"
// #include "drake/planning/continuous_collision/vpolytope_ingestion.h"

// Symbol: pydrake_doc_planning_continuous_collision
constexpr struct /* pydrake_doc_planning_continuous_collision */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::continuous_collision
      struct /* continuous_collision */ {
        // Symbol: drake::planning::continuous_collision::AddVPolytopeObstacle
        struct /* AddVPolytopeObstacle */ {
          // Source: drake/planning/continuous_collision/vpolytope_ingestion.h
          const char* doc =
R"""(Registers a V-polytope as an anchored obstacle with a collision role
(the geometry-support scope, "V-polytopes as first-class geometry",
ingestion route (b)).

The polytope is converted to ``drake∷geometry∷Convex`` through Drake's
own ``VPolytope∷ToShapeConvex()`` entry point (a thin wrapper over the
``Convex(Eigen∷Matrix3X<double> points, std∷string label, double
scale)`` constructor pinned at M0), then registered on the plant's
world body. The result therefore rides the ordinary native narrowphase
path end to end: the proximity engine and the certifier's
radius/support code all read the same ``Convex∷GetConvexHull()``
object, so the certificate stays sound even for redundant or
degenerate vertex sets.

Parameter ``plant``:
    The plant to register on. Must be non-null, must already be a
    registered SceneGraph source, and must NOT be finalized.

Parameter ``vpoly``:
    The polytope. Its vertices are interpreted in the geometry frame
    G, i.e. the world-frame obstacle is ``X_WG *
    conv(vpoly.vertices())``. Must be 3-dimensional with at least one
    vertex.

Parameter ``X_WG``:
    Pose of the geometry frame in the world frame.

Parameter ``name``:
    Geometry name; also used as the ``Convex`` shape's label (which
    Drake only uses in its own warning/error messages). Must not
    contain a newline.

Returns:
    The id of the newly registered collision geometry.

Raises:
    RuntimeError if ``plant`` is null or already finalized, if
    ``vpoly.ambient_dimension() != 3``, if the vertex set is empty, or
    if Drake rejects the resulting hull (e.g. a degenerate vertex set
    that its hull computation cannot inflate).)""";
        } AddVPolytopeObstacle;
        // Symbol: drake::planning::continuous_collision::BezierSegment
        struct /* BezierSegment */ {
          // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
          const char* doc =
R"""(One Bézier segment q(s) = Σ_j B_{j,m}(s) P_j, s ∈ [0, 1] (trajectory
normalization).)""";
          // Symbol: drake::planning::continuous_collision::BezierSegment::control_points
          struct /* control_points */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(n × (m+1); column j is control point P_j.)""";
          } control_points;
          // Symbol: drake::planning::continuous_collision::BezierSegment::t_end
          struct /* t_end */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } t_end;
          // Symbol: drake::planning::continuous_collision::BezierSegment::t_start
          struct /* t_start */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Original time interval (bookkeeping only; the certificate is a
property of the path and is invariant under time reparametrization).)""";
          } t_start;
        } BezierSegment;
        // Symbol: drake::planning::continuous_collision::BoundingSphere
        struct /* BoundingSphere */ {
          // Source: drake/planning/continuous_collision/bounding_sphere.h
          const char* doc =
R"""(A sphere, expressed in the owning body (link) frame L, that contains a
proximity geometry at every configuration of the body.)""";
          // Symbol: drake::planning::continuous_collision::BoundingSphere::center_L
          struct /* center_L */ {
            // Source: drake/planning/continuous_collision/bounding_sphere.h
            const char* doc = R"""(Sphere center in the body frame.)""";
          } center_L;
          // Symbol: drake::planning::continuous_collision::BoundingSphere::radius
          struct /* radius */ {
            // Source: drake/planning/continuous_collision/bounding_sphere.h
            const char* doc = R"""()""";
          } radius;
        } BoundingSphere;
        // Symbol: drake::planning::continuous_collision::Certificate
        struct /* Certificate */ {
          // Source: drake/planning/continuous_collision/certificate.h
          const char* doc =
R"""(Audit trail of every certification event of a run; an independent
replay (VerifyCertificate, declared in the api header) re-evaluates
every record and checks interval coverage of the full domain per pair.)""";
          // Symbol: drake::planning::continuous_collision::Certificate::pairs
          struct /* pairs */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc =
R"""(Pair table snapshot the indices refer to.)""";
          } pairs;
          // Symbol: drake::planning::continuous_collision::Certificate::records
          struct /* records */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } records;
        } Certificate;
        // Symbol: drake::planning::continuous_collision::CertificateRecord
        struct /* CertificateRecord */ {
          // Source: drake/planning/continuous_collision/certificate.h
          const char* doc =
R"""(One certification event: pair ``pair_index`` was certified over the
parameter interval [s_start, s_end] of segment ``segment`` from
representative configuration qc (the search algorithm).)""";
          // Symbol: drake::planning::continuous_collision::CertificateRecord::motion_bound
          struct /* motion_bound */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } motion_bound;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::pair_index
          struct /* pair_index */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } pair_index;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::phi_hat
          struct /* phi_hat */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } phi_hat;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::qc
          struct /* qc */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } qc;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::s_end
          struct /* s_end */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } s_end;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::s_start
          struct /* s_start */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } s_start;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::segment
          struct /* segment */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } segment;
          // Symbol: drake::planning::continuous_collision::CertificateRecord::threshold
          struct /* threshold */ {
            // Source: drake/planning/continuous_collision/certificate.h
            const char* doc = R"""()""";
          } threshold;
        } CertificateRecord;
        // Symbol: drake::planning::continuous_collision::CertificationResult
        struct /* CertificationResult */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc =
R"""(Result of one certification call (the architecture).)""";
          // Symbol: drake::planning::continuous_collision::CertificationResult::certificate
          struct /* certificate */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""(Present iff Options∷emit_certificate.)""";
          } certificate;
          // Symbol: drake::planning::continuous_collision::CertificationResult::findings
          struct /* findings */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""(Earliest-first.)""";
          } findings;
          // Symbol: drake::planning::continuous_collision::CertificationResult::stats
          struct /* stats */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } stats;
          // Symbol: drake::planning::continuous_collision::CertificationResult::verdict
          struct /* verdict */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } verdict;
        } CertificationResult;
        // Symbol: drake::planning::continuous_collision::ComputeBoundingSphere
        struct /* ComputeBoundingSphere */ {
          // Source: drake/planning/continuous_collision/bounding_sphere.h
          const char* doc =
R"""(Computes a bounding sphere, in the body frame, of shape ``shape``
posed at X_LG in the body frame (the geometry-support scope).

The sphere is centered at the shape's natural center (tighter for the
broadphase prefilter than the white paper's origin-centered radius
R_g; the origin-centered bound the reach chain needs is ‖center_L‖ +
radius, which is sound because the sphere contains the geometry).
Formulas are exact containment per shape:

- Sphere(r): center X_LG·0, radius r.
- Box(w,d,h — Drake stores full sizes): box center, radius = half diagonal.
- Capsule(r, L): center, radius = L/2 + r.
- Cylinder(r, L): center, radius = √(r² + (L/2)²) (farthest point on a rim).
- Ellipsoid(a,b,c): center, radius = max(a,b,c).
- Convex / Mesh: centroid of the convex-hull vertices, radius = max vertex
distance. The vertices MUST come from the same hull object the proximity
engine collides (Shape∷GetConvexHull()), never from the raw file: the
engine's hull bakes in scale and degeneracy inflation, and the radius must
bound the geometry actually checked.

λ soundness dies quietly if any formula under-bounds, so this function
switches on the closed set of supported shape types and

Raises:
    RuntimeError on anything else (HalfSpace included — halfspaces are
    handled by dedicated rules, never through a bounding sphere).)""";
        } ComputeBoundingSphere;
        // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker
        struct /* ContinuousCollisionChecker */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc =
R"""(Certifies — not samples — that a trajectory is collision-free over its
entire continuous time domain (the problem statement).

Guarantee: if a check returns Verdict∷kCertifiedFree, then for every
time t in the trajectory's domain and every unfiltered geometry pair
(A, B), the signed distance φ_AB(q(t)) exceeds margin + padding(A, B)
— under the stated assumptions: exact real arithmetic up to the
configured numerical slack, a distance oracle accurate to its stated
tolerance, and the geometry semantics of the geometry-support scope
(Mesh ≡ convex hull). This is a statement about the continuum of
configurations, not about samples. The certificate is a property of
the path, so retiming the trajectory afterwards does not invalidate
it.

Thread safety: the Check* methods are const, own no mutable state
outside per-call scratch, and may be called concurrently on one
instance from arbitrary threads. This is deliberately stronger than
planning∷CollisionChecker, whose documentation requires a per-thread
clone for use from threads the checker does not itself own; no clone
is needed here. Construction and destruction are not thread-safe.)""";
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckEdge
          struct /* CheckEdge */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies the straight configuration-space edge q1 → q2.)""";
          } CheckEdge;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckPath
          struct /* CheckPath */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies a piecewise-linear path through the given waypoint columns.)""";
          } CheckPath;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckTrajectory
          struct /* CheckTrajectory */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies a trajectory (any supported Drake trajectory type).)""";
          } CheckTrajectory;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::ComputeMotionBounds
          struct /* ComputeMotionBounds */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } ComputeMotionBounds;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::ContinuousCollisionChecker
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Builds contexts, bounding spheres, topology tables, and runs the
capability probe (throws on unsupported geometry pairs; the
geometry-support scope).)""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::Normalize
          struct /* Normalize */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Introspection / testing seams (all const, thread-safe).)""";
          } Normalize;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::Params
          struct /* Params */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
            // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::Params::default_options
            struct /* default_options */ {
              // Source: drake/planning/continuous_collision/continuous_collision_checker.h
              const char* doc = R"""()""";
            } default_options;
            // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::Params::model
            struct /* model */ {
              // Source: drake/planning/continuous_collision/continuous_collision_checker.h
              const char* doc =
R"""(Plant + scene graph; the plant must be finalized.)""";
            } model;
            // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::Params::padding
            struct /* padding */ {
              // Source: drake/planning/continuous_collision/continuous_collision_checker.h
              const char* doc =
R"""(Per-body-pair padding; see PaddingSpec for the env/self rule.)""";
            } padding;
          } Params;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::distance_oracle
          struct /* distance_oracle */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } distance_oracle;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::kinematics_engine
          struct /* kinematics_engine */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } kinematics_engine;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::model
          struct /* model */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } model;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::pairs
          struct /* pairs */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } pairs;
        } ContinuousCollisionChecker;
        // Symbol: drake::planning::continuous_collision::DeCasteljauSplitAtHalf
        struct /* DeCasteljauSplitAtHalf */ {
          // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
          const char* doc =
R"""(Splits the Bézier control matrix ``cps`` (n × (m+1)) at u = 1/2 by de
Casteljau, writing the two children into ``left`` and ``right``
(resized as needed) and the curve value at the midpoint (the apex)
into ``mid``. Allocation-free when the outputs are already correctly
sized.)""";
        } DeCasteljauSplitAtHalf;
        // Symbol: drake::planning::continuous_collision::DistanceOracle
        struct /* DistanceOracle */ {
          // Source: drake/planning/continuous_collision/distance_oracle.h
          const char* doc =
R"""(Narrowphase distance abstraction (the distance-oracle contract).
Stateless per query and thread-compatible: configuration comes in via
the caller's QueryObject.

Contract: SignedDistance returns φ̂ with |φ̂ − φ_true| ≤ tolerance()
whenever φ_true is at or above −tolerance(), and returns a definitely
negative value when the shapes interpenetrate beyond tolerance. Only
over-reporting a distance at or above threshold could fake a
certificate (the soundness argument), which is why the capability
probe keeps any not-a-true-distance backend out of the loop entirely.

The collision filter state is snapshotted from the model inspector at
construction: pairs() is the set of pairs that were unfiltered *then*.
Filter changes applied to a Context afterwards are not observed, so a
checker built on this oracle keeps certifying the pair set it was
constructed with.)""";
          // Symbol: drake::planning::continuous_collision::DistanceOracle::DistanceOracle
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(Runs the capability probe: enumerates the unfiltered proximity pairs
from the model's SceneGraph inspector (collision filter state
snapshotted at construction), classifies every (shape, shape)
combination as {native, halfspace-fallback, unsupported}, and

Raises:
    RuntimeError immediately naming the offending geometries if any
    pair is unsupported (deformables; halfspace–halfspace). Never
    discovers an unsupported pair mid-certification.)""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::DistanceOracle::SignedDistance
          struct /* SignedDistance */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(Signed distance for one pair at the configuration already set in the
context that produced ``query_object``. Optionally reports world-frame
closest points when the route provides them.

``pair`` need not be an element of pairs(): the facade copies the
probe's records and rewrites their thresholds, so only ``pair.id`` and
``pair.route`` are read here. Both routes always fill the optional
out-params.

Raises:
    RuntimeError if ``pair`` carries a halfspace route but its
    geometries were not classified by this oracle's capability probe
    (i.e. the record did not come from pairs()).)""";
          } SignedDistance;
          // Symbol: drake::planning::continuous_collision::DistanceOracle::pairs
          struct /* pairs */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(The unfiltered pairs found by the probe (thresholds default 0; the
facade rewrites them from margin + padding).)""";
          } pairs;
          // Symbol: drake::planning::continuous_collision::DistanceOracle::support_report
          struct /* support_report */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(Human-readable probe report: one line per distinct shape-type
combination and its route (includes the "Mesh certified as convex
hull" notices; the risk register).)""";
          } support_report;
          // Symbol: drake::planning::continuous_collision::DistanceOracle::tolerance
          struct /* tolerance */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(τ used in the certificate arithmetic (the numerical policy).)""";
          } tolerance;
        } DistanceOracle;
        // Symbol: drake::planning::continuous_collision::DistanceRoute
        struct /* DistanceRoute */ {
          // Source: drake/planning/continuous_collision/distance_oracle.h
          const char* doc =
R"""(How the oracle computes signed distance for one pair, resolved once by
the capability probe (the geometry-support scope; the distance-oracle
contract): no per-query dispatch decisions.)""";
          // Symbol: drake::planning::continuous_collision::DistanceRoute::kHalfSpaceA
          struct /* kHalfSpaceA */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(Analytic halfspace support-function fallback; geometry ``a`` is the
halfspace.)""";
          } kHalfSpaceA;
          // Symbol: drake::planning::continuous_collision::DistanceRoute::kHalfSpaceB
          struct /* kHalfSpaceB */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc = R"""(Same, geometry ``b`` is the halfspace.)""";
          } kHalfSpaceB;
          // Symbol: drake::planning::continuous_collision::DistanceRoute::kNative
          struct /* kNative */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(QueryObject∷ComputeSignedDistancePairClosestPoints.)""";
          } kNative;
        } DistanceRoute;
        // Symbol: drake::planning::continuous_collision::Finding
        struct /* Finding */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(One violation or inconclusive record (the architecture).)""";
          // Symbol: drake::planning::continuous_collision::Finding::definite
          struct /* definite */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(true ⇒ definite violation; false ⇒ grazing / inconclusive.)""";
          } definite;
          // Symbol: drake::planning::continuous_collision::Finding::distance
          struct /* distance */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Oracle signed distance at q for this pair.)""";
          } distance;
          // Symbol: drake::planning::continuous_collision::Finding::motion_bound
          struct /* motion_bound */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Motion bound Δ_p at the terminal node (0 for breakpoint findings).)""";
          } motion_bound;
          // Symbol: drake::planning::continuous_collision::Finding::nearest_a_W
          struct /* nearest_a_W */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Closest points in world frame at q, when the narrowphase provides them
(violation findings; planners use these to push trajectories out of
collision).)""";
          } nearest_a_W;
          // Symbol: drake::planning::continuous_collision::Finding::nearest_b_W
          struct /* nearest_b_W */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } nearest_b_W;
          // Symbol: drake::planning::continuous_collision::Finding::pair
          struct /* pair */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } pair;
          // Symbol: drake::planning::continuous_collision::Finding::q
          struct /* q */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(The witness configuration, exactly on the trajectory.)""";
          } q;
          // Symbol: drake::planning::continuous_collision::Finding::time
          struct /* time */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Trajectory time of the witness configuration.)""";
          } time;
        } Finding;
        // Symbol: drake::planning::continuous_collision::IsCertified
        struct /* IsCertified */ {
          // Source: drake/planning/continuous_collision/numerics.h
          const char* doc =
R"""(True iff the pair is certified on the whole node.)""";
        } IsCertified;
        // Symbol: drake::planning::continuous_collision::IsDefiniteViolation
        struct /* IsDefiniteViolation */ {
          // Source: drake/planning/continuous_collision/numerics.h
          const char* doc =
R"""(True iff the representative configuration is a definite violation.)""";
        } IsDefiniteViolation;
        // Symbol: drake::planning::continuous_collision::KinematicsEngine
        struct /* KinematicsEngine */ {
          // Source: drake/planning/continuous_collision/motion_bound_table.h
          const char* doc =
R"""(Construction-time kinematic analysis of a plant (the displacement
lemma): joint classification, per-hop fixed-transform translations,
per-body proximity geometry bounding spheres, and subtree tables for
J(p). Thread-compatible; all methods are const after construction and
hold no mutable state, so concurrent ComputeMotionBoundTable() calls
are safe.

Typical use by the certifier: - once, at checker construction:
KinematicsEngine engine(model); engine.body_spheres(b) for the
prefilter; - once per Check* call:
engine.ComputeMotionBoundTable(path, pairs); - once per node, per
pair: table.MotionBound(pair_index, w).)""";
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::ComputeMotionBoundTable
          struct /* ComputeMotionBoundTable */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc_2args =
R"""(Assembles the λ CSR table for ``pairs`` given the path's global
control-point box (prismatic chain contributions use the box, so the
bound is trajectory-adaptive; the displacement lemma). Coordinates
flagged constant by the path are removed from every J(p), and their
residual motion inside the box is charged to
MotionBoundTable∷carveout_slack() instead.

Raises:
    RuntimeError naming the joint if the path moves a coordinate of an
    unsupported joint type (quaternion floating, ball).)""";
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc_4args =
R"""(Raw-data overload of the above, for callers (and tests) that already
hold the trajectory's global control-point box. ``lower`` and
``upper`` are the per-coordinate box bounds and
``constant_coordinates`` flags the coordinates the path cannot change;
all three have size num_positions(). A coordinate flagged constant
still contributes (upper − lower) worth of residual motion to the
pair's carve-out slack, so the two arguments must describe the same
trajectory: flagging a coordinate constant does not license widening
its box.

Raises:
    RuntimeError on a size mismatch, an empty box (lower > upper), a
    non-finite bound, a moving coordinate of an unsupported joint
    type, a pair whose distal side carries a HalfSpace across a
    rotational coordinate, or a pair whose distal side carries a
    HalfSpace across a rotational coordinate that is constant only to
    within a tolerance (a HalfSpace has no finite reach, so such a
    coordinate must be *exactly* constant).)""";
          } ComputeMotionBoundTable;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::CoordinatesAffectingPair
          struct /* CoordinatesAffectingPair */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(The position-coordinate indices whose motion changes the relative pose
of the two bodies (J(p) before any carve-out), from topology alone.
Sorted ascending.)""";
          } CoordinatesAffectingPair;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::KinematicsEngine
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Builds topology tables and per-body geometry bounding spheres.
Classification only; unsupported joint types throw later, and only if
a given path actually moves them (constant-coordinate carve-out, the
joint-support scope).

``model`` is aliased and must outlive this object.

Raises:
    RuntimeError if a HalfSpace geometry is on the *distal* side of a
    rotational coordinate relative to an unfiltered partner (unbounded
    reach). A HalfSpace that is merely the static partner of a
    rotating body — the anchored ground plane under a robot arm, the
    overwhelmingly common case — is accepted: λ then bounds the
    partner's points, and signed distance is symmetric, so the
    certificate still holds.

Raises:
    RuntimeError if the plant is not finalized, if a joint is
    "reversed" (its declared parent body is outboard of its declared
    child body in the multibody tree — a documented v1 exclusion), or
    if any proximity geometry has a shape ComputeBoundingSphere()
    rejects.)""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::body_has_halfspace
          struct /* body_has_halfspace */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(True iff ``body`` carries at least one HalfSpace proximity geometry.)""";
          } body_has_halfspace;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::body_radius
          struct /* body_radius */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Radius, about the body frame origin, of a sphere containing every
proximity geometry of ``body`` — the start of the reach chain. Zero
for a body with no (non-HalfSpace) proximity geometry.)""";
          } body_radius;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::body_sphere_geometries
          struct /* body_sphere_geometries */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(The geometry ids matching body_spheres(body), element for element.)""";
          } body_sphere_geometries;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::body_spheres
          struct /* body_spheres */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Bounding spheres (body frame) of every proximity geometry of ``body``,
used by the reach chain start and by the certifier's sphere prefilter.
HalfSpace geometries have no bounding sphere and are omitted.)""";
          } body_spheres;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::geometry_sphere
          struct /* geometry_sphere */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(The bounding sphere (in its body's frame) of one proximity geometry.

Raises:
    RuntimeError if ``id`` is not a proximity geometry of this model
    or is a HalfSpace (which has none).)""";
          } geometry_sphere;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::num_positions
          struct /* num_positions */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc = R"""()""";
          } num_positions;
          // Symbol: drake::planning::continuous_collision::KinematicsEngine::plant
          struct /* plant */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc = R"""()""";
          } plant;
        } KinematicsEngine;
        // Symbol: drake::planning::continuous_collision::MotionBoundTable
        struct /* MotionBoundTable */ {
          // Source: drake/planning/continuous_collision/motion_bound_table.h
          const char* doc =
R"""(Per-pair motion-bound coefficients in CSR layout (the displacement
lemma): for pair index k, a contiguous span of (position-coordinate
index j, λ(j, p)) entries over J(p), the coordinates that change the
pair's relative pose. λ has units of meters of worst-case point
displacement of the pair's distal side per unit change of coordinate
j, valid for every configuration in the trajectory's global
control-point box.

Each pair also carries a scalar ``carveout_slack(p)``, the residual
motion of the coordinates the constant-coordinate carve-out
(trajectory normalization; the joint-support scope) removed from J(p).
"Constant" there is a *tolerance* — a coordinate whose global
control-box range is at most Options∷continuity_tolerance — not an
identity, so a carved coordinate may still displace the pair's distal
side by up to λ̃_j · range_j. That residual is charged unconditionally
inside MotionBound(), which is what makes Δ_p a true upper bound on
the pair's relative motion over the whole trajectory rather than one
that ignores the carved coordinates. It is exactly zero — bit for bit
— whenever every carved coordinate is *exactly* constant, which is the
case for every path whose control points repeat a coordinate's value
verbatim.)""";
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::GetEntries
          struct /* GetEntries */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Introspection for tests: the (coordinate, λ) entries of one pair,
ordered by increasing coordinate index.)""";
          } GetEntries;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::MotionBound
          struct /* MotionBound */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Δ_p(ν) = carveout_slack(p) + Σ_{j ∈ J(p)} λ(j,p) · w_j — a sparse dot
product against the node's per-coordinate deviations w, plus the
carved coordinates' residual (the interval certificate, requirement
P3).)""";
          } MotionBound;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::MotionBoundTable
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc_0args = R"""(Constructs an empty table (zero pairs).)""";
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc_4args =
R"""(Constructs the CSR table directly from its four arrays.

Parameter ``row_start``:
    Size num_pairs + 1, starting at 0 and non-decreasing;
    row_start.back() is the total entry count.

Parameter ``coord``:
    Position-coordinate index of every entry.

Parameter ``lambda``:
    λ of every entry, element for element with ``coord``.

Parameter ``carveout_slack``:
    One residual per pair.

Raises:
    RuntimeError if the arrays do not satisfy those invariants.)""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::carveout_slack
          struct /* carveout_slack */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Σ over the coordinates of J_topo(p) that the carve-out removed of λ̃_j
· (global_upper_j − global_lower_j): an upper bound on how far this
pair's two geometries can move relative to each other purely through
the coordinates the table no longer tracks. Zero when every carved
coordinate is exactly constant.)""";
          } carveout_slack;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::num_entries
          struct /* num_entries */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(Total number of (coordinate, λ) entries over all pairs.)""";
          } num_entries;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::num_pairs
          struct /* num_pairs */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc = R"""()""";
          } num_pairs;
          // Symbol: drake::planning::continuous_collision::MotionBoundTable::pair_is_static
          struct /* pair_is_static */ {
            // Source: drake/planning/continuous_collision/motion_bound_table.h
            const char* doc =
R"""(True iff J(p) is empty after the constant-coordinate carve-out: no
coordinate the trajectory *moves* changes this pair's relative pose,
so it is checked once. Note that "static" does not mean "immobile": a
static pair can still drift by carveout_slack(p), which callers that
shortcut MotionBound() for such a pair must charge themselves.)""";
          } pair_is_static;
        } MotionBoundTable;
        // Symbol: drake::planning::continuous_collision::Options
        struct /* Options */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Options controlling one certification call (the architecture; the
numerical policy).)""";
          // Symbol: drake::planning::continuous_collision::Options::certificate_slack
          struct /* certificate_slack */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(ε_slack: swallows floating-point noise in the bound arithmetic.)""";
          } certificate_slack;
          // Symbol: drake::planning::continuous_collision::Options::continuity_tolerance
          struct /* continuity_tolerance */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Junction C0-continuity tolerance (per coordinate; modulo 2π for
coordinates listed in continuous_revolute_indices).)""";
          } continuity_tolerance;
          // Symbol: drake::planning::continuous_collision::Options::continuous_revolute_indices
          struct /* continuous_revolute_indices */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Position coordinates whose junction continuity is checked modulo 2π
(GcsTrajectoryOptimization continuous-revolute convention).

See also:
    planning∷trajectory_optimization∷GetContinuousRevoluteJointIndices)""";
          } continuous_revolute_indices;
          // Symbol: drake::planning::continuous_collision::Options::emit_certificate
          struct /* emit_certificate */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(If true, every certification event is recorded into a Certificate that
VerifyCertificate() can independently replay (the search algorithm).)""";
          } emit_certificate;
          // Symbol: drake::planning::continuous_collision::Options::margin
          struct /* margin */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Global clearance margin δ in meters. The certificate proves signed
distance > margin + padding for every pair at every time.)""";
          } margin;
          // Symbol: drake::planning::continuous_collision::Options::max_conversion_degree
          struct /* max_conversion_degree */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Maximum polynomial degree accepted for monomial→Bernstein conversion.)""";
          } max_conversion_degree;
          // Symbol: drake::planning::continuous_collision::Options::max_nodes
          struct /* max_nodes */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Optional node budget; exceeded ⇒ Verdict∷kBudgetExhausted.)""";
          } max_nodes;
          // Symbol: drake::planning::continuous_collision::Options::max_reported_findings
          struct /* max_reported_findings */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } max_reported_findings;
          // Symbol: drake::planning::continuous_collision::Options::min_interval
          struct /* min_interval */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Resolution floor as a fraction of a segment's parameter width; nodes
narrower than this become kInconclusive findings instead of splitting.)""";
          } min_interval;
          // Symbol: drake::planning::continuous_collision::Options::mode
          struct /* mode */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } mode;
          // Symbol: drake::planning::continuous_collision::Options::parallelism
          struct /* parallelism */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } parallelism;
          // Symbol: drake::planning::continuous_collision::Options::query_tolerance
          struct /* query_tolerance */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(τ: the distance oracle's accuracy contract in meters (the
distance-oracle contract; the numerical policy).)""";
          } query_tolerance;
        } Options;
        // Symbol: drake::planning::continuous_collision::PaddingSpec
        struct /* PaddingSpec */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Per-body-pair padding: the effective threshold for pair p is margin +
padding(p).

Which of the two scalars applies to a pair is decided by *anchoring*,
from plant topology alone. A body is anchored iff no position
coordinate of the plant changes its pose relative to the world — the
world body itself, and everything welded to it directly or
transitively. A pair is a self-collision pair iff both of its bodies
are non-anchored, and an environment pair otherwise. The rule never
depends on which trajectory is being checked.)""";
          // Symbol: drake::planning::continuous_collision::PaddingSpec::env_padding
          struct /* env_padding */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Padding for robot-vs-environment pairs, i.e. pairs with at least one
anchored body.)""";
          } env_padding;
          // Symbol: drake::planning::continuous_collision::PaddingSpec::per_body_pair
          struct /* per_body_pair */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Optional dense symmetric matrix indexed by BodyIndex, sized num_bodies
× num_bodies. Entry (a, b) overrides the scalars for that body pair; a
NaN entry means "not covered", and that pair falls back to env_padding
/ self_padding.)""";
          } per_body_pair;
          // Symbol: drake::planning::continuous_collision::PaddingSpec::self_padding
          struct /* self_padding */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Padding for robot-vs-robot (self-collision) pairs, i.e. pairs whose
two bodies are both non-anchored.)""";
          } self_padding;
        } PaddingSpec;
        // Symbol: drake::planning::continuous_collision::PairId
        struct /* PairId */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Identifies an unfiltered proximity geometry pair.)""";
          // Symbol: drake::planning::continuous_collision::PairId::a
          struct /* a */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } a;
          // Symbol: drake::planning::continuous_collision::PairId::b
          struct /* b */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } b;
          // Symbol: drake::planning::continuous_collision::PairId::body_a
          struct /* body_a */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } body_a;
          // Symbol: drake::planning::continuous_collision::PairId::body_b
          struct /* body_b */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } body_b;
        } PairId;
        // Symbol: drake::planning::continuous_collision::PairRecord
        struct /* PairRecord */ {
          // Source: drake/planning/continuous_collision/distance_oracle.h
          const char* doc =
R"""(One unfiltered proximity pair with its pre-resolved distance route and
effective threshold m_p = margin + padding(p).)""";
          // Symbol: drake::planning::continuous_collision::PairRecord::id
          struct /* id */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc = R"""()""";
          } id;
          // Symbol: drake::planning::continuous_collision::PairRecord::route
          struct /* route */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc = R"""()""";
          } route;
          // Symbol: drake::planning::continuous_collision::PairRecord::threshold
          struct /* threshold */ {
            // Source: drake/planning/continuous_collision/distance_oracle.h
            const char* doc =
R"""(Filled by the facade from margin + PaddingSpec.)""";
          } threshold;
        } PairRecord;
        // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath
        struct /* PiecewiseBezierPath */ {
          // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
          const char* doc =
R"""(Ordered, C0-validated piecewise-Bézier path over the plant's
generalized positions. Every accepted trajectory type is converted,
exactly, into this representation up front (trajectory normalization).

Two Bézier facts the whole method rests on: (1) the curve lies in the
convex hull of its control points, so per coordinate i, q_i(s) ∈
[min_j P_{j,i}, max_j P_{j,i}]; (2) de Casteljau subdivision at any
parameter u yields two child curves whose control points exactly
represent the two sub-curves and are convex combinations of the
parent's, so every descendant node's control box is contained in this
path's global control box. The apex of the de Casteljau triangle at u
is exactly q(u).)""";
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::EvaluateSegment
          struct /* EvaluateSegment */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Evaluates segment ``segment_index`` at local parameter s ∈ [0, 1].)""";
          } EvaluateSegment;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::FromTrajectory
          struct /* FromTrajectory */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Normalizes any supported Drake trajectory (BezierCurve,
CompositeTrajectory, BsplineTrajectory via knot insertion,
PiecewisePolynomial via monomial→Bernstein change of basis).

Raises:
    RuntimeError on unsupported segment types, degree above
    options.max_conversion_degree, or junction discontinuity beyond
    options.continuity_tolerance (modulo 2π for coordinates in
    options.continuous_revolute_indices).)""";
          } FromTrajectory;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::FromWaypoints
          struct /* FromWaypoints */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Normalizes an n × K waypoint matrix into K−1 order-1 segments (exact).
Segment k spans time [k, k+1].

Raises:
    RuntimeError if K < 2.)""";
          } FromWaypoints;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::PiecewiseBezierPath
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::Value
          struct /* Value */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Evaluates the path at time t (for tests and breakpoint checks; the hot
loop never calls this — it uses de Casteljau apexes).)""";
          } Value;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::constant_coordinates
          struct /* constant_coordinates */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(True for coordinates whose value is identical (within the continuity
tolerance) across all control points of all segments; such coordinates
are treated as welded for the check (trajectory normalization; the
joint-support scope).)""";
          } constant_coordinates;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::end_time
          struct /* end_time */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } end_time;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::global_lower_bound
          struct /* global_lower_bound */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc =
R"""(Per-coordinate global control-point box over all segments (trajectory
normalization); used for trajectory-adaptive prismatic reach bounds.)""";
          } global_lower_bound;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::global_upper_bound
          struct /* global_upper_bound */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } global_upper_bound;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::num_positions
          struct /* num_positions */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } num_positions;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::segments
          struct /* segments */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } segments;
          // Symbol: drake::planning::continuous_collision::PiecewiseBezierPath::start_time
          struct /* start_time */ {
            // Source: drake/planning/continuous_collision/piecewise_bezier_path.h
            const char* doc = R"""()""";
          } start_time;
        } PiecewiseBezierPath;
        // Symbol: drake::planning::continuous_collision::SearchMode
        struct /* SearchMode */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Search modes for certification (the search algorithm).)""";
          // Symbol: drake::planning::continuous_collision::SearchMode::kCertifyAll
          struct /* kCertifyAll */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Certify the full domain and return every violation / inconclusive
region found (bounded by Options∷max_reported_findings).)""";
          } kCertifyAll;
          // Symbol: drake::planning::continuous_collision::SearchMode::kFindFirstViolation
          struct /* kFindFirstViolation */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Return on the first definite violation; serial execution returns the
earliest one in time.)""";
          } kFindFirstViolation;
        } SearchMode;
        // Symbol: drake::planning::continuous_collision::Statistics
        struct /* Statistics */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Cost accounting for one certification call.)""";
          // Symbol: drake::planning::continuous_collision::Statistics::max_depth
          struct /* max_depth */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } max_depth;
          // Symbol: drake::planning::continuous_collision::Statistics::narrowphase_queries
          struct /* narrowphase_queries */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } narrowphase_queries;
          // Symbol: drake::planning::continuous_collision::Statistics::nodes
          struct /* nodes */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } nodes;
          // Symbol: drake::planning::continuous_collision::Statistics::sphere_certifications
          struct /* sphere_certifications */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } sphere_certifications;
          // Symbol: drake::planning::continuous_collision::Statistics::wall_time_s
          struct /* wall_time_s */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc = R"""()""";
          } wall_time_s;
        } Statistics;
        // Symbol: drake::planning::continuous_collision::Verdict
        struct /* Verdict */ {
          // Source: drake/planning/continuous_collision/options.h
          const char* doc =
R"""(Outcome of a certification run (the problem statement).)""";
          // Symbol: drake::planning::continuous_collision::Verdict::kBudgetExhausted
          struct /* kBudgetExhausted */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(The optional node budget was exhausted first.)""";
          } kBudgetExhausted;
          // Symbol: drake::planning::continuous_collision::Verdict::kCertifiedFree
          struct /* kCertifiedFree */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Proof: every unfiltered pair keeps signed distance > margin + padding
over the entire continuous time domain.)""";
          } kCertifiedFree;
          // Symbol: drake::planning::continuous_collision::Verdict::kInconclusive
          struct /* kInconclusive */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(Subdivision hit the resolution floor with some pair's clearance within
oracle tolerance of the threshold (grazing trajectory).)""";
          } kInconclusive;
          // Symbol: drake::planning::continuous_collision::Verdict::kViolationFound
          struct /* kViolationFound */ {
            // Source: drake/planning/continuous_collision/options.h
            const char* doc =
R"""(An exactly-on-trajectory configuration violates the threshold.)""";
          } kViolationFound;
        } Verdict;
        // Symbol: drake::planning::continuous_collision::VerifyCertificate
        struct /* VerifyCertificate */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc =
R"""(Independently replays every record of ``certificate`` (recomputing
node control boxes from freshly restricted control points and
re-querying distances) and checks interval coverage of the full domain
for every pair. Returns true iff the certificate holds (the search
algorithm).)""";
        } VerifyCertificate;
      } continuous_collision;
    } planning;
  } drake;
} pydrake_doc_planning_continuous_collision;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
