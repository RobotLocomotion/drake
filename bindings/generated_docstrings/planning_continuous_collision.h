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

// #include "drake/planning/continuous_collision/continuous_collision_checker.h"

// Symbol: pydrake_doc_planning_continuous_collision
constexpr struct /* pydrake_doc_planning_continuous_collision */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::continuous_collision
      struct /* continuous_collision */ {
        // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker
        struct /* ContinuousCollisionChecker */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc =
R"""(Certifies, rather than samples, that a trajectory is collision-free
over its entire continuous time domain.

Guarantee: if a check returns Verdict∷kCertifiedFree, then for every
time t in the trajectory's domain and every unfiltered geometry pair
(A, B), the signed distance ϕ_AB(q(t)) exceeds Options∷margin. That
holds under three assumptions: exact real arithmetic up to an internal
numerical slack, a distance oracle accurate to its stated tolerance,
and Mesh ≡ convex hull. The proof is a property of the path, so
retiming the trajectory afterwards does not invalidate it.

Resolution contract: write δ for Options∷margin, r for
Options∷distance_resolution, τ_p for the oracle tolerance of pair p
(at least 1 µm; Drake's documented signed-distance accuracy for that
shape combination), ε for the internal slack (1 nm), and σ_p for the
residual motion of coordinates the trajectory holds constant only to
within the continuity tolerance (exactly zero when they are exactly
constant, the common case). Then, for every pair, - if ϕ_p(q(t)) > δ +
r + σ_p + 2τ_p + ε for every t, the pair is certified, so a trajectory
that clears the margin by that much everywhere returns
Verdict∷kCertifiedFree; - if ϕ_p(q(t)) < δ − (r + σ_p + 2τ_p) for some
t, the check returns Verdict∷kViolationFound; - Verdict∷kInconclusive
is therefore possible only when some pair's clearance comes within
that band of the margin, and its Finding then names an on-trajectory
configuration whose reported distance lies in [δ − τ_p, δ + τ_p + ε +
σ_p + r]. Resolutions below what double precision can represent along
a segment are capped by a floating-point backstop.

Thread safety: the Check* methods are const, own no mutable state
outside per-call scratch, and may be called concurrently on one
instance from arbitrary threads. This is stronger than
planning∷CollisionChecker, whose documentation requires a per-thread
clone for use from threads the checker does not itself own; no clone
is needed here. Construction and destruction are not thread-safe.)""";
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckEdge
          struct /* CheckEdge */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies the straight configuration-space edge q1 → q2.

Raises:
    RuntimeError if q1 or q2 does not have one entry per generalized
    position of the plant.

Raises:
    RuntimeError under every condition CheckTrajectory() lists.)""";
          } CheckEdge;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckPath
          struct /* CheckPath */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies the piecewise-linear path through the given waypoint
columns.

Raises:
    RuntimeError if ``waypoints`` has fewer than two columns, or does
    not have one row per generalized position of the plant.

Raises:
    RuntimeError under every condition CheckTrajectory() lists.)""";
          } CheckPath;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::CheckTrajectory
          struct /* CheckTrajectory */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Certifies a trajectory (BezierCurve, BsplineTrajectory,
PiecewisePolynomial, or a CompositeTrajectory of those).

Raises:
    RuntimeError if Options∷margin is not a finite nonnegative
    distance, or if Options∷distance_resolution is not a finite
    positive distance.

Raises:
    RuntimeError if the trajectory's row count differs from the
    plant's number of generalized positions.

Raises:
    RuntimeError if the trajectory is not one of the supported types,
    has a segment of degree above 10, or is discontinuous at a
    junction.

Raises:
    RuntimeError if Options∷continuous_revolute_indices names a
    coordinate outside the plant's.

Raises:
    RuntimeError if the trajectory moves a coordinate of an
    unsupported joint type (quaternion floating, ball), or moves a
    HalfSpace across a rotational coordinate.)""";
          } CheckTrajectory;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::ContinuousCollisionChecker
          struct /* ctor */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Builds contexts, bounding spheres and topology tables, and runs the
capability probe.

Raises:
    RuntimeError if ``model`` is null or its plant is not finalized.

Raises:
    RuntimeError if ``default_options`` is invalid; see
    CheckTrajectory().

Raises:
    RuntimeError if a pair's shape combination is unsupported, i.e. a
    deformable geometry or halfspace against halfspace.

Raises:
    RuntimeError if the plant's topology or geometry defeats the
    motion bound: a rotating HalfSpace, a reversed joint, a kinematic
    loop, or a proximity shape with no bounding sphere.)""";
          } ctor;
          // Symbol: drake::planning::continuous_collision::ContinuousCollisionChecker::model
          struct /* model */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } model;
        } ContinuousCollisionChecker;
        // Symbol: drake::planning::continuous_collision::Finding
        struct /* Finding */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc =
R"""(Where the plan fails, or where it could not be decided.)""";
          // Symbol: drake::planning::continuous_collision::Finding::body_a
          struct /* body_a */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } body_a;
          // Symbol: drake::planning::continuous_collision::Finding::body_b
          struct /* body_b */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } body_b;
          // Symbol: drake::planning::continuous_collision::Finding::distance
          struct /* distance */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""(Signed distance of the pair at q.)""";
          } distance;
          // Symbol: drake::planning::continuous_collision::Finding::geometry_a
          struct /* geometry_a */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } geometry_a;
          // Symbol: drake::planning::continuous_collision::Finding::geometry_b
          struct /* geometry_b */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } geometry_b;
          // Symbol: drake::planning::continuous_collision::Finding::nearest_a_W
          struct /* nearest_a_W */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Closest points in the world frame at q; present for violations, so
that planners can push the trajectory out of collision.)""";
          } nearest_a_W;
          // Symbol: drake::planning::continuous_collision::Finding::nearest_b_W
          struct /* nearest_b_W */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } nearest_b_W;
          // Symbol: drake::planning::continuous_collision::Finding::q
          struct /* q */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(The witness configuration, exactly on the trajectory.)""";
          } q;
          // Symbol: drake::planning::continuous_collision::Finding::time
          struct /* time */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Trajectory time of the witness configuration.)""";
          } time;
        } Finding;
        // Symbol: drake::planning::continuous_collision::Options
        struct /* Options */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc = R"""(Options controlling one check.)""";
          // Symbol: drake::planning::continuous_collision::Options::continuous_revolute_indices
          struct /* continuous_revolute_indices */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Position coordinates whose junction continuity is checked modulo 2π
(the GcsTrajectoryOptimization continuous-revolute convention).

See also:
    planning∷trajectory_optimization∷GetContinuousRevoluteJointIndices)""";
          } continuous_revolute_indices;
          // Symbol: drake::planning::continuous_collision::Options::distance_resolution
          struct /* distance_resolution */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Resolution floor r in meters. A pair stops being refined on a node
once its bounded relative motion over that node is at most r; if it is
still undecided there, the check reports Verdict∷kInconclusive with
that node's midpoint as the witness. Definitive verdicts are
guaranteed for a trajectory whose clearance stays more than r (plus
the oracle tolerance, see the class documentation) away from the
margin everywhere; the cost of a grazing trajectory grows roughly
linearly in 1/r. Must be finite and positive.)""";
          } distance_resolution;
          // Symbol: drake::planning::continuous_collision::Options::margin
          struct /* margin */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Clearance margin δ in meters: the check certifies signed distance >
margin for every unfiltered pair at every time. Must be finite and
nonnegative.)""";
          } margin;
          // Symbol: drake::planning::continuous_collision::Options::parallelism
          struct /* parallelism */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } parallelism;
        } Options;
        // Symbol: drake::planning::continuous_collision::Result
        struct /* Result */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc = R"""(Result of one check.)""";
          // Symbol: drake::planning::continuous_collision::Result::finding
          struct /* finding */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(The earliest violation, or the inconclusive witness; empty iff the
verdict is Verdict∷kCertifiedFree.)""";
          } finding;
          // Symbol: drake::planning::continuous_collision::Result::num_nodes
          struct /* num_nodes */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Nodes visited by the adaptive subdivision; a cost measure.)""";
          } num_nodes;
          // Symbol: drake::planning::continuous_collision::Result::verdict
          struct /* verdict */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc = R"""()""";
          } verdict;
        } Result;
        // Symbol: drake::planning::continuous_collision::Verdict
        struct /* Verdict */ {
          // Source: drake/planning/continuous_collision/continuous_collision_checker.h
          const char* doc = R"""(Outcome of one check.)""";
          // Symbol: drake::planning::continuous_collision::Verdict::kCertifiedFree
          struct /* kCertifiedFree */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Proof: every unfiltered pair keeps signed distance > margin over the
entire continuous time domain.)""";
          } kCertifiedFree;
          // Symbol: drake::planning::continuous_collision::Verdict::kInconclusive
          struct /* kInconclusive */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(Some pair's clearance comes within Options∷distance_resolution (plus
the oracle tolerance) of the margin, so refining further cannot decide
it: the trajectory grazes the margin.)""";
          } kInconclusive;
          // Symbol: drake::planning::continuous_collision::Verdict::kViolationFound
          struct /* kViolationFound */ {
            // Source: drake/planning/continuous_collision/continuous_collision_checker.h
            const char* doc =
R"""(An exactly-on-trajectory configuration violates the threshold.)""";
          } kViolationFound;
        } Verdict;
      } continuous_collision;
    } planning;
  } drake;
} pydrake_doc_planning_continuous_collision;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
