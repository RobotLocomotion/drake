#pragma once

/* @file
Numerical policy, shape classification and the pair record shared by every
translation unit of this package. Nothing here is public API.

Numerics. Let ϕ̂ be the oracle's reported signed distance at the node's
representative configuration, τ the oracle accuracy contract (|ϕ̂ − ϕ_true| ≤ τ
on the at-or-above-threshold branch), Δ the motion bound for the node, m the
threshold (Options::margin), and ε the numerical slack.

 - Certified:          ϕ̂ − τ − Δ > m + ε   (sound by the displacement lemma:
                       every configuration on the node keeps clearance > m).
 - Definite violation: ϕ̂ + τ < m           (the true clearance at an exactly
                       on-trajectory configuration is below threshold).
 - Otherwise the pair is gray and drives subdivision.

The certificate is mathematical modulo τ and ε. ε is 1e-9 m, which dominates
the accumulated floating-point error of the w, λ and dot-product expression
depths involved.

TODO(wernerpe): Harden the arithmetic with directed rounding, so that the
certificate holds without the ε slack. */

#include <type_traits>

#include "drake/common/unused.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/* Junction C0-continuity tolerance (per coordinate; modulo 2π for coordinates
 listed in Options::continuous_revolute_indices). Doubles as the width below
 which a coordinate's global control-point range counts as constant. */
constexpr double kContinuityTolerance = 1e-7;

/* τ: the distance oracle's baseline accuracy contract in meters. The per-pair
 τ_p also charges Drake's documented per-shape-combination accuracy. */
constexpr double kQueryTolerance = 1e-6;

/* ε: swallows floating-point noise in the bound arithmetic. */
constexpr double kNumericalSlack = 1e-9;

/* Maximum polynomial degree accepted for monomial→Bernstein conversion. */
constexpr int kMaxConversionDegree = 10;

/* True iff the pair is certified on the whole node. */
inline bool IsCertified(double phi_hat, double tau, double motion_bound,
                        double threshold, double slack = kNumericalSlack) {
  return phi_hat - tau - motion_bound > threshold + slack;
}

/* True iff the representative configuration is a definite violation. */
inline bool IsDefiniteViolation(double phi_hat, double tau, double threshold) {
  return phi_hat + tau < threshold;
}

/* The closed set of shape classes this package recognizes. The enumerator
 values are the row and column indices of the documented-accuracy table in
 continuous_collision_checker.cc, so they must stay contiguous from zero.
 Anything outside the set is `kUnsupported` and is refused by the oracle's
 capability probe, mirroring the throw-on-unknown-shape rule
 ComputeBoundingSphere() uses. */
enum class ShapeClass {
  kSphere = 0,
  kBox = 1,
  kCapsule = 2,
  kCylinder = 3,
  kEllipsoid = 4,
  kConvex = 5,
  kMesh = 6,
  kHalfSpace = 7,
  kUnsupported = 8,
};

constexpr int kNumShapeClasses = 9;

/* Classifies `shape` into the set above. */
inline ShapeClass Classify(const geometry::Shape& shape) {
  return shape.Visit<ShapeClass>([](const auto& s) {
    using S = std::decay_t<decltype(s)>;
    unused(s);
    if constexpr (std::is_same_v<S, geometry::Sphere>) {
      return ShapeClass::kSphere;
    } else if constexpr (std::is_same_v<S, geometry::Box>) {
      return ShapeClass::kBox;
    } else if constexpr (std::is_same_v<S, geometry::Capsule>) {
      return ShapeClass::kCapsule;
    } else if constexpr (std::is_same_v<S, geometry::Cylinder>) {
      return ShapeClass::kCylinder;
    } else if constexpr (std::is_same_v<S, geometry::Ellipsoid>) {
      return ShapeClass::kEllipsoid;
    } else if constexpr (std::is_same_v<S, geometry::Convex>) {
      return ShapeClass::kConvex;
    } else if constexpr (std::is_same_v<S, geometry::Mesh>) {
      return ShapeClass::kMesh;
    } else if constexpr (std::is_same_v<S, geometry::HalfSpace>) {
      return ShapeClass::kHalfSpace;
    } else {
      return ShapeClass::kUnsupported;
    }
  });
}

/* True iff `shape` is a HalfSpace. A halfspace is unbounded, so it has no
 bounding sphere, and Drake computes signed distance against it only for a
 Sphere partner. */
inline bool IsHalfSpace(const geometry::Shape& shape) {
  return Classify(shape) == ShapeClass::kHalfSpace;
}

/* How the oracle computes signed distance for one pair, resolved once by the
 capability probe: no per-query dispatch decisions. */
enum class DistanceRoute {
  /* QueryObject::ComputeSignedDistancePairClosestPoints. */
  kNative,
  /* Analytic halfspace support-function fallback; geometry `a` is the
   halfspace. */
  kHalfSpaceA,
  /* Same, geometry `b` is the halfspace. */
  kHalfSpaceB,
};

/* One unfiltered proximity geometry pair with its pre-resolved distance
 route. The threshold is not carried here: it is Options::margin, uniform over
 the pairs. */
struct PairRecord {
  geometry::GeometryId a;
  geometry::GeometryId b;
  multibody::BodyIndex body_a;
  multibody::BodyIndex body_b;
  DistanceRoute route{DistanceRoute::kNative};
};

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
