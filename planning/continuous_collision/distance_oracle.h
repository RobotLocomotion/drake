#pragma once

// NOTE(interface): This header is owned by the distance module. The class
// and file names and the documented semantics are fixed; internal details
// may be refined by the implementation.

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/query_object.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** How the oracle computes signed distance for one pair, resolved once by
the capability probe (the geometry-support scope; the distance-oracle contract):
no per-query dispatch decisions. */
enum class DistanceRoute {
  /** QueryObject::ComputeSignedDistancePairClosestPoints. */
  kNative,
  /** Analytic halfspace support-function fallback; geometry `a` is the
  halfspace. */
  kHalfSpaceA,
  /** Same, geometry `b` is the halfspace. */
  kHalfSpaceB,
};

/** One unfiltered proximity pair with its pre-resolved distance route and
effective threshold m_p = margin + padding(p). */
struct PairRecord {
  PairId id;
  DistanceRoute route{DistanceRoute::kNative};
  /** Filled by the facade from margin + PaddingSpec. */
  double threshold{0.0};
};

/** Narrowphase distance abstraction (the distance-oracle contract). Stateless
per query and thread-compatible: configuration comes in via the caller's
QueryObject.

Contract: SignedDistance returns φ̂ with |φ̂ − φ_true| ≤ tolerance()
whenever φ_true is at or above −tolerance(), and returns a definitely
negative value when the shapes interpenetrate beyond tolerance. Only
over-reporting a distance at or above threshold could fake a certificate
(the soundness argument), which is why the capability probe keeps any
not-a-true-distance backend out of the loop entirely. */
class DistanceOracle {
 public:
  /** Runs the capability probe: enumerates the unfiltered proximity pairs
  from the model's SceneGraph inspector (collision filter state snapshotted
  at construction), classifies every (shape, shape) combination as
  {native, halfspace-fallback, unsupported}, and
  @throws std::exception immediately naming the offending geometries if any
  pair is unsupported (deformables; halfspace–halfspace). Never discovers an
  unsupported pair mid-certification. */
  DistanceOracle(const drake::planning::RobotDiagram<double>& model,
                 double query_tolerance);

  /** The unfiltered pairs found by the probe (thresholds default 0; the
  facade rewrites them from margin + padding). */
  const std::vector<PairRecord>& pairs() const { return pairs_; }

  /** Signed distance for one pair at the configuration already set in the
  context that produced `query_object`. Optionally reports world-frame
  closest points when the route provides them.

  `pair` need not be an element of pairs(): the facade copies the probe's
  records and rewrites their thresholds, so only `pair.id` and `pair.route`
  are read here. Both routes always fill the optional out-params.

  @throws std::exception if `pair` carries a halfspace route but its
  geometries were not classified by this oracle's capability probe (i.e. the
  record did not come from pairs()). */
  double SignedDistance(
      const drake::geometry::QueryObject<double>& query_object,
      const PairRecord& pair, Eigen::Vector3d* nearest_a_W = nullptr,
      Eigen::Vector3d* nearest_b_W = nullptr) const;

  /** τ used in the certificate arithmetic (the numerical policy). */
  double tolerance() const { return tolerance_; }

  /** Human-readable probe report: one line per distinct shape-type
  combination and its route (includes the "Mesh certified as convex hull"
  notices; the risk register). */
  std::string support_report() const;

 protected:
  std::vector<PairRecord> pairs_;
  double tolerance_{1e-6};

 private:
  /** Immutable capability-probe results: closed-form support data for every
  halfspace partner, the resolved per-shape-combination routes, and the
  rendered report. Held by shared_ptr so the oracle stays cheaply copyable
  and thread-compatible (the probe output is never mutated after
  construction). */
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
