#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_object.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** How the oracle computes signed distance for one pair, resolved once by the
capability probe: no per-query dispatch decisions.
@ingroup planning_collision_checker */
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
effective threshold m_p = margin + padding(p).
@ingroup planning_collision_checker */
struct PairRecord {
  PairId id;
  DistanceRoute route{DistanceRoute::kNative};
  /** Filled by the facade from margin + PaddingSpec. */
  double threshold{0.0};
};

/** Narrowphase distance abstraction. Stateless per query and
thread-compatible: configuration comes in via the caller's QueryObject.

Contract: SignedDistance returns ϕ̂ with |ϕ̂ − ϕ_true| ≤ tolerance()
whenever ϕ_true is at or above −tolerance(), and returns a definitely
negative value when the shapes interpenetrate beyond tolerance. Only
over-reporting a distance at or above threshold could fake a certificate,
which is why the capability probe keeps any not-a-true-distance backend out of
the loop entirely.

The collision filter state is snapshotted from the model inspector at
construction: pairs() is the set of pairs that were unfiltered *then*. Filter
changes applied to a Context afterwards are not observed, so a checker built
on this oracle keeps certifying the pair set it was constructed with.
@ingroup planning_collision_checker */
class DistanceOracle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DistanceOracle);

  /** Runs the capability probe: enumerates the unfiltered proximity pairs
  from the model's SceneGraph inspector (collision filter state snapshotted at
  construction) and classifies every (shape, shape) combination as native,
  halfspace-fallback or unsupported. An unsupported pair is reported here, so
  one is never discovered mid-certification.
  @throws std::exception naming the offending geometries if any pair is
  unsupported, i.e. involves a deformable geometry or is halfspace against
  halfspace. */
  DistanceOracle(const RobotDiagram<double>& model, double query_tolerance);

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
  double SignedDistance(const geometry::QueryObject<double>& query_object,
                        const PairRecord& pair,
                        Eigen::Vector3d* nearest_a_W = nullptr,
                        Eigen::Vector3d* nearest_b_W = nullptr) const;

  /** τ used in the certificate arithmetic. */
  double tolerance() const { return tolerance_; }

  /** Human-readable probe report: one line per distinct shape-type
  combination and its route, including the "Mesh certified as convex hull"
  notices. */
  const std::string& support_report() const;

 private:
  std::vector<PairRecord> pairs_;
  double tolerance_{1e-6};

  /* Immutable capability-probe results: closed-form support data for every
   halfspace partner, the resolved per-shape-combination routes, and the
   rendered report. Held by shared_ptr so the oracle stays cheaply copyable
   and thread-compatible; the probe output is never mutated after
   construction. */
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
