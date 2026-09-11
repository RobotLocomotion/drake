#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_object.h"
#include "drake/planning/continuous_collision/internal.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/* Narrowphase distance abstraction. Stateless per query and
thread-compatible: configuration comes in via the caller's QueryObject.

Contract: SignedDistance returns ϕ̂ with |ϕ̂ − ϕ_true| ≤ τ_p whenever ϕ_true is
at or above −τ_p, and returns a definitely negative value when the shapes
interpenetrate beyond τ_p. Only over-reporting a distance at or above the
threshold could fake a certificate, which is why the capability probe keeps any
not-a-true-distance backend out of the loop entirely.

The collision filter state is snapshotted from the model inspector at
construction: pairs() is the set of pairs that were unfiltered *then*. Filter
changes applied to a Context afterwards are not observed, so a checker built
on this oracle keeps certifying the pair set it was constructed with. */
class DistanceOracle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DistanceOracle);

  /* Runs the capability probe: enumerates the unfiltered proximity pairs
  from the model's SceneGraph inspector (collision filter state snapshotted at
  construction) and classifies every (shape, shape) combination as native,
  halfspace-fallback or unsupported. An unsupported pair is reported here, so
  one is never discovered mid-certification.
  @throws std::exception naming the offending geometries if any pair is
  unsupported, i.e. involves a deformable geometry or is halfspace against
  halfspace, or if this Drake build cannot compute signed distance for one of
  the shape combinations present. */
  explicit DistanceOracle(const RobotDiagram<double>& model);

  /* The unfiltered pairs found by the probe. */
  const std::vector<PairRecord>& pairs() const { return pairs_; }

  /* Signed distance for one pair at the configuration already set in the
  context that produced `query_object`, optionally reporting the world-frame
  closest points. Both routes always fill the optional out-params.

  @throws std::exception if `pair` carries a halfspace route but its
  geometries were not classified by this oracle's capability probe. */
  double SignedDistance(const geometry::QueryObject<double>& query_object,
                        const PairRecord& pair,
                        Eigen::Vector3d* nearest_a_W = nullptr,
                        Eigen::Vector3d* nearest_b_W = nullptr) const;

 private:
  std::vector<PairRecord> pairs_;

  /* Immutable capability-probe results: closed-form support data for every
   halfspace partner. Held by shared_ptr so the oracle stays cheaply copyable
   and thread-compatible; the probe output is never mutated after
   construction. */
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
