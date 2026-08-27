#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "drake/common/parallelism.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** Search modes for certification.
@ingroup planning_collision_checker */
enum class SearchMode {
  /** Return on the first definite violation; serial execution returns the
  earliest one in time. */
  kFindFirstViolation,
  /** Certify the full domain and return every violation / inconclusive
  region found (bounded by Options::max_reported_findings). */
  kCertifyAll,
};

/** Outcome of a certification run.
@ingroup planning_collision_checker */
enum class Verdict {
  /** Proof: every unfiltered pair keeps signed distance > margin + padding
  over the entire continuous time domain. */
  kCertifiedFree,
  /** An exactly-on-trajectory configuration violates the threshold. */
  kViolationFound,
  /** Subdivision hit the resolution floor with some pair's clearance within
  oracle tolerance of the threshold (grazing trajectory). */
  kInconclusive,
  /** The optional node budget was exhausted first. */
  kBudgetExhausted,
};

/** Options controlling one certification call.
@ingroup planning_collision_checker */
struct Options {
  /** Global clearance margin δ in meters. The certificate proves signed
  distance > margin + padding for every pair at every time. */
  double margin{0.0};
  /** Junction C0-continuity tolerance (per coordinate; modulo 2π for
  coordinates listed in continuous_revolute_indices). */
  double continuity_tolerance{1e-7};
  /** τ: the distance oracle's accuracy contract in meters. */
  double query_tolerance{1e-6};
  /** ε_slack: swallows floating-point noise in the bound arithmetic. */
  double certificate_slack{1e-9};
  /** Resolution floor as a fraction of a segment's parameter width; nodes
  narrower than this become kInconclusive findings instead of splitting. */
  double min_interval{1e-9};
  /** Position coordinates whose junction continuity is checked modulo 2π
  (GcsTrajectoryOptimization continuous-revolute convention).
  @see planning::trajectory_optimization::GetContinuousRevoluteJointIndices */
  std::vector<int> continuous_revolute_indices{};
  /** Maximum polynomial degree accepted for monomial→Bernstein conversion. */
  int max_conversion_degree{10};
  SearchMode mode{SearchMode::kCertifyAll};
  int max_reported_findings{32};
  /** Optional node budget; exceeded => Verdict::kBudgetExhausted. */
  std::optional<uint64_t> max_nodes{};
  /** If true, every certification event is recorded into a Certificate that
  VerifyCertificate() can independently replay. */
  bool emit_certificate{false};
  Parallelism parallelism{Parallelism::Max()};
};

/** Per-body-pair padding: the effective threshold for pair p is
margin + padding(p).

Which of the two scalars applies to a pair is decided by *anchoring*, from
plant topology alone. A body is anchored iff no position coordinate of the
plant changes its pose relative to the world, i.e. the world body itself and
everything welded to it directly or transitively. A pair is a self-collision
pair iff both of its bodies are non-anchored, and an environment pair
otherwise. The rule never depends on which trajectory is being checked.
@ingroup planning_collision_checker */
struct PaddingSpec {
  /** Padding for robot-vs-environment pairs, i.e. pairs with at least one
  anchored body. */
  double env_padding{0.0};
  /** Padding for robot-vs-robot (self-collision) pairs, i.e. pairs whose two
  bodies are both non-anchored. */
  double self_padding{0.0};
  /** Optional dense symmetric matrix indexed by BodyIndex, sized
  num_bodies × num_bodies. Entry (a, b) overrides the scalars for that body
  pair; a NaN entry means "not covered", and that pair falls back to
  env_padding / self_padding. */
  std::optional<Eigen::MatrixXd> per_body_pair{};
};

/** Identifies an unfiltered proximity geometry pair.
@ingroup planning_collision_checker */
struct PairId {
  geometry::GeometryId a;
  geometry::GeometryId b;
  multibody::BodyIndex body_a;
  multibody::BodyIndex body_b;
};

/** One violation or inconclusive record.
@ingroup planning_collision_checker */
struct Finding {
  /** Trajectory time of the witness configuration. */
  double time{};
  /** The witness configuration, exactly on the trajectory. */
  Eigen::VectorXd q;
  PairId pair;
  /** Oracle signed distance at q for this pair. */
  double distance{};
  /** Motion bound Δ_p at the terminal node (0 for breakpoint findings). */
  double motion_bound{};
  /** true => definite violation; false => grazing / inconclusive. */
  bool definite{};
  /** Closest points in world frame at q, when the narrowphase provides
  them (violation findings; planners use these to push trajectories out
  of collision). */
  std::optional<Eigen::Vector3d> nearest_a_W{};
  std::optional<Eigen::Vector3d> nearest_b_W{};
};

/** Cost accounting for one certification call.
@ingroup planning_collision_checker */
struct Statistics {
  uint64_t nodes{0};
  uint64_t narrowphase_queries{0};
  uint64_t sphere_certifications{0};
  int max_depth{0};
  double wall_time_s{0.0};
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
