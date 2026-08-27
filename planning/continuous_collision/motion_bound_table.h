#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/continuous_collision/bounding_sphere.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** Per-pair motion-bound coefficients in CSR layout: for pair index k, a
contiguous span of (position-coordinate index j, λ(j, p)) entries over J(p),
the coordinates that change the pair's relative pose. λ is meters of worst-case
displacement of the pair's distal side per unit change of coordinate j, valid
for every configuration in the trajectory's global control-point box.

Each pair also carries a scalar carveout_slack(p), the residual motion of the
coordinates the constant-coordinate carve-out removed from J(p). MotionBound()
charges it unconditionally, which is what makes Δ_p an upper bound on the
pair's relative motion over the whole trajectory.
@ingroup planning_collision_checker */
class MotionBoundTable {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MotionBoundTable);

  /** Constructs an empty table (zero pairs). */
  MotionBoundTable() = default;

  /** Constructs the CSR table directly from its four arrays.
  @param row_start   Size num_pairs + 1, starting at 0 and non-decreasing;
                     row_start.back() is the total entry count.
  @param coord       Position-coordinate index of every entry.
  @param lambda      λ of every entry, element for element with `coord`.
  @param carveout_slack  One residual per pair.
  @throws std::exception if the arrays do not satisfy those invariants. */
  MotionBoundTable(std::vector<int> row_start, std::vector<int> coord,
                   std::vector<double> lambda,
                   std::vector<double> carveout_slack);

  int num_pairs() const { return static_cast<int>(row_start_.size()) - 1; }

  /** True iff J(p) is empty after the constant-coordinate carve-out: no
  coordinate the trajectory *moves* changes this pair's relative pose, so it
  is checked once. Note that "static" does not mean "immobile": a static pair
  can still drift by carveout_slack(p), which callers that shortcut
  MotionBound() for such a pair must charge themselves.
  @pre 0 <= pair_index < num_pairs(). */
  bool pair_is_static(int pair_index) const {
    return row_start_[pair_index] == row_start_[pair_index + 1];
  }

  /** Δ_p(ν) = carveout_slack(p) + Σ_{j ∈ J(p)} λ(j,p) · w_j: a sparse dot
  product against the node's per-coordinate deviations w, plus the carved
  coordinates' residual.
  @pre 0 <= pair_index < num_pairs().
  @pre w.size() equals the plant's number of position coordinates. */
  double MotionBound(int pair_index, const Eigen::VectorXd& w) const {
    double delta = carveout_slack_[pair_index];
    for (int e = row_start_[pair_index]; e < row_start_[pair_index + 1]; ++e) {
      delta += lambda_[e] * w[coord_[e]];
    }
    return delta;
  }

  /** Σ over the coordinates of J_topo(p) that the carve-out removed of
  λ̃_j · (global_upper_j − global_lower_j): an upper bound on how far this
  pair's two geometries can move relative to each other purely through the
  coordinates the table no longer tracks. The carve-out's "constant" is a
  tolerance, a coordinate whose global control-box range is at most
  Options::continuity_tolerance, so this is zero exactly when every carved
  coordinate is exactly constant.
  @pre 0 <= pair_index < num_pairs(). */
  double carveout_slack(int pair_index) const {
    return carveout_slack_[pair_index];
  }

  /** Introspection for tests: the (coordinate, λ) entries of one pair,
  ordered by increasing coordinate index.
  @throws std::exception if pair_index is outside [0, num_pairs()). */
  std::vector<std::pair<int, double>> GetEntries(int pair_index) const;

  /** Total number of (coordinate, λ) entries over all pairs. */
  int num_entries() const { return static_cast<int>(coord_.size()); }

 private:
  std::vector<int> row_start_{0};
  std::vector<int> coord_;
  std::vector<double> lambda_;
  std::vector<double> carveout_slack_;
};

/** Construction-time kinematic analysis of a plant: joint classification,
per-hop fixed-transform translations, per-body proximity
geometry bounding spheres, and subtree tables for J(p). Thread-compatible;
all methods are const after construction and hold no mutable state, so
concurrent ComputeMotionBoundTable() calls are safe.

Typical use by the certifier:
- once, at checker construction:   KinematicsEngine engine(model);
                                   engine.body_spheres(b) for the prefilter;
- once per Check* call:            engine.ComputeMotionBoundTable(path, pairs);
- once per node, per pair:         table.MotionBound(pair_index, w).
@ingroup planning_collision_checker */
class KinematicsEngine {
 public:
  /* Copies alias the same model: the RobotDiagram passed to the constructor
   must outlive every copy, not just the original. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicsEngine);

  /** Builds topology tables and per-body geometry bounding spheres.
  Classification only; unsupported joint types throw later, and only if a
  given path actually moves them.

  `model` is aliased and must outlive this object.

  @throws std::exception if the plant is not finalized.
  @throws std::exception if a HalfSpace geometry is on the *distal* side of a
  rotational coordinate relative to an unfiltered partner, whose reach is then
  unbounded. A HalfSpace that is merely the static partner of a rotating body,
  such as the anchored ground plane under a robot arm, is accepted: λ then
  bounds the partner's points, and signed distance is symmetric, so the
  certificate still holds.
  @throws std::exception if a joint is "reversed", i.e. its declared parent
  body is outboard of its declared child body in the multibody tree.
  @throws std::exception if a joint closes a kinematic loop, or if the plant's
  kinematically-affected sets disagree with the world-rooted tree walk.
  @throws std::exception if any proximity geometry has a shape
  ComputeBoundingSphere() rejects. */
  explicit KinematicsEngine(const RobotDiagram<double>& model);

  /** The position-coordinate indices whose motion changes the relative pose
  of the two bodies (J(p) before any carve-out), from topology alone. Sorted
  ascending. */
  std::vector<int> CoordinatesAffectingPair(multibody::BodyIndex body_a,
                                            multibody::BodyIndex body_b) const;

  /** Assembles the λ CSR table for `pairs` given the path's global
  control-point box; prismatic chain contributions use the box, so the bound is
  trajectory-adaptive. Coordinates flagged constant by the path are removed
  from every J(p), and their residual motion inside the box is charged to
  MotionBoundTable::carveout_slack() instead.
  @throws std::exception if the path's number of positions differs from the
  plant's.
  @throws std::exception naming the joint if the path moves a coordinate of
  an unsupported joint type (quaternion floating, ball). */
  MotionBoundTable ComputeMotionBoundTable(
      const PiecewiseBezierPath& path, const std::vector<PairId>& pairs) const;

  /** Raw-data overload of the above, for callers (and tests) that already
  hold the trajectory's global control-point box. `lower` and `upper` are the
  per-coordinate box bounds and `constant_coordinates` flags the coordinates
  the path cannot change; all three have size num_positions(). A coordinate
  flagged constant still contributes (upper − lower) worth of residual motion
  to the pair's carve-out slack, so the two arguments must describe the same
  trajectory: flagging a coordinate constant does not license widening its
  box.
  @throws std::exception on a size mismatch, an empty box (lower > upper), a
  non-finite bound, a moving coordinate of an unsupported joint type, a pair
  whose distal side carries a HalfSpace across a rotational coordinate, or a
  pair whose distal side carries a HalfSpace across a rotational coordinate
  that is constant only to within a tolerance (a HalfSpace has no finite
  reach, so such a coordinate must be *exactly* constant). */
  MotionBoundTable ComputeMotionBoundTable(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
      const std::vector<bool>& constant_coordinates,
      const std::vector<PairId>& pairs) const;

  /** Bounding spheres (body frame) of every proximity geometry of `body`,
  used by the reach chain start and by the certifier's sphere prefilter.
  HalfSpace geometries have no bounding sphere and are omitted. */
  const std::vector<BoundingSphere>& body_spheres(
      multibody::BodyIndex body) const;

  /** The geometry ids matching body_spheres(body), element for element. */
  const std::vector<geometry::GeometryId>& body_sphere_geometries(
      multibody::BodyIndex body) const;

  /** The bounding sphere (in its body's frame) of one proximity geometry.
  @throws std::exception if `id` is not a proximity geometry of this model or
  is a HalfSpace (which has none). */
  const BoundingSphere& geometry_sphere(geometry::GeometryId id) const;

  /** True iff `body` carries at least one HalfSpace proximity geometry. */
  bool body_has_halfspace(multibody::BodyIndex body) const;

  /** Radius, about the body frame origin, of a sphere containing every
  proximity geometry of `body`; this is the start of the reach chain. Zero for
  a body with no (non-HalfSpace) proximity geometry. */
  double body_radius(multibody::BodyIndex body) const;

  int num_positions() const { return num_positions_; }

  const multibody::MultibodyPlant<double>& plant() const { return *plant_; }

 private:
  /* The λ rule a joint's coordinates follow. */
  enum class JointKind {
    kWeld,        // 0 dof; contributes fixed translations to reach only.
    kRevolute,    // λ = r.
    kPrismatic,   // λ = 1.
    kPlanar,      // λ = 1 (x, y), λ = r (θ).
    kScrew,       // λ = r + |pitch| / 2π.
    kUnsupported  // Throws if the path moves any of its coordinates.
  };

  /* The λ̃ rule a single *coordinate* follows when the carve-out has removed
   it from J(p) but it is not exactly constant. It is per coordinate, not per
   joint, because the joint kinds the carve-out admits (floating bases in
   particular) mix rotation and translation coordinates inside one joint. For
   the four supported kinds this reproduces JointKind's λ exactly; it extends
   to the kUnsupported kinds, which have no λ but do have a λ̃. */
  enum class CoordRule {
    kTranslation,  // λ̃ = 1: a unit translation of the outboard frame.
    kRotation,     // λ̃ = r: a unit-angle rotation about an axis through Mo.
    kScrewCoord,   // λ̃ = r + |pitch| / 2π.
    kQuaternion,   // λ̃ = 2r / m; see ComputeMotionBoundTable() for the proof.
  };

  static bool IsRotationalRule(CoordRule rule) {
    return rule != CoordRule::kTranslation;
  }

  /* One tree edge, oriented from its outboard body toward the world. */
  struct JointRecord {
    multibody::JointIndex index;
    std::string name;
    std::string type_name;
    JointKind kind{JointKind::kUnsupported};
    int position_start{0};
    int num_positions{0};
    /* Tree-inboard / tree-outboard bodies (from the world-rooted walk, which
     is cross-checked against Drake's own subtree query). */
    multibody::BodyIndex inboard;
    multibody::BodyIndex outboard;
    /* ‖p_PF‖ + ‖p_CM‖ (+ ‖p_FM‖ for a weld): the configuration-independent
     part of one hop from the outboard body frame to the inboard body frame. */
    double fixed_hop{0.0};
    /* ‖p_CM‖ alone: the top-of-chain term, from the outboard body's frame
     origin to the joint's M-frame origin (the point that stays fixed when
     this joint's coordinates move). */
    double p_CM_norm{0.0};
    /* Screw pitch (meters of translation per full revolution). */
    double screw_pitch{0.0};
    /* Position-coordinate offsets, relative to position_start, holding a
     translation of X_FM for the unsupported-but-constant carve-out. */
    std::vector<int> translation_offsets;
    /* False for a joint type this library has never been taught, whose X_FM
     translation cannot be bounded from the control box at all. */
    bool translation_offsets_known{false};
    /* One CoordRule per position coordinate of this joint (size
     num_positions), used only for coordinates the carve-out removed. Empty
     exactly when translation_offsets_known is false, i.e. for a joint type
     this library cannot bound even when held constant. */
    std::vector<CoordRule> coord_rules;
    /* Subtree membership: bodies whose pose depends on this joint's
     coordinates. Empty for welds. */
    std::vector<bool> subtree;
  };

  /* Returns the joint ordinal (index into joints_) of `body`'s inboard joint,
   or -1 for the world body. */
  int inboard_joint_of(multibody::BodyIndex body) const {
    return inboard_joint_[body];
  }

  /* r(joint_ord, body): an upper bound, valid over the whole control box, on
   the distance from the joint's M-frame origin to any point of `body`'s
   proximity geometry. `box_hop` holds the per-call, box-dependent part of
   each joint's hop translation. Requires `body` to be in the joint's
   subtree. */
  double Reach(int joint_ord, multibody::BodyIndex body,
               const std::vector<double>& box_hop) const;

  void BuildTopology();
  void BuildGeometry();
  void CheckHalfSpaceRule() const;

  const RobotDiagram<double>* model_{};
  const multibody::MultibodyPlant<double>* plant_{};
  int num_positions_{0};
  int num_bodies_{0};

  std::vector<JointRecord> joints_;
  /* Ordinals of the joints with at least one position coordinate, sorted by
   position_start so that every J(p) comes out in ascending coordinate order. */
  std::vector<int> positioned_order_;
  /* JointIndex value -> ordinal into joints_, or -1. */
  std::vector<int> joint_ordinal_;
  /* BodyIndex value -> ordinal of its inboard joint, or -1 for the world. */
  std::vector<int> inboard_joint_;
  /* Position coordinate -> ordinal of the owning joint. */
  std::vector<int> coord_joint_;

  std::vector<std::vector<BoundingSphere>> body_spheres_;
  std::vector<std::vector<geometry::GeometryId>> body_sphere_geoms_;
  std::vector<double> body_radius_;
  std::vector<bool> body_has_halfspace_;
  std::vector<std::string> body_halfspace_name_;
  std::unordered_map<geometry::GeometryId, BoundingSphere> geometry_spheres_;
};

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
