#pragma once

// NOTE(interface): This header is owned by the kinematics module. The class
// and file names and the documented semantics are fixed; internal details
// (private members, helper structs) may be refined by the implementation.

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/planning/certified_ccd/bounding_sphere.h"
#include "drake/planning/certified_ccd/options.h"
#include "drake/planning/certified_ccd/piecewise_bezier_path.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace certified_ccd {

/** Per-pair motion-bound coefficients in CSR layout (the displacement lemma):
for pair index k, a contiguous span of (position-coordinate index j, λ(j, p))
entries over J(p), the coordinates that change the pair's relative pose. λ has
units of meters of worst-case point displacement of the pair's distal side per
unit change of coordinate j, valid for every configuration in the
trajectory's global control-point box. */
class MotionBoundTable {
 public:
  int num_pairs() const { return static_cast<int>(row_start_.size()) - 1; }

  /** True iff J(p) is empty after the constant-coordinate carve-out: the
  trajectory cannot change this pair's status, so it is checked once. */
  bool pair_is_static(int pair_index) const {
    return row_start_[pair_index] == row_start_[pair_index + 1];
  }

  /** Δ_p(ν) = Σ_{j ∈ J(p)} λ(j,p) · w_j — a sparse dot product against the
  node's per-coordinate deviations w (the interval certificate, requirement P3).
*/
  double MotionBound(int pair_index, const Eigen::VectorXd& w) const {
    double delta = 0.0;
    for (int e = row_start_[pair_index]; e < row_start_[pair_index + 1]; ++e) {
      delta += lambda_[e] * w[coord_[e]];
    }
    return delta;
  }

  /** Introspection for tests: the (coordinate, λ) entries of one pair,
  ordered by increasing coordinate index. */
  std::vector<std::pair<int, double>> entries(int pair_index) const;

  /** Total number of (coordinate, λ) entries over all pairs. */
  int num_entries() const { return static_cast<int>(coord_.size()); }

  /** Builder access (kinematics module internals only). */
  std::vector<int>& mutable_row_start() { return row_start_; }
  std::vector<int>& mutable_coord() { return coord_; }
  std::vector<double>& mutable_lambda() { return lambda_; }

 private:
  std::vector<int> row_start_{0};
  std::vector<int> coord_;
  std::vector<double> lambda_;
};

/** Construction-time kinematic analysis of a plant (the displacement lemma):
joint classification, per-hop fixed-transform translations, per-body proximity
geometry bounding spheres, and subtree tables for J(p). Thread-compatible;
all methods are const after construction and hold no mutable state, so
concurrent ComputeMotionBoundTable() calls are safe.

Typical use by the certifier:
- once, at checker construction:   KinematicsEngine engine(model);
                                   engine.body_spheres(b) for the prefilter;
- once per Check* call:            engine.ComputeMotionBoundTable(path, pairs);
- once per node, per pair:         table.MotionBound(pair_index, w). */
class KinematicsEngine {
 public:
  /** Builds topology tables and per-body geometry bounding spheres.
  Classification only; unsupported joint types throw later, and only if a
  given path actually moves them (constant-coordinate carve-out, the
  joint-support scope).

  `model` is aliased and must outlive this object.

  @throws std::exception if a HalfSpace geometry is on the *distal* side of a
  rotational coordinate relative to an unfiltered partner (unbounded reach).
  A HalfSpace that is merely the static partner of a rotating body — the
  anchored ground plane under a robot arm, the overwhelmingly common case — is
  accepted: λ then bounds the partner's points, and signed distance is
  symmetric, so the certificate still holds.
  @throws std::exception if the plant is not finalized, if a joint is
  "reversed" (its declared parent body is outboard of its declared child body
  in the multibody tree — a documented v1 exclusion), or if any proximity
  geometry has a shape ComputeBoundingSphere() rejects. */
  explicit KinematicsEngine(const drake::planning::RobotDiagram<double>& model);

  /** The position-coordinate indices whose motion changes the relative pose
  of the two bodies (J(p) before any carve-out), from topology alone. Sorted
  ascending. */
  std::vector<int> CoordinatesAffectingPair(
      drake::multibody::BodyIndex body_a,
      drake::multibody::BodyIndex body_b) const;

  /** Assembles the λ CSR table for `pairs` given the path's global
  control-point box (prismatic chain contributions use the box, so the bound
  is trajectory-adaptive; the displacement lemma). Coordinates flagged constant
  by the path are removed from every J(p).
  @throws std::exception naming the joint if the path moves a coordinate of
  an unsupported joint type (quaternion floating, ball). */
  MotionBoundTable ComputeMotionBoundTable(
      const PiecewiseBezierPath& path, const std::vector<PairId>& pairs) const;

  /** Raw-data overload of the above, for callers (and tests) that already
  hold the trajectory's global control-point box. `lower` and `upper` are the
  per-coordinate box bounds and `constant_coordinates` flags the coordinates
  the path cannot change; all three have size num_positions().
  @throws std::exception on a size mismatch, an empty box (lower > upper), a
  non-finite bound, a moving coordinate of an unsupported joint type, or a
  pair whose distal side carries a HalfSpace across a rotational coordinate. */
  MotionBoundTable ComputeMotionBoundTable(
      const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
      const std::vector<bool>& constant_coordinates,
      const std::vector<PairId>& pairs) const;

  /** Bounding spheres (body frame) of every proximity geometry of `body`,
  used by the reach chain start and by the certifier's sphere prefilter.
  HalfSpace geometries have no bounding sphere and are omitted. */
  const std::vector<BoundingSphere>& body_spheres(
      drake::multibody::BodyIndex body) const;

  /** The geometry ids matching body_spheres(body), element for element. */
  const std::vector<drake::geometry::GeometryId>& body_sphere_geometries(
      drake::multibody::BodyIndex body) const;

  /** The bounding sphere (in its body's frame) of one proximity geometry.
  @throws std::exception if `id` is not a proximity geometry of this model or
  is a HalfSpace (which has none). */
  const BoundingSphere& geometry_sphere(drake::geometry::GeometryId id) const;

  /** True iff `body` carries at least one HalfSpace proximity geometry. */
  bool body_has_halfspace(drake::multibody::BodyIndex body) const;

  /** Radius, about the body frame origin, of a sphere containing every
  proximity geometry of `body` — the start of the reach chain. Zero for a
  body with no (non-HalfSpace) proximity geometry. */
  double body_radius(drake::multibody::BodyIndex body) const;

  int num_positions() const { return num_positions_; }

  const drake::multibody::MultibodyPlant<double>& plant() const {
    return *plant_;
  }

 private:
  /* The λ rule a joint's coordinates follow (the displacement lemma; the
   * joint-support scope). */
  enum class JointKind {
    kWeld,        // 0 dof; contributes fixed translations to reach only.
    kRevolute,    // λ = r.
    kPrismatic,   // λ = 1.
    kPlanar,      // λ = 1 (x, y), λ = r (θ).
    kScrew,       // λ = r + |pitch| / 2π.
    kUnsupported  // Throws if the path moves any of its coordinates.
  };

  /* One tree edge, oriented from its outboard body toward the world. */
  struct JointRecord {
    drake::multibody::JointIndex index;
    std::string name;
    std::string type_name;
    JointKind kind{JointKind::kUnsupported};
    int position_start{0};
    int num_positions{0};
    /* Tree-inboard / tree-outboard bodies (from the world-rooted walk, which
     is cross-checked against Drake's own subtree query). */
    drake::multibody::BodyIndex inboard;
    drake::multibody::BodyIndex outboard;
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
    /* Subtree membership: bodies whose pose depends on this joint's
     coordinates. Empty for welds. */
    std::vector<bool> subtree;
  };

  /* Returns the joint ordinal (index into joints_) of `body`'s inboard joint,
   or -1 for the world body. */
  int inboard_joint_of(drake::multibody::BodyIndex body) const {
    return inboard_joint_[body];
  }

  /* r(joint_ord, body): an upper bound, valid over the whole control box, on
   the distance from the joint's M-frame origin to any point of `body`'s
   proximity geometry. `box_hop` holds the per-call, box-dependent part of
   each joint's hop translation. Requires `body` to be in the joint's
   subtree. */
  double Reach(int joint_ord, drake::multibody::BodyIndex body,
               const std::vector<double>& box_hop) const;

  void BuildTopology();
  void BuildGeometry();
  void CheckHalfSpaceRule() const;

  const drake::planning::RobotDiagram<double>* model_{};
  const drake::multibody::MultibodyPlant<double>* plant_{};
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
  std::vector<std::vector<drake::geometry::GeometryId>> body_sphere_geoms_;
  std::vector<double> body_radius_;
  std::vector<bool> body_has_halfspace_;
  std::vector<std::string> body_halfspace_name_;
  std::unordered_map<drake::geometry::GeometryId, BoundingSphere>
      geometry_spheres_;
};

}  // namespace certified_ccd
}  // namespace planning
}  // namespace drake
