#include "drake/planning/continuous_collision/motion_bound_table.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace planning {
namespace continuous_collision {

using drake::geometry::GeometryId;
using drake::geometry::HalfSpace;
using drake::geometry::Role;
using drake::geometry::Shape;
using drake::math::RigidTransform;
using drake::multibody::BodyIndex;
using drake::multibody::Joint;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::ScrewJoint;
using drake::multibody::WeldJoint;

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

bool IsHalfSpace(const Shape& shape) {
  return dynamic_cast<const HalfSpace*>(&shape) != nullptr;
}

}  // namespace

MotionBoundTable::MotionBoundTable(std::vector<int> row_start,
                                   std::vector<int> coord,
                                   std::vector<double> lambda,
                                   std::vector<double> carveout_slack)
    : row_start_(std::move(row_start)),
      coord_(std::move(coord)),
      lambda_(std::move(lambda)),
      carveout_slack_(std::move(carveout_slack)) {
  DRAKE_THROW_UNLESS(!row_start_.empty());
  DRAKE_THROW_UNLESS(row_start_.front() == 0);
  for (int i = 1; i < static_cast<int>(row_start_.size()); ++i) {
    DRAKE_THROW_UNLESS(row_start_[i] >= row_start_[i - 1]);
  }
  DRAKE_THROW_UNLESS(coord_.size() == lambda_.size());
  DRAKE_THROW_UNLESS(static_cast<int>(coord_.size()) == row_start_.back());
  DRAKE_THROW_UNLESS(carveout_slack_.size() + 1 == row_start_.size());
}

std::vector<std::pair<int, double>> MotionBoundTable::GetEntries(
    int pair_index) const {
  DRAKE_THROW_UNLESS(pair_index >= 0 && pair_index < num_pairs());
  std::vector<std::pair<int, double>> out;
  out.reserve(row_start_[pair_index + 1] - row_start_[pair_index]);
  for (int e = row_start_[pair_index]; e < row_start_[pair_index + 1]; ++e) {
    out.emplace_back(coord_[e], lambda_[e]);
  }
  return out;
}

KinematicsEngine::KinematicsEngine(const RobotDiagram<double>& model)
    : model_(&model), plant_(&model.plant()) {
  if (!plant_->is_finalized()) {
    throw std::runtime_error(
        "KinematicsEngine: requires a finalized "
        "MultibodyPlant; call Finalize() before building the checker.");
  }
  BuildTopology();
  BuildGeometry();
  CheckHalfSpaceRule();
}

void KinematicsEngine::BuildTopology() {
  const MultibodyPlant<double>& plant = *plant_;
  num_positions_ = plant.num_positions();
  num_bodies_ = plant.num_bodies();

  // ------------------------------------------------------------------
  // 1. Classify every joint (welds included) and cache its per-hop fixed
  //    translation norms.
  // ------------------------------------------------------------------
  const std::vector<JointIndex>& joint_indices = plant.GetJointIndices();
  int max_joint_index = -1;
  for (JointIndex ji : joint_indices) {
    max_joint_index = std::max(max_joint_index, static_cast<int>(ji));
  }
  joint_ordinal_.assign(max_joint_index + 1, -1);
  joints_.clear();
  joints_.reserve(joint_indices.size());

  for (JointIndex ji : joint_indices) {
    const Joint<double>& joint = plant.get_joint(ji);
    JointRecord rec;
    rec.index = ji;
    rec.name = joint.name();
    rec.type_name = joint.type_name();
    rec.num_positions = joint.num_positions();
    rec.position_start = rec.num_positions > 0 ? joint.position_start() : 0;

    // `coord_rules` classifies each coordinate for the carve-out residual
    // (see ComputeMotionBoundTable). For the supported kinds it mirrors the
    // λ switch in the CSR assembly, coordinate for coordinate; the two must
    // agree, and the property test in test/motion_bound_test.cc pins that.
    using R = CoordRule;
    bool translation_known = false;
    if (rec.type_name == WeldJoint<double>::kTypeName) {
      rec.kind = JointKind::kWeld;
      translation_known = true;
    } else if (rec.type_name == "revolute") {
      rec.kind = JointKind::kRevolute;
      rec.coord_rules = {R::kRotation};
      translation_known = true;
    } else if (rec.type_name == "prismatic") {
      rec.kind = JointKind::kPrismatic;
      rec.coord_rules = {R::kTranslation};
      translation_known = true;
    } else if (rec.type_name == "planar") {
      rec.kind = JointKind::kPlanar;
      // q = (x, y, θ); see PlanarJoint's class documentation.
      rec.coord_rules = {R::kTranslation, R::kTranslation, R::kRotation};
      translation_known = true;
    } else if (rec.type_name == ScrewJoint<double>::kTypeName) {
      rec.kind = JointKind::kScrew;
      rec.screw_pitch =
          dynamic_cast<const ScrewJoint<double>&>(joint).screw_pitch();
      rec.coord_rules = {R::kScrewCoord};
      translation_known = true;
    } else if (rec.type_name == "quaternion_floating") {
      // q = (q_FM wxyz, p_FM): the translation lives in coordinates 4..6.
      rec.kind = JointKind::kUnsupported;
      rec.translation_offsets = {4, 5, 6};
      rec.coord_rules = {R::kQuaternion, R::kQuaternion,  R::kQuaternion,
                         R::kQuaternion, R::kTranslation, R::kTranslation,
                         R::kTranslation};
      translation_known = true;
    } else if (rec.type_name == "rpy_floating") {
      // q = (rpy, p_FM): the translation lives in coordinates 3..5.
      rec.kind = JointKind::kUnsupported;
      rec.translation_offsets = {3, 4, 5};
      rec.coord_rules = {R::kRotation,    R::kRotation,    R::kRotation,
                         R::kTranslation, R::kTranslation, R::kTranslation};
      translation_known = true;
    } else if (rec.type_name == "ball_rpy" || rec.type_name == "universal") {
      // Pure rotation about coincident origins: X_FM has zero translation.
      rec.kind = JointKind::kUnsupported;
      rec.coord_rules.assign(rec.num_positions, R::kRotation);
      translation_known = true;
    } else {
      // A shape of joint this library has never been taught. It cannot even
      // contribute a chain hop safely, so it is rejected unconditionally in
      // ComputeMotionBoundTable(). It gets no coord_rules either: without
      // knowing what its coordinates *are*, no λ̃ can be written down.
      rec.kind = JointKind::kUnsupported;
      translation_known = false;
    }
    rec.translation_offsets_known = translation_known;
    if (translation_known && rec.num_positions > 0) {
      // Every coordinate of a joint we admit must have a carve-out rule, or
      // a carved coordinate could slip through uncharged.
      DRAKE_DEMAND(static_cast<int>(rec.coord_rules.size()) ==
                   rec.num_positions);
    }

    // Frame offsets: F = frame_on_parent (Jp), M = frame_on_child (Jc).
    // ‖p_PF‖ and ‖p_CM‖ are the two configuration-independent legs of one hop
    // across this joint; the middle leg is the translation of X_FM, which is
    // zero for a revolute, fixed for a weld, and box-bounded otherwise.
    RigidTransform<double> X_PF;
    RigidTransform<double> X_CM;
    try {
      X_PF = joint.frame_on_parent().GetFixedPoseInBodyFrame();
      X_CM = joint.frame_on_child().GetFixedPoseInBodyFrame();
    } catch (const std::exception& e) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: joint '{}' ({}) is mounted on a frame whose pose "
          "in its body is not fixed, so its chain contribution to the reach "
          "bound cannot be computed at construction time. Mount joints on body "
          "frames or FixedOffsetFrames. Underlying error: {}",
          rec.name, rec.type_name, e.what()));
    }
    rec.p_CM_norm = X_CM.translation().norm();
    rec.fixed_hop = rec.p_CM_norm + X_PF.translation().norm();
    if (rec.kind == JointKind::kWeld) {
      rec.fixed_hop += dynamic_cast<const WeldJoint<double>&>(joint)
                           .X_FM()
                           .translation()
                           .norm();
    }

    joint_ordinal_[ji] = static_cast<int>(joints_.size());
    joints_.push_back(std::move(rec));
  }

  // ------------------------------------------------------------------
  // 2. Orient the joint graph into the world-rooted multibody tree by a
  //    breadth-first walk from the world over the (body, joint) graph. Post
  //    Finalize() every non-world body has exactly one inboard joint
  //    (ephemeral floating joints included), so the walk is well defined.
  //
  //    The walk is what supplies the inboard/outboard orientation and the
  //    per-hop reach data, neither of which the plant exposes. The descendant
  //    sets it implies are *not* what the λ table then uses: step 3 takes each
  //    joint's subtree from the plant's own GetBodiesKinematicallyAffectedBy()
  //    and throws if the two disagree. The walk is therefore an independent
  //    cross-check of Drake's answer rather than a substitute for it.
  // ------------------------------------------------------------------
  std::vector<std::vector<int>> incident(num_bodies_);
  for (int k = 0; k < static_cast<int>(joints_.size()); ++k) {
    const Joint<double>& joint = plant.get_joint(joints_[k].index);
    incident[joint.parent_body().index()].push_back(k);
    incident[joint.child_body().index()].push_back(k);
  }

  inboard_joint_.assign(num_bodies_, -1);
  std::vector<bool> visited(num_bodies_, false);
  const BodyIndex world = plant.world_body().index();
  visited[world] = true;
  std::queue<BodyIndex> bfs;
  bfs.push(world);
  while (!bfs.empty()) {
    const BodyIndex b = bfs.front();
    bfs.pop();
    for (int k : incident[b]) {
      const Joint<double>& joint = plant.get_joint(joints_[k].index);
      const BodyIndex parent = joint.parent_body().index();
      const BodyIndex child = joint.child_body().index();
      const BodyIndex other = (parent == b) ? child : parent;
      if (visited[other]) continue;
      visited[other] = true;
      inboard_joint_[other] = k;
      joints_[k].inboard = b;
      joints_[k].outboard = other;
      bfs.push(other);
    }
  }
  for (int b = 0; b < num_bodies_; ++b) {
    if (!visited[b]) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: body '{}' is not connected to the world through "
          "the plant's joints; the kinematics module requires the single "
          "world-rooted tree a finalized MultibodyPlant provides.",
          plant.get_body(BodyIndex(b)).name()));
    }
  }
  for (const JointRecord& rec : joints_) {
    if (!rec.outboard.is_valid()) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: joint '{}' ({}) closes a kinematic loop (both of "
          "its bodies are already reachable from the world without it). The "
          "motion bound is defined over a single world-rooted tree, so the "
          "loop-closing joint must be removed; express the constraint it "
          "carried with a MultibodyPlant constraint instead.",
          rec.name, rec.type_name));
    }
  }

  // Descendant sets implied by the tree we just built: walking each body up to
  // the world marks it into every joint it hangs below. O(#bodies × depth).
  std::vector<std::vector<bool>> tree_subtree(
      joints_.size(), std::vector<bool>(num_bodies_, false));
  for (int b = 0; b < num_bodies_; ++b) {
    int k = inboard_joint_[b];
    int guard = 0;
    while (k >= 0) {
      tree_subtree[k][b] = true;
      k = inboard_joint_[joints_[k].inboard];
      DRAKE_DEMAND(++guard <= num_bodies_ + 1);
    }
  }

  // ------------------------------------------------------------------
  // 3. Subtree membership S_j for the positioned joints, taken from Drake so
  //    J(p) matches the plant's own notion of "kinematically affected", and
  //    cross-checked against the tree walk above (they must agree; a
  //    disagreement would mean the chain walk and J(p) disagree about which
  //    side is distal, which is a soundness hazard).
  // ------------------------------------------------------------------
  positioned_order_.clear();
  for (int k = 0; k < static_cast<int>(joints_.size()); ++k) {
    JointRecord& rec = joints_[k];
    const Joint<double>& joint = plant.get_joint(rec.index);
    if (joint.num_velocities() == 0) {
      DRAKE_DEMAND(rec.num_positions == 0);
      continue;
    }
    DRAKE_DEMAND(rec.num_positions > 0);
    if (rec.outboard != joint.child_body().index()) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: joint '{}' ({}) is reversed: its declared parent "
          "body '{}' is outboard of its declared child body '{}' in the "
          "multibody tree. The frame that stays fixed under the joint's motion "
          "is then on the outboard side, which the reach chain does not model. "
          "Re-declare the joint with the inboard body as its parent.",
          rec.name, rec.type_name, joint.parent_body().name(),
          joint.child_body().name()));
    }
    rec.subtree.assign(num_bodies_, false);
    for (BodyIndex b : plant.GetBodiesKinematicallyAffectedBy({rec.index})) {
      rec.subtree[b] = true;
    }
    if (rec.subtree != tree_subtree[k]) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: the plant's kinematically-affected set for joint "
          "'{}' ({}) disagrees with the world-rooted tree walk over the same "
          "joints, so which side of a pair is distal to this joint is "
          "ambiguous. This model's topology is not supported.",
          rec.name, rec.type_name));
    }
    positioned_order_.push_back(k);
  }
  std::sort(positioned_order_.begin(), positioned_order_.end(),
            [this](int a, int b) {
              return joints_[a].position_start < joints_[b].position_start;
            });

  // Position coordinate -> owning joint ordinal (every coordinate is owned).
  coord_joint_.assign(num_positions_, -1);
  for (int k : positioned_order_) {
    const JointRecord& rec = joints_[k];
    for (int c = rec.position_start; c < rec.position_start + rec.num_positions;
         ++c) {
      DRAKE_DEMAND(c >= 0 && c < num_positions_);
      DRAKE_DEMAND(coord_joint_[c] == -1);
      coord_joint_[c] = k;
    }
  }
  for (int c = 0; c < num_positions_; ++c) {
    DRAKE_DEMAND(coord_joint_[c] >= 0);
  }
}

void KinematicsEngine::BuildGeometry() {
  const MultibodyPlant<double>& plant = *plant_;
  const auto& inspector = model_->scene_graph().model_inspector();

  body_spheres_.assign(num_bodies_, {});
  body_sphere_geoms_.assign(num_bodies_, {});
  body_radius_.assign(num_bodies_, 0.0);
  body_has_halfspace_.assign(num_bodies_, false);
  body_halfspace_name_.assign(num_bodies_, std::string{});

  for (int b = 0; b < num_bodies_; ++b) {
    const BodyIndex body(b);
    DRAKE_DEMAND(plant.get_body(body).index() == body);
    const std::optional<drake::geometry::FrameId> frame_id =
        plant.GetBodyFrameIdIfExists(body);
    if (!frame_id.has_value()) continue;
    for (GeometryId gid :
         inspector.GetGeometries(*frame_id, Role::kProximity)) {
      const Shape& shape = inspector.GetShape(gid);
      if (IsHalfSpace(shape)) {
        // Half spaces are unbounded: they get no bounding sphere, and
        // CheckHalfSpaceRule() keeps them off the distal side of any
        // rotational coordinate.
        body_has_halfspace_[b] = true;
        if (body_halfspace_name_[b].empty()) {
          body_halfspace_name_[b] = inspector.GetName(gid);
        }
        continue;
      }
      BoundingSphere sphere;
      try {
        sphere = ComputeBoundingSphere(shape, inspector.GetPoseInFrame(gid));
      } catch (const std::exception& e) {
        throw std::runtime_error(fmt::format(
            "KinematicsEngine: proximity geometry '{}' on body '{}' cannot be "
            "bounded. {}",
            inspector.GetName(gid), plant.get_body(body).name(), e.what()));
      }
      // Origin-centred radius for the reach chain: ‖c_L‖ + ρ bounds every
      // point of the geometry's distance from the body frame origin, by the
      // triangle inequality on the sphere that contains it.
      body_radius_[b] =
          std::max(body_radius_[b], sphere.center_L.norm() + sphere.radius);
      body_sphere_geoms_[b].push_back(gid);
      body_spheres_[b].push_back(sphere);
      geometry_spheres_.emplace(gid, sphere);
    }
  }
}

void KinematicsEngine::CheckHalfSpaceRule() const {
  const MultibodyPlant<double>& plant = *plant_;
  const auto& inspector = model_->scene_graph().model_inspector();

  for (const auto& [ga, gb] : inspector.GetCollisionCandidates()) {
    const bool a_is_half = IsHalfSpace(inspector.GetShape(ga));
    const bool b_is_half = IsHalfSpace(inspector.GetShape(gb));
    if (!a_is_half && !b_is_half) continue;
    const drake::multibody::RigidBody<double>* body_a =
        plant.GetBodyFromFrameId(inspector.GetFrameId(ga));
    const drake::multibody::RigidBody<double>* body_b =
        plant.GetBodyFromFrameId(inspector.GetFrameId(gb));
    DRAKE_THROW_UNLESS(body_a != nullptr && body_b != nullptr);
    const BodyIndex ia = body_a->index();
    const BodyIndex ib = body_b->index();

    for (int k : positioned_order_) {
      const JointRecord& rec = joints_[k];
      const bool in_a = rec.subtree[ia];
      const bool in_b = rec.subtree[ib];
      if (in_a == in_b) continue;
      // The distal side is the one inside S_j; only *it* needs a finite reach.
      // A half space that is merely the static partner of a rotating body is
      // fine: λ then bounds the partner's points, and signed distance is
      // symmetric, so the certificate still holds.
      const bool distal_is_halfspace = in_a ? a_is_half : b_is_half;
      if (!distal_is_halfspace) continue;
      const bool rotational = rec.kind == JointKind::kRevolute ||
                              rec.kind == JointKind::kScrew ||
                              rec.kind == JointKind::kPlanar;
      if (!rotational) continue;
      const GeometryId offender = in_a ? ga : gb;
      const GeometryId partner = in_a ? gb : ga;
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: HalfSpace geometry '{}' (body '{}') rotates "
          "relative to its unfiltered partner geometry '{}' (body '{}') "
          "through joint '{}' ({}). A half space has unbounded reach, so no "
          "finite motion bound λ exists for that pair. Fix the model by "
          "anchoring the half space, filtering the pair, or replacing the "
          "half space with a large Box.",
          inspector.GetName(offender), plant.get_body(in_a ? ia : ib).name(),
          inspector.GetName(partner), plant.get_body(in_a ? ib : ia).name(),
          rec.name, rec.type_name));
    }
  }
}

std::vector<int> KinematicsEngine::CoordinatesAffectingPair(
    BodyIndex body_a, BodyIndex body_b) const {
  DRAKE_THROW_UNLESS(body_a.is_valid() && body_a < num_bodies_);
  DRAKE_THROW_UNLESS(body_b.is_valid() && body_b < num_bodies_);
  std::vector<int> out;
  for (int k : positioned_order_) {
    const JointRecord& rec = joints_[k];
    // Joint j ∈ J(p) iff exactly one of the pair's bodies is outboard of it:
    // only then does moving j change the pair's relative pose.
    if (rec.subtree[body_a] == rec.subtree[body_b]) continue;
    for (int c = rec.position_start; c < rec.position_start + rec.num_positions;
         ++c) {
      out.push_back(c);
    }
  }
  return out;
}

double KinematicsEngine::Reach(int joint_ord, BodyIndex body,
                               const std::vector<double>& box_hop) const {
  // r(j, B): distance from joint j's outboard (M) frame origin to any point of
  // B's proximity geometry, bounded uniformly over the control box.
  //
  // The walk accumulates translation norms only. Every hop composes rigid
  // transforms, and a rotation preserves norms, so by the triangle inequality
  //   ‖X_PF · X_FM · X_MC · p_C‖ ≤ ‖p_PF‖ + ‖t_FM‖ + ‖p_MC‖ + ‖p_C‖,
  // with ‖p_MC‖ = ‖p_CM‖. Only the middle term depends on the configuration,
  // and box_hop[] holds a uniform bound on it over the control box.
  double r = body_radius_[body];
  BodyIndex b = body;
  for (int guard = 0; guard <= num_bodies_; ++guard) {
    const int k = inboard_joint_[b];
    DRAKE_DEMAND(k >= 0);
    if (k == joint_ord) {
      // Top of the chain: measure from j's M-frame origin, the point that
      // stays fixed when coordinate j moves (for a revolute, the axis passes
      // through it). j's own X_FM and parent-side offset are excluded.
      return r + joints_[k].p_CM_norm;
    }
    r += joints_[k].fixed_hop + box_hop[k];
    b = joints_[k].inboard;
  }
  throw std::runtime_error(
      "KinematicsEngine: internal error: the reach chain walk did not reach "
      "the requested joint. This indicates inconsistent topology tables.");
}

MotionBoundTable KinematicsEngine::ComputeMotionBoundTable(
    const PiecewiseBezierPath& path, const std::vector<PairId>& pairs) const {
  if (path.num_positions() != num_positions_) {
    throw std::runtime_error(fmt::format(
        "KinematicsEngine: the path has {} positions but the plant has {}.",
        path.num_positions(), num_positions_));
  }
  return ComputeMotionBoundTable(path.global_lower_bound(),
                                 path.global_upper_bound(),
                                 path.constant_coordinates(), pairs);
}

MotionBoundTable KinematicsEngine::ComputeMotionBoundTable(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const std::vector<bool>& constant_coordinates,
    const std::vector<PairId>& pairs) const {
  if (lower.size() != num_positions_ || upper.size() != num_positions_ ||
      static_cast<int>(constant_coordinates.size()) != num_positions_) {
    throw std::runtime_error(fmt::format(
        "KinematicsEngine: control-box size mismatch: got lower={}, upper={}, "
        "constant_coordinates={} for a plant with {} positions.",
        lower.size(), upper.size(), constant_coordinates.size(),
        num_positions_));
  }
  for (int c = 0; c < num_positions_; ++c) {
    if (!std::isfinite(lower[c]) || !std::isfinite(upper[c]) ||
        lower[c] > upper[c]) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: the trajectory's global control box is invalid at "
          "coordinate {}: [{}, {}].",
          c, lower[c], upper[c]));
    }
  }

  const auto abs_max = [&lower, &upper](int c) {
    return std::max(std::abs(lower[c]), std::abs(upper[c]));
  };
  // The carve-out flags a coordinate constant when its whole control-point
  // range collapses to within Options::continuity_tolerance. That is a
  // tolerance, not an identity, and `range` is what the residual is charged
  // against.
  const auto range = [&lower, &upper](int c) {
    return upper[c] - lower[c];
  };

  // ------------------------------------------------------------------
  // Per-joint, box-dependent bound on ‖translation(X_FM)‖. This is the only
  // part of a chain hop that varies with the configuration; taking the max
  // over the trajectory's *control box* (not the plant's joint limits) keeps
  // unbounded prismatic joints usable and makes every reach trajectory
  // adaptive.
  // ------------------------------------------------------------------
  std::vector<double> box_hop(joints_.size(), 0.0);
  for (int k = 0; k < static_cast<int>(joints_.size()); ++k) {
    const JointRecord& rec = joints_[k];
    const int ps = rec.position_start;
    switch (rec.kind) {
      case JointKind::kWeld:
        // Fixed X_FM; already folded into fixed_hop at construction.
        break;
      case JointKind::kRevolute:
        // X_FM is a pure rotation about a point: zero translation.
        break;
      case JointKind::kPrismatic:
        box_hop[k] = abs_max(ps);
        break;
      case JointKind::kPlanar:
        // p_FoMo_F = (x, y, 0); ‖(x, y)‖ ≤ ‖(max|x|, max|y|)‖ over the box.
        box_hop[k] = std::hypot(abs_max(ps), abs_max(ps + 1));
        break;
      case JointKind::kScrew:
        // Drake's screw pitch is meters of travel per full revolution, so the
        // helix advances |θ|·|pitch| / 2π meters.
        box_hop[k] = abs_max(ps) * std::abs(rec.screw_pitch) / kTwoPi;
        break;
      case JointKind::kUnsupported: {
        for (int c = ps; c < ps + rec.num_positions; ++c) {
          if (!constant_coordinates[c]) {
            // TODO(wernerpe): Support quaternion coordinates via a
            // manifold-curve bound.
            throw std::runtime_error(fmt::format(
                "KinematicsEngine: this trajectory moves coordinate {} of "
                "joint '{}', whose type '{}' is not supported. Quaternion "
                "coordinates are not a vector space, so Bézier interpolation "
                "of their components has no rotation-space meaning and the "
                "convex-hull motion bound does not apply. Supported joint "
                "types are revolute, prismatic, planar, screw and weld; a "
                "floating base whose pose is *constant* along the trajectory "
                "is accepted via the constant-coordinate carve-out.",
                c, rec.name, rec.type_name));
          }
        }
        if (!rec.translation_offsets_known) {
          throw std::runtime_error(fmt::format(
              "KinematicsEngine: joint '{}' has type '{}', which this library "
              "does not know how to bound even when held constant. Supported "
              "joint types are revolute, prismatic, planar, screw and weld.",
              rec.name, rec.type_name));
        }
        double sum_sq = 0.0;
        for (int off : rec.translation_offsets) {
          const double m = abs_max(ps + off);
          sum_sq += m * m;
        }
        box_hop[k] = std::sqrt(sum_sq);
        break;
      }
    }
    DRAKE_DEMAND(std::isfinite(box_hop[k]) && box_hop[k] >= 0.0);
  }

  // ------------------------------------------------------------------
  // m_k: the minimum Euclidean norm of the quaternion 4-vector over the
  // control box, for the quaternion-floating joints. ‖q‖² is separable over
  // the coordinates, so the minimum is attained coordinate-wise at whichever
  // of {lower, upper, 0} lies in the interval and is closest to zero. It is
  // the only box-dependent quantity the quaternion λ̃ needs (see below); it is
  // left at 0 for every other joint, where it is never read.
  // ------------------------------------------------------------------
  std::vector<double> quat_min_norm(joints_.size(), 0.0);
  for (int k = 0; k < static_cast<int>(joints_.size()); ++k) {
    const JointRecord& rec = joints_[k];
    double sum_sq = 0.0;
    bool any_quaternion = false;
    for (int off = 0; off < static_cast<int>(rec.coord_rules.size()); ++off) {
      if (rec.coord_rules[off] != CoordRule::kQuaternion) continue;
      any_quaternion = true;
      const int c = rec.position_start + off;
      const double closest =
          (lower[c] <= 0.0 && upper[c] >= 0.0)
              ? 0.0
              : std::min(std::abs(lower[c]), std::abs(upper[c]));
      sum_sq += closest * closest;
    }
    if (any_quaternion) quat_min_norm[k] = std::sqrt(sum_sq);
  }

  // ------------------------------------------------------------------
  // Assemble the CSR table.
  //
  // Displacement lemma (PWL ancestor: Schwarzer, Saha & Latombe,
  // "Adaptive Dynamic Collision Checking for Single and Multiple Articulated
  // Robots in Complex Environments", IEEE T-RO 21(3), 2005):
  //
  //   For any q, q′ in the control box and any pair p = (A, B), the signed
  //   distance between the two geometries changes by at most
  //   Σ_{j ∈ J(p)} λ(j,p)·|q′_j − q_j|.
  //
  // Proof sketch. Walk from q to q′ one coordinate at a time along the
  // axis-aligned path; every intermediate configuration stays inside the box
  // (a box is closed under coordinate-wise interpolation), so every reach r,
  // computed as a uniform bound over that box, is valid at each step. On the
  // step that moves coordinate j alone, only the distal side D(j,p) (the body
  // of the pair inside S_j) moves relative to the other body, and the relative
  // transform factors as
  //     X_{O,D}(q) = X_{O,P}·X_{P,F}·X_FM(q_j)·X_{M,C}·X_{C,D},
  // in which every factor but X_FM(q_j) is constant. A material point of D is
  // therefore displaced, in O's frame, by exactly
  //     ‖(X_FM(q′_j) − X_FM(q_j))·u‖ with ‖u‖ ≤ r(j, D),
  // because the leading factors are isometries and u is the point measured
  // from Mo. Bounding that per joint type gives the λ values below:
  //   revolute  chord ≤ arc  => λ = r;
  //   prismatic pure unit translation => λ = 1;
  //   planar    λ = 1 for x and y, λ = r for θ;
  //   screw     rotation + |pitch|/2π of axial travel => λ = r + |pitch|/2π.
  // Since a rigid motion of one of two sets changes their separation distance
  // by at most the supremum pointwise displacement (triangle inequality on the
  // minimizing witness pair), each step changes the distance by at most
  // λ(j,p)·|Δq_j|, and the telescoping sum over steps gives the lemma. Note
  // that the *distal side varies per joint* on a self-collision pair; the sum
  // is still valid because each step is bounded in the frame of that step's
  // static side and distance is frame-invariant.
  //
  // Only the separated branch of the distance function is ever used, so no
  // penetration-depth regularity is needed.
  //
  // ------------------------------------------------------------------
  // The carve-out residual (carveout_slack_p).
  //
  // The constant-coordinate carve-out drops coordinate j from J(p) when its
  // *whole* control-point range fits inside Options::continuity_tolerance.
  // That is a tolerance, not an identity: the curve may still move q_j
  // anywhere inside [lower_j, upper_j], and the telescoping proof above
  // therefore still owes one step for j. Dropping the step outright would
  // understate Δ_p by up to λ̃_j·range_j, which is unaccounted for anywhere
  // else and is orders of magnitude above Options::certificate_slack, so the
  // certificate inequality could pass with the true clearance below the
  // threshold by that much. We charge the step at its worst case instead,
  // once per pair, against the *global* range (the node's own excursion in a
  // carved coordinate is contained in it):
  //
  //   carveout_slack_p = Σ_{j ∈ J_topo(p), j carved} λ̃_j · range_j,
  //   range_j = upper_j − lower_j.
  //
  // J_topo(p) is the pre-carve-out coordinate set, so the sum runs over
  // exactly the steps the CSR row no longer carries. MotionBound() adds it
  // unconditionally, which restores the telescoping sum in full. It is
  // bit-exactly zero whenever every carved coordinate is exactly constant,
  // which is the case for every path whose control points repeat the
  // coordinate's value verbatim. λ̃_j, per coordinate kind:
  //
  //  * revolute / prismatic / planar / screw: the λ formulas above,
  //    unchanged. The step being bounded is the same step; the carve-out
  //    changed nothing about the geometry, only about what the table stores.
  //
  //  * RpyFloating, BallRpy and Universal *rotation* coordinates: λ̃ = r.
  //    Each such angle enters X_FM as one factor of a product of elementary
  //    rotations about axes through Mo (Rz(y)·Ry(p)·Rx(r) for rpy, likewise
  //    for a universal joint's two angles), so changing angle j alone takes
  //    R to R′ with R′R⁻¹ conjugate to a rotation by |Δq_j|, i.e. a rotation
  //    by exactly |Δq_j| about *some* axis through Mo. A material point u of
  //    the distal side, measured from Mo, is then displaced by
  //    ‖(R′ − R)u‖ = ‖(R′R⁻¹ − I)(Ru)‖ ≤ |Δq_j|·‖u‖ ≤ r·|Δq_j|, which is the
  //    revolute bound with the same r from the same chain walk (the walk
  //    bounds the distance from Mo to the distal geometry and does not care
  //    what kind of joint sits at the top of the chain). X_FM's translation
  //    is untouched by these coordinates: zero for BallRpy/Universal, and
  //    p_FM for RpyFloating, which is carried by its own coordinates.
  //
  //  * RpyFloating / QuaternionFloating *translation* coordinates: λ̃ = 1.
  //    They are p_FM's components; a unit change translates the whole distal
  //    side by one unit.
  //
  //  * QuaternionFloating quaternion coefficients: λ̃ = 2r/m ≤ 4r, with
  //    m = min over the control box of ‖q‖ (computed above). Derivation.
  //    Drake normalizes internally, X_FM using R(q/‖q‖), so the map from
  //    coefficients to rotation is q ↦ R(π(q)) with π(q) = q/‖q‖. π has
  //    derivative Dπ(q) = (I − q̂q̂ᵀ)/‖q‖, an orthogonal projector scaled by
  //    1/‖q‖, hence ‖Dπ(q)‖₂ = 1/‖q‖. The control box is convex and every
  //    point of it has ‖q‖ ≥ m, so for u, v in the box the straight segment
  //    between them stays in the box and the geodesic distance on S³ between
  //    π(u) and π(v) is at most the length of its image,
  //      ψ ≤ ∫₀¹ ‖Dπ(γ(t))·γ′(t)‖ dt ≤ ‖u − v‖ / m.
  //    The rotation-angle metric on SO(3) is at most twice the geodesic
  //    metric on S³ (the unit quaternions double-cover SO(3): a geodesic of
  //    length ψ maps to a rotation of angle 2ψ), so the rotation angle
  //    between R(π(u)) and R(π(v)) obeys θ ≤ 2‖u − v‖/m. A material point at
  //    distance ≤ r from Mo is displaced by at most the chord
  //    2r·sin(θ/2) ≤ r·θ ≤ (2r/m)·‖u − v‖, and since
  //    ‖u − v‖₂ ≤ ‖u − v‖₁ ≤ Σ_j range_j over the four coefficients, charging
  //    λ̃ = 2r/m per coefficient covers every pair (u, v) in the box.
  //    In the regime the carve-out produces, a box of diameter
  //    ρ ≤ continuity_tolerance around a unit quaternion, m ≥ 1 − ρ, so
  //    2r/m ≤ 2r/(1 − ρ) ≤ 4r for any ρ ≤ 1/2: the coefficient is at worst
  //    the small-angle constant 2r with a factor-2 margin, and is computed
  //    rather than assumed. m = 0, a box containing the zero quaternion,
  //    admits no bound at all, because Drake's own normalization is undefined
  //    there, and throws.
  //
  //  * Any rotational carved coordinate whose distal side carries a HalfSpace
  //    has no finite r and therefore no finite λ̃; its residual is genuinely
  //    unbounded. Such a coordinate must be *exactly* constant; anything else
  //    throws.
  // ------------------------------------------------------------------
  std::vector<int> row_start;
  std::vector<int> coord;
  std::vector<double> lambda;
  std::vector<double> carveout_slack;
  row_start.reserve(pairs.size() + 1);
  row_start.push_back(0);
  carveout_slack.reserve(pairs.size());

  // r(j, D) is shared by every pair with the same (joint, distal body), which
  // is the common case for an environment-heavy scene.
  std::unordered_map<std::int64_t, double> reach_cache;
  const auto reach_of = [&](int k, BodyIndex distal) {
    const std::int64_t key =
        static_cast<std::int64_t>(k) * num_bodies_ + static_cast<int>(distal);
    auto it = reach_cache.find(key);
    if (it != reach_cache.end()) return it->second;
    const double r = Reach(k, distal, box_hop);
    reach_cache.emplace(key, r);
    return r;
  };

  for (const PairId& pair : pairs) {
    const BodyIndex a = pair.body_a;
    const BodyIndex b = pair.body_b;
    if (!a.is_valid() || !b.is_valid() || a >= num_bodies_ ||
        b >= num_bodies_) {
      throw std::runtime_error(fmt::format(
          "KinematicsEngine: pair references body indices ({}, {}) outside the "
          "plant's {} bodies.",
          static_cast<int>(a), static_cast<int>(b), num_bodies_));
    }
    double slack = 0.0;
    for (int k : positioned_order_) {
      const JointRecord& rec = joints_[k];
      const bool in_a = rec.subtree[a];
      const bool in_b = rec.subtree[b];
      if (in_a == in_b) continue;  // j ∉ J_topo(p).
      const BodyIndex distal = in_a ? a : b;
      const int ps = rec.position_start;

      double r = -1.0;  // Computed lazily: only rotational λ needs it.
      const auto reach = [&]() {
        if (r < 0.0) {
          if (body_has_halfspace_[distal]) {
            throw std::runtime_error(fmt::format(
                "KinematicsEngine: HalfSpace geometry '{}' on body '{}' is the "
                "distal side of joint '{}' ({}), which rotates it. A half "
                "space has unbounded reach, so no finite λ exists.",
                body_halfspace_name_[distal], plant_->get_body(distal).name(),
                rec.name, rec.type_name));
          }
          r = reach_of(k, distal);
        }
        return r;
      };

      for (int c = ps; c < ps + rec.num_positions; ++c) {
        if (constant_coordinates[c]) {
          // Joint-support carve-out: c leaves J(p), and its residual
          // motion inside the control box is charged to the pair's slack
          // instead. See the derivation above for every λ̃ used here.
          const double span = range(c);
          DRAKE_DEMAND(std::isfinite(span) && span >= 0.0);
          if (span == 0.0) continue;  // Exactly constant: nothing to charge.
          if (rec.coord_rules.empty()) {
            // Unreachable: a joint kind with no rules is rejected above,
            // constant or not. Kept as a guard so a future joint kind cannot
            // reach here uncharged.
            throw std::runtime_error(fmt::format(
                "KinematicsEngine: joint '{}' has type '{}', which this "
                "library does not know how to bound even when held constant.",
                rec.name, rec.type_name));
          }
          const CoordRule rule = rec.coord_rules[c - ps];
          if (IsRotationalRule(rule) && body_has_halfspace_[distal]) {
            throw std::runtime_error(fmt::format(
                "KinematicsEngine: HalfSpace geometry '{}' on body '{}' is the "
                "distal side of coordinate {} of joint '{}' ({}), which "
                "rotates it, and this trajectory holds that coordinate "
                "constant only to within a tolerance: its control-point "
                "range is {}, not 0. A half space has unbounded reach, so the "
                "residual motion of a rotational coordinate across it cannot "
                "be bounded by any finite λ. A half space may only sit across "
                "a rotational coordinate that is EXACTLY constant. Fix the "
                "trajectory so that coordinate's control points are "
                "identical, anchor the half space, filter the pair, or "
                "replace the half space with a large Box.",
                body_halfspace_name_[distal], plant_->get_body(distal).name(),
                c, rec.name, rec.type_name, span));
          }
          double lam_tilde = 0.0;
          switch (rule) {
            case CoordRule::kTranslation:
              lam_tilde = 1.0;
              break;
            case CoordRule::kRotation:
              lam_tilde = reach();
              break;
            case CoordRule::kScrewCoord:
              lam_tilde = reach() + std::abs(rec.screw_pitch) / kTwoPi;
              break;
            case CoordRule::kQuaternion: {
              const double m = quat_min_norm[k];
              if (!(m > 0.0)) {
                throw std::runtime_error(fmt::format(
                    "KinematicsEngine: the trajectory's control box for the "
                    "quaternion coordinates of joint '{}' ({}) contains the "
                    "zero quaternion, whose normalized rotation is undefined, "
                    "so the residual motion of its carved-out coordinates "
                    "cannot be bounded. Quaternion control points must be "
                    "unit quaternions.",
                    rec.name, rec.type_name));
              }
              lam_tilde = 2.0 * reach() / m;
              break;
            }
          }
          DRAKE_DEMAND(std::isfinite(lam_tilde) && lam_tilde >= 0.0);
          slack += lam_tilde * span;
          continue;
        }
        double lam = 0.0;
        switch (rec.kind) {
          case JointKind::kRevolute:
            lam = reach();
            break;
          case JointKind::kPrismatic:
            lam = 1.0;
            break;
          case JointKind::kPlanar:
            // q = (x, y, θ); see PlanarJoint's class documentation.
            lam = (c == ps + 2) ? reach() : 1.0;
            break;
          case JointKind::kScrew:
            lam = reach() + std::abs(rec.screw_pitch) / kTwoPi;
            break;
          case JointKind::kWeld:
          case JointKind::kUnsupported:
            throw std::runtime_error(fmt::format(
                "KinematicsEngine: internal error: joint '{}' ({}) reached "
                "the λ assembly with an unsupported kind.",
                rec.name, rec.type_name));
        }
        DRAKE_DEMAND(std::isfinite(lam) && lam >= 0.0);
        coord.push_back(c);
        lambda.push_back(lam);
      }
    }
    DRAKE_DEMAND(std::isfinite(slack) && slack >= 0.0);
    carveout_slack.push_back(slack);
    row_start.push_back(static_cast<int>(coord.size()));
  }
  return MotionBoundTable(std::move(row_start), std::move(coord),
                          std::move(lambda), std::move(carveout_slack));
}

const std::vector<BoundingSphere>& KinematicsEngine::body_spheres(
    BodyIndex body) const {
  DRAKE_THROW_UNLESS(body.is_valid() && body < num_bodies_);
  return body_spheres_[body];
}

const std::vector<GeometryId>& KinematicsEngine::body_sphere_geometries(
    BodyIndex body) const {
  DRAKE_THROW_UNLESS(body.is_valid() && body < num_bodies_);
  return body_sphere_geoms_[body];
}

const BoundingSphere& KinematicsEngine::geometry_sphere(GeometryId id) const {
  auto it = geometry_spheres_.find(id);
  if (it == geometry_spheres_.end()) {
    throw std::runtime_error(fmt::format(
        "KinematicsEngine: geometry {} has no bounding sphere; it is either "
        "not a proximity geometry of this model or it is a HalfSpace.",
        id));
  }
  return it->second;
}

bool KinematicsEngine::body_has_halfspace(BodyIndex body) const {
  DRAKE_THROW_UNLESS(body.is_valid() && body < num_bodies_);
  return body_has_halfspace_[body];
}

double KinematicsEngine::body_radius(BodyIndex body) const {
  DRAKE_THROW_UNLESS(body.is_valid() && body < num_bodies_);
  return body_radius_[body];
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
