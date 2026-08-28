#include "drake/planning/continuous_collision/distance_oracle.h"

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/continuous_collision/shape_class.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraphInspector;
using drake::math::RigidTransformd;
using internal::Classify;
using internal::ShapeClass;

/* Everything the analytic halfspace fallback needs about the *non*-halfspace
partner, extracted once by the probe. Only the fields relevant to `klass` are
populated. All quantities are in the geometry's canonical frame G. */
struct SupportData {
  ShapeClass klass{ShapeClass::kUnsupported};
  /* Sphere / Capsule / Cylinder radius. */
  double radius{0.0};
  /* Half the axial length of a Capsule / Cylinder. */
  double half_length{0.0};
  /* Box half-sizes, or Ellipsoid semi-axes (a, b, c). */
  Eigen::Vector3d extent{Eigen::Vector3d::Zero()};
  /* Convex / Mesh: the vertices of the very hull object the proximity engine
  collides (`GetConvexHull()`), so scale and any degeneracy inflation Drake
  applied are already baked in. */
  Eigen::Matrix3Xd hull_G;
};

Eigen::Matrix3Xd HullVertices(
    const drake::geometry::PolygonSurfaceMesh<double>& hull) {
  Eigen::Matrix3Xd v(3, hull.num_vertices());
  for (int i = 0; i < hull.num_vertices(); ++i) {
    v.col(i) = hull.vertex(i);
  }
  return v;
}

SupportData MakeSupportData(const drake::geometry::Shape& shape) {
  SupportData data;
  data.klass = Classify(shape);
  switch (data.klass) {
    case ShapeClass::kSphere:
      data.radius = static_cast<const drake::geometry::Sphere&>(shape).radius();
      break;
    case ShapeClass::kBox:
      data.extent =
          static_cast<const drake::geometry::Box&>(shape).size() / 2.0;
      break;
    case ShapeClass::kCapsule: {
      const auto& s = static_cast<const drake::geometry::Capsule&>(shape);
      data.radius = s.radius();
      data.half_length = s.length() / 2.0;
      break;
    }
    case ShapeClass::kCylinder: {
      const auto& s = static_cast<const drake::geometry::Cylinder&>(shape);
      data.radius = s.radius();
      data.half_length = s.length() / 2.0;
      break;
    }
    case ShapeClass::kEllipsoid: {
      const auto& s = static_cast<const drake::geometry::Ellipsoid&>(shape);
      data.extent = Eigen::Vector3d(s.a(), s.b(), s.c());
      break;
    }
    case ShapeClass::kConvex:
      data.hull_G = HullVertices(
          static_cast<const drake::geometry::Convex&>(shape).GetConvexHull());
      break;
    case ShapeClass::kMesh:
      data.hull_G = HullVertices(
          static_cast<const drake::geometry::Mesh&>(shape).GetConvexHull());
      break;
    case ShapeClass::kHalfSpace:
    case ShapeClass::kUnsupported:
      break;
  }
  return data;
}

/* Returns argmax over x ∈ C of d_W·x, with C the shape described by `data`
posed at `X_WC` and `d_W` a unit vector -- i.e. the point attaining the
support function h_C(d_W). Each branch is the standard closed form.

Below R = X_WC.rotation(), c = X_WC.translation(), d_C = Rᵀ·d_W, and
â = R·ẑ is the shape's canonical axis expressed in world. */
Eigen::Vector3d SupportPoint(const SupportData& data,
                             const RigidTransformd& X_WC,
                             const Eigen::Vector3d& d_W) {
  const Eigen::Matrix3d& R = X_WC.rotation().matrix();
  const Eigen::Vector3d& c = X_WC.translation();
  switch (data.klass) {
    case ShapeClass::kSphere:
      return c + data.radius * d_W;
    case ShapeClass::kBox: {
      // The box is a product of intervals in frame C, so each coordinate
      // maximizes independently at the half-size with the sign of d_C.
      const Eigen::Vector3d d_C = R.transpose() * d_W;
      Eigen::Vector3d corner_C;
      for (int i = 0; i < 3; ++i) {
        corner_C(i) = (d_C(i) >= 0.0 ? data.extent(i) : -data.extent(i));
      }
      return c + R * corner_C;
    }
    case ShapeClass::kCapsule: {
      // Minkowski sum of the axis segment and a ball: support functions add.
      const Eigen::Vector3d axis = R.col(2);
      const double s = (d_W.dot(axis) >= 0.0 ? 1.0 : -1.0);
      return c + (s * data.half_length) * axis + data.radius * d_W;
    }
    case ShapeClass::kCylinder: {
      // Product of the axis segment and a disk in the orthogonal plane, so
      // the axial and radial maximizations are independent.
      const Eigen::Vector3d axis = R.col(2);
      const double s = (d_W.dot(axis) >= 0.0 ? 1.0 : -1.0);
      Eigen::Vector3d p = c + (s * data.half_length) * axis;
      const Eigen::Vector3d d_perp = d_W - d_W.dot(axis) * axis;
      const double norm = d_perp.norm();
      // Near-axis-parallel direction: every rim point ties, so keep the cap
      // center. It is still a supporting point and still on the surface (the
      // caps are flat disks).
      if (norm > 1e-14) {
        p += (data.radius / norm) * d_perp;
      }
      return p;
    }
    case ShapeClass::kEllipsoid: {
      // E = {c + M·u : ‖u‖ ≤ 1} with M = R·diag(a,b,c). Cauchy-Schwarz gives
      //   max_{‖u‖≤1} d·(c + M·u) = d·c + ‖Mᵀd‖,  attained at u* = Mᵀd/‖Mᵀd‖,
      // so x* = c + M·Mᵀd/‖Mᵀd‖. M is invertible (radii > 0) and ‖d‖ = 1, so
      // ‖Mᵀd‖ ≥ min(a,b,c) > 0.
      const Eigen::Matrix3d M = R * data.extent.asDiagonal();
      const Eigen::Vector3d Mt_d = M.transpose() * d_W;
      const double norm = Mt_d.norm();
      if (norm <= 0.0) return c;
      return c + (M * Mt_d) / norm;
    }
    case ShapeClass::kConvex:
    case ShapeClass::kMesh: {
      // The support of a polytope is attained at a vertex; maximize in the
      // geometry frame so the rotation is applied only once, at the end.
      const Eigen::Vector3d d_C = R.transpose() * d_W;
      Eigen::Index best = 0;
      (d_C.transpose() * data.hull_G).maxCoeff(&best);
      return c + R * data.hull_G.col(best);
    }
    case ShapeClass::kHalfSpace:
    case ShapeClass::kUnsupported:
      break;
  }
  throw std::logic_error(
      "DistanceOracle: internal error - no support function for this shape "
      "class; the capability probe should have refused it.");
}

std::string ClassName(ShapeClass klass) {
  switch (klass) {
    case ShapeClass::kSphere:
      return "Sphere";
    case ShapeClass::kBox:
      return "Box";
    case ShapeClass::kCapsule:
      return "Capsule";
    case ShapeClass::kCylinder:
      return "Cylinder";
    case ShapeClass::kEllipsoid:
      return "Ellipsoid";
    case ShapeClass::kConvex:
      return "Convex";
    case ShapeClass::kMesh:
      return "Mesh";
    case ShapeClass::kHalfSpace:
      return "HalfSpace";
    case ShapeClass::kUnsupported:
      break;
  }
  return "<unsupported>";
}

/* "geometry_name (ShapeType)", for error messages and the report. */
std::string Describe(const SceneGraphInspector<double>& inspector,
                     GeometryId id) {
  return fmt::format("{} ({})", inspector.GetName(id),
                     inspector.GetShape(id).type_name());
}

/* One row of the probe report: a distinct unordered shape-type combination
and the route it resolved to. */
struct ComboRow {
  DistanceRoute route{DistanceRoute::kNative};
  int pair_count{0};
  /* A representative pair, used for the probe query and error messages. */
  GeometryId example_a;
  GeometryId example_b;
};

}  // namespace

struct DistanceOracle::Impl {
  /* Closed-form support data for every geometry that partners a halfspace.
  Keyed by geometry id because the facade hands back its own PairRecord
  copies, so SignedDistance() cannot index into pairs_. */
  std::unordered_map<GeometryId, SupportData> support;
  std::string report;
};

DistanceOracle::DistanceOracle(const RobotDiagram<double>& model,
                               double query_tolerance) {
  DRAKE_THROW_UNLESS(query_tolerance >= 0.0);
  tolerance_ = query_tolerance;
  auto impl = std::make_shared<Impl>();

  const drake::geometry::SceneGraph<double>& scene_graph = model.scene_graph();
  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();
  const drake::multibody::MultibodyPlant<double>& plant = model.plant();

  // --- Deformables are out of scope: refuse, naming them. -----------------
  const std::vector<GeometryId> deformables =
      inspector.GetAllDeformableGeometryIds();
  if (!deformables.empty()) {
    std::string names;
    for (const GeometryId id : deformables) {
      names += fmt::format("\n  - {}", inspector.GetName(id));
    }
    throw std::runtime_error(fmt::format(
        "DistanceOracle: deformable geometries are not supported (certified "
        "continuous collision checking assumes rigid bodies whose motion the "
        "plant's kinematics describe). Offending geometries:{}",
        names));
  }

  // --- Snapshot the unfiltered pairs and classify each one. ----------------
  // GetCollisionCandidates() returns a sorted std::set and std::map keeps the
  // report ordering fixed, so both pairs_ and support_report() are
  // deterministic for a given model.
  std::map<std::pair<ShapeClass, ShapeClass>, ComboRow> combos;
  std::set<std::string> mesh_names;

  for (const auto& [id_a, id_b] : inspector.GetCollisionCandidates()) {
    const drake::multibody::RigidBody<double>* body_a =
        plant.GetBodyFromFrameId(inspector.GetFrameId(id_a));
    const drake::multibody::RigidBody<double>* body_b =
        plant.GetBodyFromFrameId(inspector.GetFrameId(id_b));
    if (body_a == nullptr || body_b == nullptr) {
      throw std::runtime_error(fmt::format(
          "DistanceOracle: collision geometry {} is not attached to a "
          "MultibodyPlant body; the checker can only certify geometry whose "
          "motion the plant describes.",
          Describe(inspector, body_a == nullptr ? id_a : id_b)));
    }

    const ShapeClass class_a = Classify(inspector.GetShape(id_a));
    const ShapeClass class_b = Classify(inspector.GetShape(id_b));

    if (class_a == ShapeClass::kHalfSpace &&
        class_b == ShapeClass::kHalfSpace) {
      throw std::runtime_error(fmt::format(
          "DistanceOracle: signed distance between two HalfSpace geometries "
          "is undefined, so the pair {} / {} cannot be certified. Remove one "
          "halfspace, or filter the pair (CollisionFilterManager / a "
          "collision filter group).",
          Describe(inspector, id_a), Describe(inspector, id_b)));
    }

    DistanceRoute route = DistanceRoute::kNative;
    if (class_a == ShapeClass::kHalfSpace) {
      route = DistanceRoute::kHalfSpaceA;
    } else if (class_b == ShapeClass::kHalfSpace) {
      route = DistanceRoute::kHalfSpaceB;
    }

    if (route != DistanceRoute::kNative) {
      // The analytic fallback needs a closed-form support function for the
      // partner; anything outside the supported set is refused here rather
      // than mid-certification.
      const bool a_is_halfspace = (route == DistanceRoute::kHalfSpaceA);
      const GeometryId partner = a_is_halfspace ? id_b : id_a;
      const ShapeClass partner_class = a_is_halfspace ? class_b : class_a;
      if (partner_class == ShapeClass::kUnsupported) {
        throw std::runtime_error(fmt::format(
            "DistanceOracle: no closed-form support function for shape type "
            "'{}', so the halfspace pair {} / {} cannot be certified.",
            inspector.GetShape(partner).type_name(), Describe(inspector, id_a),
            Describe(inspector, id_b)));
      }
      if (impl->support.find(partner) == impl->support.end()) {
        impl->support.emplace(partner,
                              MakeSupportData(inspector.GetShape(partner)));
      }
    }

    // Meshes are certified as their convex hulls; the report says so.
    if (class_a == ShapeClass::kMesh)
      mesh_names.insert(inspector.GetName(id_a));
    if (class_b == ShapeClass::kMesh)
      mesh_names.insert(inspector.GetName(id_b));

    const auto key = std::minmax(class_a, class_b);
    const std::pair<ShapeClass, ShapeClass> combo{key.first, key.second};
    auto it = combos.find(combo);
    if (it == combos.end()) {
      combos.emplace(combo, ComboRow{route, 1, id_a, id_b});
    } else {
      ++it->second.pair_count;
    }

    pairs_.push_back(PairRecord{
        PairId{id_a, id_b, body_a->index(), body_b->index()}, route, 0.0});
  }

  // --- One probe query per distinct native combination. --------------------
  // The whole point of the probe: an unsupported (type, type) combination is
  // discovered here, at construction, and never mid-certification.
  if (!combos.empty()) {
    std::unique_ptr<drake::systems::Context<double>> root_context =
        model.CreateDefaultContext();
    const drake::systems::Context<double>& sg_context =
        scene_graph.GetMyContextFromRoot(*root_context);
    const auto& query_object =
        scene_graph.get_query_output_port().Eval<QueryObject<double>>(
            sg_context);

    for (const auto& [combo, row] : combos) {
      if (row.route != DistanceRoute::kNative) continue;
      try {
        query_object.ComputeSignedDistancePairClosestPoints(row.example_a,
                                                            row.example_b);
      } catch (const std::exception& e) {
        throw std::runtime_error(fmt::format(
            "DistanceOracle: this Drake build cannot compute signed distance "
            "for the shape combination ({}, {}); an offending pair is {} / "
            "{}. Filter the pair, or replace the geometry with a supported "
            "shape (Convex is always supported). Drake reported: {}",
            ClassName(combo.first), ClassName(combo.second),
            Describe(inspector, row.example_a),
            Describe(inspector, row.example_b), e.what()));
      }
    }
  }

  // --- Render the report. --------------------------------------------------
  std::string report = fmt::format(
      "DistanceOracle capability probe: {} unfiltered pair(s), {} distinct "
      "shape-type combination(s), tolerance tau = {} m.\n",
      pairs_.size(), combos.size(), tolerance_);
  for (const auto& [combo, row] : combos) {
    const char* const route =
        (row.route == DistanceRoute::kNative)
            ? "native (ComputeSignedDistancePairClosestPoints, probed ok)"
            : "halfspace analytic support-function fallback (exact)";
    report += fmt::format("  {}-{}: {}; {} pair(s)\n", ClassName(combo.first),
                          ClassName(combo.second), route, row.pair_count);
  }
  for (const std::string& name : mesh_names) {
    report += fmt::format("  Mesh {}: certified as its convex hull\n", name);
  }
  impl->report = std::move(report);

  impl_ = std::move(impl);
}

double DistanceOracle::SignedDistance(const QueryObject<double>& query_object,
                                      const PairRecord& pair,
                                      Eigen::Vector3d* nearest_a_W,
                                      Eigen::Vector3d* nearest_b_W) const {
  if (pair.route == DistanceRoute::kNative) {
    const drake::geometry::SignedDistancePair<double> result =
        query_object.ComputeSignedDistancePairClosestPoints(pair.id.a,
                                                            pair.id.b);
    // Drake reports the pair in its own fixed but undocumented order, which
    // may be the reverse of this record's; the witness points come back in
    // *its* A/B geometry frames, so undo any swap explicitly.
    Eigen::Vector3d p_ACa;
    Eigen::Vector3d p_BCb;
    if (result.id_A == pair.id.a && result.id_B == pair.id.b) {
      p_ACa = result.p_ACa;
      p_BCb = result.p_BCb;
    } else if (result.id_A == pair.id.b && result.id_B == pair.id.a) {
      p_ACa = result.p_BCb;
      p_BCb = result.p_ACa;
    } else {
      throw std::runtime_error(
          "DistanceOracle: Drake returned a signed distance result for a "
          "different geometry pair than the one queried.");
    }
    if (nearest_a_W != nullptr) {
      *nearest_a_W = query_object.GetPoseInWorld(pair.id.a) * p_ACa;
    }
    if (nearest_b_W != nullptr) {
      *nearest_b_W = query_object.GetPoseInWorld(pair.id.b) * p_BCb;
    }
    return result.distance;
  }

  // --- Analytic halfspace fallback (exact). --------------------------------
  // Drake's HalfSpace is {x : n̂·(x - p0) ≤ 0}, with n̂ = R_WG·ẑ the outward
  // normal and p0 = X_WG.translation() a point of the boundary plane. For a
  // convex partner C,
  //     ϕ = min_{x ∈ C} n̂·(x - p0) = -h_C(-n̂) - n̂·p0.
  // Proof that this is the signed distance on both branches: translating C by
  // t·n̂ shifts the minimum by exactly t, and C is disjoint from the halfspace
  // iff that minimum is ≥ 0. Hence for ϕ ≥ 0 the pair is separated and the
  // minimizer together with its foot on the plane realizes the gap (any point
  // of C is at least ϕ from the plane, and the minimizer is exactly ϕ), while
  // for ϕ < 0 the smallest translation that separates them has length -ϕ,
  // which is Drake's negative-penetration-depth definition. Exact, so this
  // route contributes 0 to τ -- but τ accounting stays uniform (the numerical
  // policy).
  const bool a_is_halfspace = (pair.route == DistanceRoute::kHalfSpaceA);
  const GeometryId halfspace_id = a_is_halfspace ? pair.id.a : pair.id.b;
  const GeometryId partner_id = a_is_halfspace ? pair.id.b : pair.id.a;

  const auto it = impl_->support.find(partner_id);
  if (it == impl_->support.end()) {
    throw std::runtime_error(
        "DistanceOracle::SignedDistance(): the pair's halfspace route names a "
        "geometry the capability probe never classified. Pass PairRecords "
        "obtained from pairs() (thresholds may be rewritten; ids and routes "
        "may not).");
  }

  const RigidTransformd& X_WH = query_object.GetPoseInWorld(halfspace_id);
  const Eigen::Vector3d n_W = X_WH.rotation().matrix().col(2);
  const Eigen::Vector3d p0_W = X_WH.translation();
  const RigidTransformd& X_WC = query_object.GetPoseInWorld(partner_id);

  const Eigen::Vector3d x_W = SupportPoint(it->second, X_WC, -n_W);
  const double phi = n_W.dot(x_W - p0_W);
  // The halfspace witness is the minimizer's orthogonal projection onto the
  // boundary plane; the witness displacement is then exactly ϕ·n̂.
  const Eigen::Vector3d plane_W = x_W - phi * n_W;

  if (nearest_a_W != nullptr) {
    *nearest_a_W = a_is_halfspace ? plane_W : x_W;
  }
  if (nearest_b_W != nullptr) {
    *nearest_b_W = a_is_halfspace ? x_W : plane_W;
  }
  return phi;
}

const std::string& DistanceOracle::support_report() const {
  return impl_->report;
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
