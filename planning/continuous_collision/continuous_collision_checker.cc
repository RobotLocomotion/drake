#include "drake/planning/continuous_collision/continuous_collision_checker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/continuous_collision/certifier.h"
#include "drake/planning/continuous_collision/internal.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::GeometryId;
using drake::multibody::BodyIndex;
using internal::Classify;
using internal::DistanceRoute;
using internal::kNumShapeClasses;
using internal::PairRecord;
using internal::ShapeClass;

// ---------------------------------------------------------------------------
// Per-pair oracle tolerance τ_p.
// ---------------------------------------------------------------------------
//
// Drake documents ComputeSignedDistancePairClosestPoints() accuracy as bad as
// 5e-5 m for some shape pairs, well outside the 1e-6 m internal::
// kQueryTolerance, and an oracle that over-reports a distance at or above the
// threshold can fake a certificate. Every use of τ (node certificate test,
// definite violation test, breakpoints) therefore takes τ_p =
// max(kQueryTolerance, documented_accuracy(shape_a, shape_b)); pairs routed
// through the analytic halfspace fallback are closed-form and keep the raw
// kQueryTolerance.
//
// The table below is Table 4 of drake/geometry/query_object.h. Mesh is
// certified as its convex hull, so its row and column duplicate Convex's, and
// a shape the checker cannot classify is charged the worst documented value.
// Never relax an entry ahead of Drake's own documentation.
//
// clang-format off
//   |           | Box   | Caps  | Conv  | Cyl   | Ellip | Mesh  | Sph   |
//   | Box       | 4e-15 |       |       |       |       |       |       |
//   | Capsule   | 3e-6  | 2e-5  |       |       |       |       |       |
//   | Convex    | 3e-15 | 2e-5  | 3e-15 |       |       |       |       |
//   | Cylinder  | 6e-6  | 1e-5  | 6e-6  | 2e-5  |       |       |       |
//   | Ellipsoid | 9e-6  | 5e-6  | 9e-6  | 5e-5  | 2e-5  |       |       |
//   | Mesh      | (= the Convex row)                    | 3e-15 |       |
//   | Sphere    | 4e-15 | 6e-15 | 3e-6  | 5e-15 | 4e-5  | 3e-6  | 6e-15 |
// clang-format on

/* Worst documented error over the whole table, charged to any shape the
 checker cannot classify. Such a shape never reaches the narrowphase, because
 the capability probe refuses unknown shapes at construction, but the default
 must still be the conservative one. */
constexpr double kWorstDocumentedAccuracy = 5e-5;

using AccuracyTable =
    std::array<std::array<double, kNumShapeClasses>, kNumShapeClasses>;

const AccuracyTable& DocumentedAccuracyTable() {
  static const AccuracyTable table = []() {
    AccuracyTable t{};
    for (auto& row : t) row.fill(kWorstDocumentedAccuracy);
    const auto set = [&t](ShapeClass a, ShapeClass b, double value) {
      t[static_cast<int>(a)][static_cast<int>(b)] = value;
      t[static_cast<int>(b)][static_cast<int>(a)] = value;
    };
    using S = ShapeClass;
    set(S::kSphere, S::kSphere, 6e-15);
    set(S::kSphere, S::kBox, 4e-15);
    set(S::kSphere, S::kCapsule, 6e-15);
    set(S::kSphere, S::kCylinder, 5e-15);
    set(S::kSphere, S::kEllipsoid, 4e-5);
    set(S::kSphere, S::kConvex, 3e-6);
    set(S::kSphere, S::kMesh, 3e-6);
    set(S::kBox, S::kBox, 4e-15);
    set(S::kBox, S::kCapsule, 3e-6);
    set(S::kBox, S::kCylinder, 6e-6);
    set(S::kBox, S::kEllipsoid, 9e-6);
    set(S::kBox, S::kConvex, 3e-15);
    set(S::kBox, S::kMesh, 3e-15);
    set(S::kCapsule, S::kCapsule, 2e-5);
    set(S::kCapsule, S::kCylinder, 1e-5);
    set(S::kCapsule, S::kEllipsoid, 5e-6);
    set(S::kCapsule, S::kConvex, 2e-5);
    set(S::kCapsule, S::kMesh, 2e-5);
    set(S::kCylinder, S::kCylinder, 2e-5);
    set(S::kCylinder, S::kEllipsoid, 5e-5);
    set(S::kCylinder, S::kConvex, 6e-6);
    set(S::kCylinder, S::kMesh, 6e-6);
    set(S::kEllipsoid, S::kEllipsoid, 2e-5);
    set(S::kEllipsoid, S::kConvex, 9e-6);
    set(S::kEllipsoid, S::kMesh, 9e-6);
    set(S::kConvex, S::kConvex, 3e-15);
    set(S::kConvex, S::kMesh, 3e-15);
    set(S::kMesh, S::kMesh, 3e-15);
    // Drake supports exactly one halfspace combination natively (Sphere, at
    // 3e-15); the rest it refuses. Halfspace pairs never reach the
    // narrowphase here, because the capability probe routes every one of them
    // through the exact analytic support-function fallback and
    // ComputeTauTable() below never consults this table for a non-native
    // route. The entries are filled anyway, with the documented value where
    // there is one and the worst documented value otherwise, so that a future
    // routing change cannot inherit a τ of zero.
    set(S::kSphere, S::kHalfSpace, 3e-15);
    return t;
  }();
  return table;
}

/* τ_p for every pair of `pairs`. */
std::vector<double> ComputeTauTable(const RobotDiagram<double>& model,
                                    const std::vector<PairRecord>& pairs) {
  const drake::geometry::SceneGraphInspector<double>& inspector =
      model.scene_graph().model_inspector();
  const AccuracyTable& table = DocumentedAccuracyTable();
  std::vector<double> tau(pairs.size(), internal::kQueryTolerance);
  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    if (pairs[p].route != DistanceRoute::kNative) continue;  // exact.
    const int a = static_cast<int>(Classify(inspector.GetShape(pairs[p].a)));
    const int b = static_cast<int>(Classify(inspector.GetShape(pairs[p].b)));
    tau[p] = std::max(internal::kQueryTolerance, table[a][b]);
  }
  return tau;
}

/* Per-pair bounding-sphere slots for the broadphase prefilter. */
internal::PrefilterTable ComputePrefilterTable(
    const internal::KinematicsEngine& engine,
    const std::vector<PairRecord>& pairs) {
  internal::PrefilterTable table;
  table.slot_a.resize(pairs.size(), -1);
  table.slot_b.resize(pairs.size(), -1);
  std::unordered_map<GeometryId, int> slot_of;

  // HalfSpace has no bounding sphere, so such pairs skip the prefilter
  // entirely and go straight to the analytic oracle route, which is cheap
  // anyway. The oracle probe already found the halfspace: a pair is routed
  // kHalfSpaceA/kHalfSpaceB exactly when geometry a/b is one.
  const auto slot = [&](GeometryId id, BodyIndex body, bool is_half_space) {
    if (is_half_space) return -1;
    const auto it = slot_of.find(id);
    if (it != slot_of.end()) return it->second;
    const internal::BoundingSphere& sphere = engine.geometry_sphere(id);
    const int index = static_cast<int>(table.geometries.size());
    table.geometries.push_back(internal::PrefilterTable::Geometry{
        body, sphere.center_L, sphere.radius});
    slot_of.emplace(id, index);
    return index;
  };

  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    table.slot_a[p] = slot(pairs[p].a, pairs[p].body_a,
                           pairs[p].route == DistanceRoute::kHalfSpaceA);
    table.slot_b[p] = slot(pairs[p].b, pairs[p].body_b,
                           pairs[p].route == DistanceRoute::kHalfSpaceB);
  }
  return table;
}

void ValidateOptions(const Options& options) {
  // The displacement lemma argues entirely in the separated regime, so the
  // proof is meaningless for a negative threshold: a pair meant to touch must
  // be collision-filtered, not given a negative margin.
  if (!(options.margin >= 0.0) || !std::isfinite(options.margin)) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::margin must be a finite "
        "nonnegative distance; got {}. Filter a pair out instead of giving it "
        "a negative margin.",
        options.margin));
  }
  if (!(options.min_interval > 0.0) || !(options.min_interval <= 1.0)) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::min_interval is a fraction of a "
        "segment's parameter width and must lie in (0, 1]; got {}.",
        options.min_interval));
  }
}

}  // namespace

class ContinuousCollisionChecker::Impl {
 public:
  Impl(std::shared_ptr<const RobotDiagram<double>> model,
       const Options& default_options)
      : model_(std::move(model)),
        default_options_(default_options),
        engine_(*model_),
        oracle_(*model_),
        pairs_(oracle_.pairs()),
        tau_(ComputeTauTable(*model_, pairs_)),
        prefilter_(ComputePrefilterTable(engine_, pairs_)),
        pool_(*model_,
              std::max(1, default_options_.parallelism.num_threads())) {}

  const RobotDiagram<double>& model() const { return *model_; }

  const Options& Resolve(const std::optional<Options>& options) const {
    return options.has_value() ? *options : default_options_;
  }

  Result Check(const internal::PiecewiseBezierPath& path,
               const Options& options) const {
    ValidateOptions(options);
    const int expected = model_->plant().num_positions();
    if (path.num_positions() != expected) {
      throw std::runtime_error(fmt::format(
          "ContinuousCollisionChecker: the trajectory has {} rows but the "
          "plant has {} generalized positions.",
          path.num_positions(), expected));
    }

    // The λ table is per call: it depends on the trajectory's control box.
    const internal::MotionBoundTable table =
        engine_.ComputeMotionBoundTable(path, pairs_);

    internal::CertifierInput input;
    input.oracle = &oracle_;
    input.table = &table;
    input.path = &path;
    input.pairs = &pairs_;
    input.tau = &tau_;
    input.prefilter = &prefilter_;
    input.options = options;
    return internal::RunCertifier(input, &pool_);
  }

 private:
  std::shared_ptr<const RobotDiagram<double>> model_;
  Options default_options_;
  internal::KinematicsEngine engine_;
  internal::DistanceOracle oracle_;
  std::vector<PairRecord> pairs_;
  /* τ_p: max(kQueryTolerance, Drake's documented accuracy for the pair). */
  std::vector<double> tau_;
  internal::PrefilterTable prefilter_;
  mutable internal::ContextPool pool_;
};

ContinuousCollisionChecker::ContinuousCollisionChecker(
    std::shared_ptr<const RobotDiagram<double>> model,
    const Options& default_options) {
  if (model == nullptr) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: the model is null; supply a RobotDiagram "
        "whose plant is finalized.");
  }
  if (!model->plant().is_finalized()) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: the plant is not finalized; call "
        "MultibodyPlant::Finalize() (or RobotDiagramBuilder::Build()) first.");
  }
  ValidateOptions(default_options);
  impl_ = std::make_unique<Impl>(std::move(model), default_options);
}

ContinuousCollisionChecker::~ContinuousCollisionChecker() = default;

Result ContinuousCollisionChecker::CheckTrajectory(
    const drake::trajectories::Trajectory<double>& trajectory,
    const std::optional<Options>& options) const {
  const Options& resolved = impl_->Resolve(options);
  return impl_->Check(internal::PiecewiseBezierPath::FromTrajectory(
                          trajectory, resolved.continuous_revolute_indices),
                      resolved);
}

Result ContinuousCollisionChecker::CheckPath(
    const Eigen::MatrixXd& waypoints,
    const std::optional<Options>& options) const {
  const Options& resolved = impl_->Resolve(options);
  const int expected = impl_->model().plant().num_positions();
  if (waypoints.rows() != expected) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker::CheckPath: the waypoint matrix has {} "
        "rows but the plant has {} generalized positions (waypoints are "
        "columns).",
        waypoints.rows(), expected));
  }
  return impl_->Check(internal::PiecewiseBezierPath::FromWaypoints(waypoints),
                      resolved);
}

Result ContinuousCollisionChecker::CheckEdge(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const std::optional<Options>& options) const {
  const int expected = impl_->model().plant().num_positions();
  if (q1.size() != expected || q2.size() != expected) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker::CheckEdge: the endpoints have sizes {} "
        "and {} but the plant has {} generalized positions.",
        q1.size(), q2.size(), expected));
  }
  Eigen::MatrixXd waypoints(expected, 2);
  waypoints.col(0) = q1;
  waypoints.col(1) = q2;
  return CheckPath(waypoints, options);
}

const RobotDiagram<double>& ContinuousCollisionChecker::model() const {
  return impl_->model();
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
