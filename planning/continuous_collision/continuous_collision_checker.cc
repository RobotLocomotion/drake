#include "drake/planning/continuous_collision/continuous_collision_checker.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/continuous_collision/certifier_internal.h"
#include "drake/planning/continuous_collision/shape_class.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::GeometryId;
using drake::multibody::BodyIndex;
using drake::planning::RobotDiagram;
using internal::Classify;
using internal::kNumShapeClasses;
using internal::ShapeClass;

// ---------------------------------------------------------------------------
// Per-pair oracle tolerance τ_p.
// ---------------------------------------------------------------------------
//
// Drake documents ComputeSignedDistancePairClosestPoints() accuracy as bad as
// 5e-5 m for some shape pairs, well outside the 1e-6 m default of
// Options::query_tolerance, and an oracle that over-reports a distance at or
// above the threshold can fake a certificate. Every use of τ (node
// certificate test, definite violation test, breakpoints, certificate replay)
// therefore takes τ_p = max(Options::query_tolerance,
// documented_accuracy(shape_a, shape_b)); pairs routed through the analytic
// halfspace fallback are closed-form and keep the raw
// Options::query_tolerance.
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

/* τ_p for every pair of `pairs`, given the call's query tolerance. */
std::vector<double> ComputeTauTable(const RobotDiagram<double>& model,
                                    const std::vector<PairRecord>& pairs,
                                    double query_tolerance) {
  const drake::geometry::SceneGraphInspector<double>& inspector =
      model.scene_graph().model_inspector();
  const AccuracyTable& table = DocumentedAccuracyTable();
  std::vector<double> tau(pairs.size(), query_tolerance);
  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    if (pairs[p].route != DistanceRoute::kNative) continue;  // exact.
    const int a = static_cast<int>(Classify(inspector.GetShape(pairs[p].id.a)));
    const int b = static_cast<int>(Classify(inspector.GetShape(pairs[p].id.b)));
    tau[p] = std::max(query_tolerance, table[a][b]);
  }
  return tau;
}

// ---------------------------------------------------------------------------
// Padding.
// ---------------------------------------------------------------------------
//
// PaddingSpec mirrors drake::planning::CollisionChecker: a pair's effective
// threshold is m_p = margin + padding(p), where padding comes from the dense
// per-body-pair matrix when one is supplied and otherwise from the {env, self}
// scalars. A pair is self iff both bodies are non-anchored and env otherwise.
// The rule is pure topology, so padding never depends on which trajectory is
// being checked; in particular the constant-coordinate carve-out, which can
// make a moving body behave as if welded for one trajectory, does not enter
// here.

std::vector<double> ComputePaddingTable(const KinematicsEngine& engine,
                                        const std::vector<PairRecord>& pairs,
                                        const PaddingSpec& padding) {
  const drake::multibody::MultibodyPlant<double>& plant = engine.plant();
  const int num_bodies = plant.num_bodies();
  if (padding.per_body_pair.has_value()) {
    const Eigen::MatrixXd& matrix = *padding.per_body_pair;
    if (matrix.rows() != num_bodies || matrix.cols() != num_bodies) {
      throw std::runtime_error(fmt::format(
          "ContinuousCollisionChecker: PaddingSpec::per_body_pair is "
          "{}x{} but must be {}x{} (one row and column per BodyIndex of the "
          "plant).",
          matrix.rows(), matrix.cols(), num_bodies, num_bodies));
    }
  }

  std::vector<bool> anchored(num_bodies, false);
  for (int b = 0; b < num_bodies; ++b) {
    anchored[b] = plant.IsAnchored(plant.get_body(BodyIndex(b)));
  }

  std::vector<double> result(pairs.size(), 0.0);
  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    const int a = static_cast<int>(pairs[p].id.body_a);
    const int b = static_cast<int>(pairs[p].id.body_b);
    double value = (!anchored[a] && !anchored[b]) ? padding.self_padding
                                                  : padding.env_padding;
    if (padding.per_body_pair.has_value()) {
      const double entry = (*padding.per_body_pair)(a, b);
      // A NaN entry means "not covered by the matrix"; fall back to the
      // scalars for that pair.
      if (!std::isnan(entry)) value = entry;
    }
    if (!std::isfinite(value)) {
      throw std::runtime_error(
          fmt::format("ContinuousCollisionChecker: padding for the body pair "
                      "({}, {}) is not finite.",
                      plant.get_body(BodyIndex(a)).name(),
                      plant.get_body(BodyIndex(b)).name()));
    }
    result[p] = value;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Prefilter table.
// ---------------------------------------------------------------------------

internal::PrefilterTable ComputePrefilterTable(
    const KinematicsEngine& engine, const std::vector<PairRecord>& pairs) {
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
    const BoundingSphere& sphere = engine.geometry_sphere(id);
    const int index = static_cast<int>(table.geometries.size());
    table.geometries.push_back(internal::PrefilterTable::Geometry{
        body, sphere.center_L, sphere.radius});
    slot_of.emplace(id, index);
    return index;
  };

  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    table.slot_a[p] = slot(pairs[p].id.a, pairs[p].id.body_a,
                           pairs[p].route == DistanceRoute::kHalfSpaceA);
    table.slot_b[p] = slot(pairs[p].id.b, pairs[p].id.body_b,
                           pairs[p].route == DistanceRoute::kHalfSpaceB);
  }
  return table;
}

void ValidateOptions(const Options& options) {
  if (!std::isfinite(options.margin)) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: Options::margin must be finite.");
  }
  if (!(options.query_tolerance >= 0.0) ||
      !std::isfinite(options.query_tolerance)) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::query_tolerance must be "
        "a finite non-negative distance; got {}.",
        options.query_tolerance));
  }
  if (!(options.certificate_slack >= 0.0) ||
      !std::isfinite(options.certificate_slack)) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::certificate_slack must "
        "be a finite non-negative distance; got {}.",
        options.certificate_slack));
  }
  if (!(options.min_interval > 0.0) || !(options.min_interval <= 1.0)) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::min_interval is a "
        "fraction of a segment's parameter width and must lie in (0, 1]; got "
        "{}.",
        options.min_interval));
  }
  if (options.max_reported_findings < 1) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker: Options::max_reported_findings "
        "must be at least 1; got {}.",
        options.max_reported_findings));
  }
  if (options.max_nodes.has_value() && *options.max_nodes == 0) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: Options::max_nodes must be at "
        "least 1 when set.");
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Impl.
// ---------------------------------------------------------------------------

class ContinuousCollisionChecker::Impl {
 public:
  explicit Impl(Params params)
      : model_(std::move(params.model)),
        default_options_(std::move(params.default_options)),
        engine_(*model_),
        oracle_(*model_, default_options_.query_tolerance),
        pairs_(oracle_.pairs()),
        padding_(ComputePaddingTable(engine_, pairs_, params.padding)),
        tau_base_(ComputeTauTable(*model_, pairs_,
                                  /* query_tolerance = */ 0.0)),
        prefilter_(ComputePrefilterTable(engine_, pairs_)),
        pool_(*model_,
              std::max(1, default_options_.parallelism.num_threads())) {
    pair_ids_.reserve(pairs_.size());
    for (int p = 0; p < static_cast<int>(pairs_.size()); ++p) {
      pair_ids_.push_back(pairs_[p].id);
      // pairs() reports the checker's default thresholds; every call rewrites
      // its own copy from that call's margin.
      pairs_[p].threshold = default_options_.margin + padding_[p];
    }
  }

  const Options& default_options() const { return default_options_; }
  const RobotDiagram<double>& model() const { return *model_; }
  const KinematicsEngine& engine() const { return engine_; }
  const DistanceOracle& oracle() const { return oracle_; }
  const std::vector<PairRecord>& pairs() const { return pairs_; }
  const std::vector<PairId>& pair_ids() const { return pair_ids_; }

  const Options& Resolve(const std::optional<Options>& options) const {
    return options.has_value() ? *options : default_options_;
  }

  void ValidatePath(const PiecewiseBezierPath& path) const {
    const int expected = model_->plant().num_positions();
    if (path.num_positions() != expected) {
      throw std::runtime_error(fmt::format(
          "ContinuousCollisionChecker: the trajectory has {} rows but "
          "the plant has {} generalized positions.",
          path.num_positions(), expected));
    }
  }

  CertificationResult Check(const PiecewiseBezierPath& path,
                            const Options& options) const {
    ValidateOptions(options);
    ValidatePath(path);
    const auto start = std::chrono::steady_clock::now();

    // Per-call: the λ table (it depends on the trajectory's control box), the
    // effective thresholds and the per-pair oracle tolerances.
    const MotionBoundTable table =
        engine_.ComputeMotionBoundTable(path, pair_ids_);
    std::vector<PairRecord> pairs = pairs_;
    std::vector<double> tau(pairs.size());
    for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
      pairs[p].threshold = options.margin + padding_[p];
      tau[p] = std::max(options.query_tolerance, tau_base_[p]);
      // The displacement lemma argues entirely in the separated regime, so
      // the certificate is meaningless for a negative effective threshold: a
      // pair meant to touch must be collision-filtered, not padded below
      // zero. Negative padding is therefore rejected rather than certified.
      if (pairs[p].threshold < 0.0) {
        throw std::runtime_error(fmt::format(
            "ContinuousCollisionChecker: margin ({}) + padding ({}) "
            "is negative for the pair on bodies {} and {}. The certificate "
            "is only proven for nonnegative thresholds; filter the pair out "
            "instead of using negative padding.",
            options.margin, padding_[p],
            model_->plant().get_body(pairs[p].id.body_a).name(),
            model_->plant().get_body(pairs[p].id.body_b).name()));
      }
    }

    internal::CertifierInput input;
    input.model = model_.get();
    input.oracle = &oracle_;
    input.table = &table;
    input.path = &path;
    input.pairs = &pairs;
    input.tau = &tau;
    input.prefilter = &prefilter_;
    input.options = options;

    internal::CertifierOutput output = internal::RunCertifier(input, &pool_);

    CertificationResult result;
    result.verdict = output.verdict;
    result.findings = std::move(output.findings);
    result.stats = output.stats;
    result.stats.wall_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    if (options.emit_certificate) {
      result.certificate = std::move(output.certificate);
    }
    return result;
  }

 private:
  std::shared_ptr<const RobotDiagram<double>> model_;
  Options default_options_;
  KinematicsEngine engine_;
  DistanceOracle oracle_;
  std::vector<PairRecord> pairs_;
  std::vector<PairId> pair_ids_;
  /* padding(p) alone; the margin is added per call. */
  std::vector<double> padding_;
  /* Drake's documented accuracy per pair; τ_p = max(query_tolerance, this). */
  std::vector<double> tau_base_;
  internal::PrefilterTable prefilter_;
  mutable internal::ContextPool pool_;
};

// ---------------------------------------------------------------------------
// ContinuousCollisionChecker.
// ---------------------------------------------------------------------------

ContinuousCollisionChecker::ContinuousCollisionChecker(Params params) {
  if (params.model == nullptr) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: Params::model is null; supply a "
        "RobotDiagram whose plant is finalized.");
  }
  if (!params.model->plant().is_finalized()) {
    throw std::runtime_error(
        "ContinuousCollisionChecker: the plant is not finalized; call "
        "MultibodyPlant::Finalize() (or RobotDiagramBuilder::Build()) first.");
  }
  ValidateOptions(params.default_options);
  impl_ = std::make_unique<Impl>(std::move(params));
}

ContinuousCollisionChecker::~ContinuousCollisionChecker() = default;

CertificationResult ContinuousCollisionChecker::CheckTrajectory(
    const drake::trajectories::Trajectory<double>& trajectory,
    const std::optional<Options>& options) const {
  const Options& resolved = impl_->Resolve(options);
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, resolved);
  return impl_->Check(path, resolved);
}

CertificationResult ContinuousCollisionChecker::CheckPath(
    const Eigen::MatrixXd& waypoints,
    const std::optional<Options>& options) const {
  const Options& resolved = impl_->Resolve(options);
  const int expected = impl_->model().plant().num_positions();
  if (waypoints.rows() != expected) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker::CheckPath: the waypoint matrix "
        "has {} rows but the plant has {} generalized positions (waypoints are "
        "columns).",
        waypoints.rows(), expected));
  }
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromWaypoints(waypoints, resolved);
  return impl_->Check(path, resolved);
}

CertificationResult ContinuousCollisionChecker::CheckEdge(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const std::optional<Options>& options) const {
  const int expected = impl_->model().plant().num_positions();
  if (q1.size() != expected || q2.size() != expected) {
    throw std::runtime_error(fmt::format(
        "ContinuousCollisionChecker::CheckEdge: the endpoints have "
        "sizes {} and {} but the plant has {} generalized positions.",
        q1.size(), q2.size(), expected));
  }
  Eigen::MatrixXd waypoints(expected, 2);
  waypoints.col(0) = q1;
  waypoints.col(1) = q2;
  return CheckPath(waypoints, options);
}

PiecewiseBezierPath ContinuousCollisionChecker::Normalize(
    const drake::trajectories::Trajectory<double>& trajectory,
    const std::optional<Options>& options) const {
  const Options& resolved = impl_->Resolve(options);
  PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, resolved);
  impl_->ValidatePath(path);
  return path;
}

MotionBoundTable ContinuousCollisionChecker::ComputeMotionBounds(
    const PiecewiseBezierPath& path) const {
  impl_->ValidatePath(path);
  return impl_->engine().ComputeMotionBoundTable(path, impl_->pair_ids());
}

const DistanceOracle& ContinuousCollisionChecker::distance_oracle() const {
  return impl_->oracle();
}

const KinematicsEngine& ContinuousCollisionChecker::kinematics_engine() const {
  return impl_->engine();
}

const std::vector<PairRecord>& ContinuousCollisionChecker::pairs() const {
  return impl_->pairs();
}

const RobotDiagram<double>& ContinuousCollisionChecker::model() const {
  return impl_->model();
}

// ---------------------------------------------------------------------------
// VerifyCertificate.
// ---------------------------------------------------------------------------
//
// Written against the checker's *public* introspection seams only: it
// re-derives the λ table from the path, re-restricts every record's control
// points with its own de Casteljau code, recomputes w about the record's qc,
// re-queries the oracle at qc from a fresh context, and re-checks the
// interval-certificate inequality with τ_p. It then verifies that the
// certified intervals cover [0, 1] of every segment for every pair. Nothing of
// the certifier's own bookkeeping is trusted.
//
// The replay charges the checker's construction-time query tolerance and the
// Options::certificate_slack default, so a run made with a *larger* slack, and
// hence a stricter certificate, still verifies.

bool VerifyCertificate(const ContinuousCollisionChecker& checker,
                       const PiecewiseBezierPath& path,
                       const Certificate& certificate) {
  const MotionBoundTable table = checker.ComputeMotionBounds(path);
  const std::vector<PairRecord>& pairs = checker.pairs();
  const std::vector<double> tau = ComputeTauTable(
      checker.model(), pairs, checker.distance_oracle().tolerance());

  internal::ReplayInput input;
  input.model = &checker.model();
  input.oracle = &checker.distance_oracle();
  input.table = &table;
  input.path = &path;
  input.pairs = &pairs;
  input.tau = &tau;
  input.slack = Options{}.certificate_slack;
  return internal::ReplayCertificate(input, certificate, nullptr);
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
