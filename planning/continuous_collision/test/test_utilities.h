#pragma once

// Helpers shared by this package's tests: seeded random primitives and surface
// samplers, the throw-message probe, the checker factory, the random world
// generator two corpora are built from, and the corpus plus deep workload that
// concurrency_test.cc pins the driver's determinism against.
//
// Nothing here asserts; the claims live in the test files.

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/continuous_collision/continuous_collision_checker.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace test {

using drake::Parallelism;
using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Cylinder;
using drake::geometry::HalfSpace;
using drake::geometry::QueryObject;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::trajectories::BezierCurve;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using Rng = std::mt19937_64;

inline CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

inline SpatialInertia<double> Inertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

// ---------------------------------------------------------------------------
// Seeded random primitives.
//
// Every helper that draws more than one variate sequences them through named
// locals: the order in which a compiler evaluates sibling constructor or
// operator arguments is unspecified, so drawing inline would make a seeded
// corpus toolchain-dependent.
// ---------------------------------------------------------------------------

inline double Uniform(Rng* rng, double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(*rng);
}

inline int UniformInt(Rng* rng, int lo, int hi) {
  return std::uniform_int_distribution<int>(lo, hi)(*rng);
}

inline Vector3d UniformVector(Rng* rng, double lo, double hi) {
  const double x = Uniform(rng, lo, hi);
  const double y = Uniform(rng, lo, hi);
  const double z = Uniform(rng, lo, hi);
  return Vector3d(x, y, z);
}

inline Vector3d RandomUnitVector(Rng* rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Vector3d v;
  do {
    const double x = normal(*rng);
    const double y = normal(*rng);
    const double z = normal(*rng);
    v = Vector3d(x, y, z);
  } while (v.norm() < 1e-6);
  return v.normalized();
}

inline RotationMatrixd RandomRotation(Rng* rng) {
  return math::UniformlyRandomRotationMatrix<double>(rng);
}

inline RigidTransformd RandomTransform(Rng* rng, double scale) {
  const RotationMatrixd R = RandomRotation(rng);
  return RigidTransformd(R, UniformVector(rng, -scale, scale));
}

// ---------------------------------------------------------------------------
// Surface samplers: one point on the surface of a primitive, in its canonical
// geometry frame G. Exact area weighting is irrelevant to the property tests
// that use these; hitting every region of the surface is not.
// ---------------------------------------------------------------------------

using Sampler = std::function<Vector3d(Rng*)>;

inline Vector3d SampleSphere(Rng* rng, double radius) {
  return radius * RandomUnitVector(rng);
}

inline Vector3d SampleBox(Rng* rng, const Vector3d& size) {
  const Vector3d half = 0.5 * size;
  Vector3d p = UniformVector(rng, -1.0, 1.0).cwiseProduct(half);
  const int axis = UniformInt(rng, 0, 2);
  p(axis) = (UniformInt(rng, 0, 1) == 0 ? -1.0 : 1.0) * half(axis);
  return p;
}

inline Vector3d SampleCapsule(Rng* rng, double radius, double length) {
  const double half = 0.5 * length;
  if (UniformInt(rng, 0, 1) == 0) {  // Barrel.
    const double phi = Uniform(rng, 0.0, 2.0 * M_PI);
    const double z = Uniform(rng, -half, half);
    return Vector3d(radius * std::cos(phi), radius * std::sin(phi), z);
  }
  const Vector3d u = RandomUnitVector(rng);  // Cap.
  const double z0 = u.z() >= 0.0 ? half : -half;
  return Vector3d(radius * u.x(), radius * u.y(), z0 + radius * u.z());
}

inline Vector3d SampleCylinder(Rng* rng, double radius, double length) {
  const double half = 0.5 * length;
  const double phi = Uniform(rng, 0.0, 2.0 * M_PI);
  if (UniformInt(rng, 0, 1) == 0) {  // Barrel.
    const double z = Uniform(rng, -half, half);
    return Vector3d(radius * std::cos(phi), radius * std::sin(phi), z);
  }
  // Cap disk: the sqrt keeps the sample uniform in area, and hits the rim.
  const double rho = radius * std::sqrt(Uniform(rng, 0.0, 1.0));
  const double z = UniformInt(rng, 0, 1) == 0 ? -half : half;
  return Vector3d(rho * std::cos(phi), rho * std::sin(phi), z);
}

inline Vector3d SampleEllipsoid(Rng* rng, const Vector3d& radii) {
  return radii.cwiseProduct(RandomUnitVector(rng));
}

// `count` columns drawn from `sampler`.
inline Matrix3Xd SampleSurface(Rng* rng, int count, const Sampler& sampler) {
  Matrix3Xd p(3, count);
  for (int i = 0; i < count; ++i) p.col(i) = sampler(rng);
  return p;
}

// ---------------------------------------------------------------------------
// Checker plumbing.
// ---------------------------------------------------------------------------

// Runs `call`, requires it to throw, and returns the message, so the caller can
// assert on the several identifiers it must contain. (For a single identifier,
// prefer DRAKE_EXPECT_THROWS_MESSAGE.)
template <typename Callable>
std::string ThrowMessage(Callable&& call) {
  try {
    call();
  } catch (const std::exception& error) {
    return error.what();
  }
  ADD_FAILURE() << "expected an exception, but the call returned normally";
  return {};
}

inline ContinuousCollisionChecker::Params CheckerParams(
    std::shared_ptr<const RobotDiagram<double>> model, Options options,
    PaddingSpec padding = {}) {
  ContinuousCollisionChecker::Params params;
  params.model = std::move(model);
  params.default_options = std::move(options);
  params.padding = std::move(padding);
  return params;
}

// The checker is neither copyable nor movable, so tests that need to own one
// inside a container take the pointer flavor.
inline ContinuousCollisionChecker MakeChecker(
    std::shared_ptr<const RobotDiagram<double>> model, Options options,
    PaddingSpec padding = {}) {
  return ContinuousCollisionChecker(
      CheckerParams(std::move(model), std::move(options), std::move(padding)));
}

inline std::unique_ptr<ContinuousCollisionChecker> MakeCheckerPtr(
    std::shared_ptr<const RobotDiagram<double>> model, Options options,
    PaddingSpec padding = {}) {
  return std::make_unique<ContinuousCollisionChecker>(
      CheckerParams(std::move(model), std::move(options), std::move(padding)));
}

// Signed distance of `finding`'s pair, re-measured at the witness
// configuration from a fresh context: an independent confirmation that the
// witness is a real contact and not an artifact of the search.
inline double DistanceAtFinding(const ContinuousCollisionChecker& checker,
                                const Finding& finding) {
  const RobotDiagram<double>& model = checker.model();
  auto root = model.CreateDefaultContext();
  auto& plant_context = model.plant().GetMyMutableContextFromRoot(root.get());
  model.plant().SetPositions(&plant_context, finding.q);
  const auto& scene_graph = model.scene_graph();
  const auto& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(
          scene_graph.GetMyContextFromRoot(*root));
  for (const PairRecord& pair : checker.pairs()) {
    if (pair.id.a == finding.pair.a && pair.id.b == finding.pair.b) {
      return checker.distance_oracle().SignedDistance(query_object, pair);
    }
  }
  ADD_FAILURE() << "the finding names a pair the checker does not know.";
  return std::numeric_limits<double>::quiet_NaN();
}

// ---------------------------------------------------------------------------
// The random world generator behind two corpora.
// ---------------------------------------------------------------------------

// A chain of revolute joints (every third one prismatic) carrying small
// primitive geometry, plus anchored obstacles and, on odd seeds, a HalfSpace
// floor, so a corpus exercises the native narrowphase route and the analytic
// one. Link geometries are small next to the joint spacing, so adjacent links
// have real clearance in most configurations but can genuinely fold into each
// other: MultibodyPlant::Finalize only filters *welded* subgraphs, so every
// parent/child pair here is live.
struct WorldSpec {
  int num_links{4};
  int num_obstacles{4};
  bool floor{true};
};

inline std::unique_ptr<RobotDiagram<double>> MakeRandomWorld(
    uint64_t seed, const WorldSpec& spec = {}) {
  Rng rng(seed);
  const auto offset = [&rng](double lo, double hi) {
    const Vector3d unit = RandomUnitVector(&rng);
    const double length = Uniform(&rng, lo, hi);
    return Vector3d(unit * length);
  };

  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  std::vector<const RigidBody<double>*> links;
  for (int i = 0; i < spec.num_links; ++i) {
    const std::string name = "link" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    const RigidBody<double>& parent =
        (i == 0) ? plant.world_body() : *links.back();
    const Vector3d rpy_PF = UniformVector(&rng, -0.5, 0.5);
    const RigidTransformd X_PF(RollPitchYawd(rpy_PF), offset(0.22, 0.32));
    const Vector3d axis = RandomUnitVector(&rng);
    const std::string joint = "j" + std::to_string(i);
    if (i % 3 == 2) {
      plant.AddJoint<PrismaticJoint>(joint, parent, X_PF, body,
                                     RigidTransformd(), axis);
    } else {
      plant.AddJoint<RevoluteJoint>(joint, parent, X_PF, body,
                                    RigidTransformd(), axis);
    }
    const RigidTransformd X_LG(offset(0.10, 0.16));
    if (i % 2 == 0) {
      const double radius = Uniform(&rng, 0.02, 0.04);
      const double length = Uniform(&rng, 0.05, 0.10);
      plant.RegisterCollisionGeometry(body, X_LG, Capsule(radius, length),
                                      name + "_geom", Friction());
    } else {
      const Vector3d size = UniformVector(&rng, 0.04, 0.09);
      plant.RegisterCollisionGeometry(body, X_LG,
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    }
    links.push_back(&body);
  }
  for (int i = 0; i < spec.num_obstacles; ++i) {
    const std::string name = "obstacle" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    const Vector3d rpy_W = UniformVector(&rng, -3, 3);
    plant.WeldFrames(plant.world_frame(), body.body_frame(),
                     RigidTransformd(RollPitchYawd(rpy_W), offset(0.30, 0.75)));
    if (i % 3 == 0) {
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Sphere(Uniform(&rng, 0.05, 0.12)),
                                      name + "_geom", Friction());
    } else if (i % 3 == 1) {
      const Vector3d size = UniformVector(&rng, 0.08, 0.20);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    } else {
      const double radius = Uniform(&rng, 0.04, 0.09);
      const double length = Uniform(&rng, 0.08, 0.18);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Cylinder(radius, length), name + "_geom",
                                      Friction());
    }
  }
  if (spec.floor && seed % 2 == 1) {
    const RigidBody<double>& floor = plant.AddRigidBody("floor", Inertia());
    plant.WeldFrames(plant.world_frame(), floor.body_frame(),
                     RigidTransformd(Vector3d(0.0, 0.0, -0.5)));
    plant.RegisterCollisionGeometry(floor, RigidTransformd(), HalfSpace(),
                                    "floor_geom", Friction());
  }
  return builder.Build();
}

// ---------------------------------------------------------------------------
// The concurrency corpus and the deep workload derived from it.
// ---------------------------------------------------------------------------

constexpr double kMargin = 0.005;
// Ten cases keeps the full 4-thread-count x 2-mode sweep (80 certification
// runs) plus the concurrent-call test under a second in Release, which is what
// makes this affordable to run again under TSan (~100x slower).
constexpr int kNumCases = 10;
constexpr int kMinFreeCases = 3;
constexpr int kMinViolatingCases = 3;

// A quintic Bezier with random control points, so the corpus has real curved
// trajectories rather than straight edges.
inline Eigen::MatrixXd MakeControlPoints(uint64_t seed, int num_positions) {
  Rng rng(seed ^ 0xa5a5'5a5a'0f0f'f0f0ull);
  Eigen::MatrixXd points(num_positions, 6);
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < num_positions; ++i) {
      points(i, j) = Uniform(&rng, -1.4, 1.4);
    }
  }
  return points;
}

inline Options BaseOptions(Parallelism parallelism, SearchMode mode) {
  Options options;
  options.margin = kMargin;
  options.parallelism = parallelism;
  options.mode = mode;
  // Bounded cost per run: the whole sweep is executed 8 times per case.
  options.min_interval = 1e-6;
  return options;
}

struct Case {
  std::string name;
  std::shared_ptr<const RobotDiagram<double>> model;
  std::unique_ptr<ContinuousCollisionChecker> checker;
  Eigen::MatrixXd control_points;
  Verdict serial_verdict{};

  BezierCurve<double> trajectory() const {
    return BezierCurve<double>(0.0, 1.0, control_points);
  }
};

// Ten cases with at least three free and three violating, taken from the
// lowest seeds that supply them (deterministic, no hard-coded lucky numbers).
//
// The vector is allocated and never freed: it owns RobotDiagrams and checkers
// whose destruction would otherwise race Drake's own static teardown. Expect
// LSan to report it if an asan preset is ever added next to the tsan one.
inline const std::vector<std::unique_ptr<Case>>& Corpus() {
  static const std::vector<std::unique_ptr<Case>>* corpus = [] {
    auto* cases = new std::vector<std::unique_ptr<Case>>();
    int free_count = 0;
    int violating_count = 0;
    for (uint64_t seed = 1; seed <= 200; ++seed) {
      if (static_cast<int>(cases->size()) >= kNumCases) break;
      auto entry = std::make_unique<Case>();
      entry->name = "seed_" + std::to_string(seed);
      entry->model = MakeRandomWorld(seed);
      ContinuousCollisionChecker::Params params;
      params.model = entry->model;
      params.default_options =
          BaseOptions(Parallelism::None(), SearchMode::kCertifyAll);
      entry->checker = std::make_unique<ContinuousCollisionChecker>(params);
      entry->control_points =
          MakeControlPoints(seed, entry->model->plant().num_positions());
      const CertificationResult result = entry->checker->CheckTrajectory(
          entry->trajectory(),
          BaseOptions(Parallelism::None(), SearchMode::kCertifyAll));
      entry->serial_verdict = result.verdict;
      // Keep the corpus balanced: stop taking more of whichever kind is
      // already well represented.
      const bool is_free = result.verdict == Verdict::kCertifiedFree;
      const bool is_violating = result.verdict == Verdict::kViolationFound;
      if (!is_free && !is_violating) continue;
      if (is_free && free_count >= kNumCases - kMinViolatingCases) continue;
      if (is_violating && violating_count >= kNumCases - kMinFreeCases) {
        continue;
      }
      (is_free ? free_count : violating_count) += 1;
      cases->push_back(std::move(entry));
    }
    return cases;
  }();
  return *corpus;
}

// Bit-for-bit equality of two findings. Nothing here is a tolerance: two runs
// of the same deterministic computation either agree exactly or the claim of
// determinism is false.
inline ::testing::AssertionResult FindingsIdentical(
    const std::vector<Finding>& a, const std::vector<Finding>& b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure()
           << "finding counts differ: " << a.size() << " vs " << b.size();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].time != b[i].time) {
      return ::testing::AssertionFailure()
             << "finding " << i << " time " << a[i].time << " vs " << b[i].time;
    }
    if (a[i].q.size() != b[i].q.size() ||
        !(a[i].q.array() == b[i].q.array()).all()) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness configuration differs";
    }
    if (a[i].pair.a != b[i].pair.a || a[i].pair.b != b[i].pair.b) {
      return ::testing::AssertionFailure()
             << "finding " << i << " pair differs";
    }
    if (a[i].distance != b[i].distance ||
        a[i].motion_bound != b[i].motion_bound ||
        a[i].definite != b[i].definite) {
      return ::testing::AssertionFailure()
             << "finding " << i << " payload differs";
    }
    if (a[i].nearest_a_W.has_value() != b[i].nearest_a_W.has_value() ||
        (a[i].nearest_a_W.has_value() &&
         *a[i].nearest_a_W != *b[i].nearest_a_W)) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness point A differs";
    }
    if (a[i].nearest_b_W.has_value() != b[i].nearest_b_W.has_value() ||
        (a[i].nearest_b_W.has_value() &&
         *a[i].nearest_b_W != *b[i].nearest_b_W)) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness point B differs";
    }
  }
  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult EarliestWitnessIdentical(
    const CertificationResult& a, const CertificationResult& b) {
  if (a.findings.empty() != b.findings.empty()) {
    return ::testing::AssertionFailure()
           << "one run reported findings and the other did not";
  }
  if (a.findings.empty()) return ::testing::AssertionSuccess();
  return FindingsIdentical({a.findings.front()}, {b.findings.front()});
}

// The bisection's node budget below doubles as the deep workload's size: the
// margin it converges to is the largest one still certifiable inside this
// budget, so the tree it produces has just under this many nodes. Large enough
// that no fixed seeding depth could ever have covered it; small enough that the
// ~40 probes that find it stay cheap, sanitizers included. kMinDeepNodes is the
// floor concurrency_test.cc holds the result to, so the workload cannot
// silently degenerate if the corpus or the bisection drifts.
constexpr uint64_t kProbeBudget = 6000;
// Floors concurrency_test.cc holds the result to, so the workload cannot
// silently degenerate if the corpus or the bisection drifts. Depth is the load
// bearing one: a deep, narrow spike is the shape a depth-seeded work queue
// cannot split, and it is what the sharing path exists for.
constexpr uint64_t kMinDeepNodes = 1000;
constexpr int kMinDeepDepth = 15;

// A corpus case run at a margin just below its own swept clearance, which is
// what makes the subdivision tree deep and *narrow*: certifying a node needs
// phi - tau - Delta > m, so as the threshold approaches the trajectory's
// closest approach the motion bound has to be driven to nothing there and
// nowhere else. The result is thousands of nodes concentrated in a tiny
// sub-interval of one segment, which is the shape a depth-seeded work queue
// cannot split. That margin is found by bisection rather than hard-coded, so
// the workload survives any change to the random worlds, the bounds, or Drake.
struct DeepWorkload {
  const Case* entry{};
  double margin{0.0};
  double min_interval{1e-8};
  uint64_t nodes{0};
  int max_depth{0};

  Options options(Parallelism parallelism) const {
    Options options = BaseOptions(parallelism, SearchMode::kCertifyAll);
    options.margin = margin;
    options.min_interval = min_interval;
    return options;
  }
};

inline const DeepWorkload& Deep() {
  static const DeepWorkload* workload = []() {
    auto* deep = new DeepWorkload();
    for (const auto& entry : Corpus()) {
      if (entry->serial_verdict != Verdict::kCertifiedFree) continue;
      deep->entry = entry.get();
      break;
    }
    if (deep->entry == nullptr) return deep;

    const auto certifiable_within_budget = [&](double margin) {
      Options options = deep->options(Parallelism::None());
      options.margin = margin;
      options.max_nodes = kProbeBudget;
      return deep->entry->checker
                 ->CheckTrajectory(deep->entry->trajectory(), options)
                 .verdict == Verdict::kCertifiedFree;
    };
    double certifiable = 0.0;
    double grazing = kMargin;
    for (int i = 0; i < 12 && certifiable_within_budget(grazing); ++i) {
      certifiable = grazing;
      grazing *= 2.0;
    }
    for (int i = 0; i < 30; ++i) {
      const double mid = 0.5 * (certifiable + grazing);
      (certifiable_within_budget(mid) ? certifiable : grazing) = mid;
    }
    deep->margin = certifiable;
    const Statistics stats =
        deep->entry->checker
            ->CheckTrajectory(deep->entry->trajectory(),
                              deep->options(Parallelism::None()))
            .stats;
    deep->nodes = stats.nodes;
    deep->max_depth = stats.max_depth;
    return deep;
  }();
  return *workload;
}

}  // namespace test
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
