#pragma once

// Helpers shared by this package's tests: seeded random primitives and surface
// samplers, the throw-message probe, the random world generator two corpora are
// built from, and the corpus plus deep workload that concurrency_test.cc pins
// the driver's determinism against.
//
// Nothing here asserts; the claims live in the test files.

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
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
#include "drake/planning/continuous_collision/distance_oracle.h"
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

// The checker is neither copyable nor movable, so tests that need to own one
// inside a container take the pointer flavor.
inline std::unique_ptr<ContinuousCollisionChecker> MakeCheckerPtr(
    std::shared_ptr<const RobotDiagram<double>> model, const Options& options) {
  return std::make_unique<ContinuousCollisionChecker>(std::move(model),
                                                      options);
}

// Signed distance of `finding`'s pair, re-measured at the witness
// configuration from a fresh context and a fresh oracle: an independent
// confirmation that the witness is a real contact and not an artifact of the
// search.
inline double DistanceAtFinding(const RobotDiagram<double>& model,
                                const Finding& finding) {
  auto root = model.CreateDefaultContext();
  auto& plant_context = model.plant().GetMyMutableContextFromRoot(root.get());
  model.plant().SetPositions(&plant_context, finding.q);
  const auto& scene_graph = model.scene_graph();
  const auto& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(
          scene_graph.GetMyContextFromRoot(*root));
  const internal::DistanceOracle oracle(model);
  for (const internal::PairRecord& pair : oracle.pairs()) {
    if (pair.a == finding.geometry_a && pair.b == finding.geometry_b) {
      return oracle.SignedDistance(query_object, pair);
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
// Ten cases keeps the full 4-thread-count sweep plus the concurrent-call test
// under a second in Release, which is what makes this affordable to run again
// under TSan (~100x slower).
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

inline Options BaseOptions(Parallelism parallelism) {
  Options options;
  options.margin = kMargin;
  options.parallelism = parallelism;
  // Bounded cost per run: the whole sweep is executed several times per case.
  // See the note on DeepWorkload for why the resolution matters.
  options.distance_resolution = 1e-6;
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
      entry->checker =
          MakeCheckerPtr(entry->model, BaseOptions(Parallelism::None()));
      entry->control_points =
          MakeControlPoints(seed, entry->model->plant().num_positions());
      entry->serial_verdict =
          entry->checker->CheckTrajectory(entry->trajectory()).verdict;
      // Keep the corpus balanced: stop taking more of whichever kind is
      // already well represented.
      const bool is_free = entry->serial_verdict == Verdict::kCertifiedFree;
      const bool is_violating =
          entry->serial_verdict == Verdict::kViolationFound;
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

// Bit-for-bit equality of two reported witnesses. Nothing here is a tolerance:
// two runs of the same deterministic computation either agree exactly or the
// claim of determinism is false.
inline ::testing::AssertionResult FindingIdentical(
    const std::optional<Finding>& a, const std::optional<Finding>& b) {
  if (a.has_value() != b.has_value()) {
    return ::testing::AssertionFailure()
           << "one run reported a finding and the other did not";
  }
  if (!a.has_value()) return ::testing::AssertionSuccess();
  if (a->time != b->time) {
    return ::testing::AssertionFailure()
           << "time " << a->time << " vs " << b->time;
  }
  if (a->q.size() != b->q.size() || !(a->q.array() == b->q.array()).all()) {
    return ::testing::AssertionFailure() << "witness configuration differs";
  }
  if (a->geometry_a != b->geometry_a || a->geometry_b != b->geometry_b) {
    return ::testing::AssertionFailure() << "pair differs";
  }
  if (a->distance != b->distance) {
    return ::testing::AssertionFailure() << "distance differs";
  }
  if (a->nearest_a_W != b->nearest_a_W || a->nearest_b_W != b->nearest_b_W) {
    return ::testing::AssertionFailure() << "witness points differ";
  }
  return ::testing::AssertionSuccess();
}

// A corpus case run at a margin just below its own swept clearance, which is
// what makes the subdivision tree deep and *narrow*: certifying a node needs
// phi - tau - Delta > m, so as the threshold approaches the trajectory's
// closest approach the motion bound has to be driven to nothing there and
// nowhere else. The result is thousands of nodes concentrated in a tiny
// sub-interval of one segment, which is the shape a depth-seeded work queue
// cannot split. That margin is found by bisection rather than hard-coded, so
// the workload survives any change to the random worlds, the bounds, or Drake.
//
// kProbeBudget is the node count the bisection converges against, and
// kMinDeepNodes is the floor concurrency_test.cc holds the result to, so the
// workload cannot silently degenerate if the corpus or the bisection drifts.
constexpr uint64_t kProbeBudget = 6000;
constexpr uint64_t kMinDeepNodes = 800;

// The workload runs at its own resolution, finer than BaseOptions', because
// the resolution sets how deep a *certified* tree can get: a margin close
// enough to the tangency to need a deeper tree than the resolution allows
// ends kInconclusive instead, so the bisection converges on the largest
// margin whose tree certifies above the floor, and a finer resolution admits
// a deeper one. Measured on this corpus, the converged tree has 625 nodes at
// 1e-6, 847 at 1e-7, 1055 at 1e-8 and 1237 at 1e-9.
//
// The resolution is also what bounds the bisection's cost in an unoptimized
// build. Most probes land on a margin the search rejects, and a rejected
// probe keeps subdividing until every pair on every leaf is either certified
// or bounded to within the resolution, so its cost grows roughly linearly in
// 1 / resolution: the whole bisection visits ~6.5e4 nodes at 1e-8 but ~3.7e5
// at 1e-9, which is the difference between fitting the dbg test budget and
// straining it. (Before Options::max_nodes was withdrawn from the public API
// the probe bounded itself directly and the resolution did not have to.)
constexpr double kDeepResolution = 1e-8;

struct DeepWorkload {
  const Case* entry{};
  double margin{0.0};
  uint64_t num_nodes{0};

  Options options(Parallelism parallelism) const {
    Options options = BaseOptions(parallelism);
    options.margin = margin;
    options.distance_resolution = kDeepResolution;
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
      const Result result = deep->entry->checker->CheckTrajectory(
          deep->entry->trajectory(), options);
      return result.verdict == Verdict::kCertifiedFree &&
             result.num_nodes <= kProbeBudget;
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
    deep->num_nodes = deep->entry->checker
                          ->CheckTrajectory(deep->entry->trajectory(),
                                            deep->options(Parallelism::None()))
                          .num_nodes;
    return deep;
  }();
  return *workload;
}

}  // namespace test
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
