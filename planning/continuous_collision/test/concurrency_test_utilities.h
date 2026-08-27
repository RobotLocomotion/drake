#pragma once

// The shared fixture of the two concurrency targets: the random corpus that
// `concurrency_test.cc` pins the driver's determinism against, and the deep
// workload that both it and `concurrency_timing_test.cc` need. Each target
// builds its own copy lazily, so the fixture lives in a header rather than
// duplicating three hundred lines of world generation between the two files.
//
// Nothing here asserts; the claims live in the two test files.

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/shape_specification.h"
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
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::trajectories::BezierCurve;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kMargin = 0.005;
// Ten cases keeps the full 4-thread-count × 2-mode sweep (80 certification
// runs) plus the concurrent-call test under a second in Release, which is what
// makes this affordable to run again under TSan (~100× slower).
constexpr int kNumCases = 10;
constexpr int kMinFreeCases = 3;
constexpr int kMinViolatingCases = 3;

inline CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

inline SpatialInertia<double> Inertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

// A four-link chain of revolute and prismatic joints with primitive geometry,
// four anchored obstacles and (on odd seeds) a HalfSpace floor, so the corpus
// exercises the native narrowphase route and the analytic one.
inline std::unique_ptr<RobotDiagram<double>> MakeWorld(uint64_t seed) {
  std::mt19937_64 rng(seed);
  const auto uniform = [&rng](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  // Every helper below sequences its draws through named locals: the order in
  // which a compiler evaluates sibling constructor or operator arguments is
  // unspecified, so drawing inline would make the corpus toolchain-dependent
  // and could silently shift the free/violating balance this file relies on.
  const auto vector3 = [&uniform](double lo, double hi) {
    const double x = uniform(lo, hi);
    const double y = uniform(lo, hi);
    const double z = uniform(lo, hi);
    return Vector3d(x, y, z);
  };
  const auto direction = [&vector3]() {
    Vector3d v;
    do {
      v = vector3(-1, 1);
    } while (v.norm() < 1e-3 || v.norm() > 1.0);
    return v.normalized();
  };
  const auto offset = [&direction, &uniform](double lo, double hi) {
    const Vector3d unit = direction();
    const double length = uniform(lo, hi);
    return Vector3d(unit * length);
  };
  const auto pose = [&vector3, &offset](double lo, double hi) {
    const Vector3d rpy = vector3(-3, 3);
    const Vector3d p = offset(lo, hi);
    return RigidTransformd(RollPitchYawd(rpy), p);
  };

  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  std::vector<const RigidBody<double>*> links;
  for (int i = 0; i < 4; ++i) {
    const std::string name = "link" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    const RigidBody<double>& parent =
        (i == 0) ? plant.world_body() : *links.back();
    const Vector3d rpy_PF = vector3(-0.5, 0.5);
    const RigidTransformd X_PF(RollPitchYawd(rpy_PF), offset(0.22, 0.32));
    const Vector3d axis = direction();
    if (i == 2) {
      plant.AddJoint<PrismaticJoint>("j" + std::to_string(i), parent, X_PF,
                                     body, RigidTransformd(), axis);
    } else {
      plant.AddJoint<RevoluteJoint>("j" + std::to_string(i), parent, X_PF, body,
                                    RigidTransformd(), axis);
    }
    const RigidTransformd X_LG(offset(0.10, 0.16));
    if (i % 2 == 0) {
      const double radius = uniform(0.02, 0.04);
      const double length = uniform(0.05, 0.10);
      plant.RegisterCollisionGeometry(body, X_LG, Capsule(radius, length),
                                      name + "_geom", Friction());
    } else {
      const Vector3d size = vector3(0.04, 0.09);
      plant.RegisterCollisionGeometry(body, X_LG,
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    }
    links.push_back(&body);
  }
  for (int i = 0; i < 4; ++i) {
    const std::string name = "obstacle" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    plant.WeldFrames(plant.world_frame(), body.body_frame(), pose(0.30, 0.75));
    if (i % 3 == 0) {
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Sphere(uniform(0.05, 0.12)),
                                      name + "_geom", Friction());
    } else if (i % 3 == 1) {
      const Vector3d size = vector3(0.08, 0.20);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    } else {
      const double radius = uniform(0.04, 0.09);
      const double length = uniform(0.08, 0.18);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Cylinder(radius, length), name + "_geom",
                                      Friction());
    }
  }
  if (seed % 2 == 1) {
    const RigidBody<double>& floor = plant.AddRigidBody("floor", Inertia());
    plant.WeldFrames(plant.world_frame(), floor.body_frame(),
                     RigidTransformd(Vector3d(0.0, 0.0, -0.5)));
    plant.RegisterCollisionGeometry(floor, RigidTransformd(), HalfSpace(),
                                    "floor_geom", Friction());
  }
  return builder.Build();
}

// A quintic Bézier with random control points, so the corpus has real curved
// trajectories rather than straight edges.
inline Eigen::MatrixXd MakeControlPoints(uint64_t seed, int num_positions) {
  std::mt19937_64 rng(seed ^ 0xa5a5'5a5a'0f0f'f0f0ull);
  std::uniform_real_distribution<double> value(-1.4, 1.4);
  Eigen::MatrixXd points(num_positions, 6);
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < num_positions; ++i) points(i, j) = value(rng);
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
      entry->model = MakeWorld(seed);
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

// The bisection's node budget in Deep() doubles as the deep workload's size:
// the margin it converges to is the largest one still certifiable inside this
// budget, so the tree it produces has just under this many nodes. Large enough
// that a run takes tens of milliseconds (a wall-clock ratio then means
// something) and that no fixed seeding depth could ever have covered it; small
// enough that the ~40 probes that find it, and the timed repetitions
// concurrency_timing_test.cc runs on it, stay cheap, sanitizers included.
//
// kMinDeepNodes is the floor concurrency_test.cc holds the result to, so the
// workload cannot silently degenerate if the corpus or the bisection drifts.
constexpr uint64_t kProbeBudget = 6000;
constexpr uint64_t kMinDeepNodes = 3000;

// A corpus case run at a margin just below its own swept clearance, which is
// what makes the subdivision tree deep and *narrow*: certifying a node needs
// ϕ̂ − τ − Δ > m, so as the threshold m approaches the trajectory's closest
// approach the motion bound Δ has to be driven to nothing there and nowhere
// else. The result is thousands of nodes concentrated in a tiny sub-interval
// of one segment, which is the shape a depth-seeded work queue cannot split.
//
// That margin is found by bisection rather than hard-coded, so the workload
// survives any change to the random worlds, the bounds, or Drake: the largest
// margin still certifiable within kProbeBudget nodes is by construction the
// one that costs about kProbeBudget nodes.
struct DeepWorkload {
  const Case* entry{};
  double margin{0.0};
  double min_interval{1e-8};
  uint64_t nodes{0};

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
    deep->nodes = deep->entry->checker
                      ->CheckTrajectory(deep->entry->trajectory(),
                                        deep->options(Parallelism::None()))
                      .stats.nodes;
    return deep;
  }();
  return *workload;
}

}  // namespace test
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
