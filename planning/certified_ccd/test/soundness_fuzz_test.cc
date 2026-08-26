/// @file
/// T4 — end-to-end soundness fuzz (test plan T4; implementation note 2).
///
/// Random worlds × random trajectories, cross-checked three ways:
///
///   * a single sampled configuration whose clearance reaches the threshold
///     would refute a `kCertifiedFree` verdict outright, so every certified
///     case is searched for one — hard (10⁴ configurations, 10⁵ on a subset) —
///     and its emitted certificate is independently replayed;
///   * every definite `Finding` is re-evaluated exactly at its witness
///     configuration, from a context this run never touched, and must really
///     violate;
///   * every non-definite `Finding` that claims to be a resolution-floor
///     grazing record must be backed by a clearance that really sits within
///     10·(τ_p + ε) of the threshold near the reported time.
///
/// Any cross-check failure here is a soundness bug in the library, never a
/// reason to loosen the test (the implementation notes, item 2). Failure
/// messages carry the complete repro — seed, world recipe, trajectory control
/// points — so a failing case can be reconstructed from the CI log alone.
///
/// Budget. The gate is CI wall time, not case count: the dominant cost is the
/// dense cross-check (~10⁷ signed-distance queries per run), not certification.
/// kNumCases = 200 clears test-plan T4's ≥ 150 (world, trajectory) pairs per CI
/// run by a third and measures ~14 s in Release here — a 10× margin against the
/// ~3 min budget, so the suite still fits on a CI machine an order of magnitude
/// slower. The spare budget is spent on resolution rather than on more
/// shallowly-checked cases: kDenseSamples = 10⁴ resolves any clearance dip
/// wider than ~10⁻⁴ of the domain, and every 10th certified case gets the 10⁵
/// sweep, which resolves 10× finer at 10× the cost. (Sample counts are per
/// case and approximate: they are split evenly across segments and each segment
/// gets both endpoints, so the true count is total + #segments.)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/certified_ccd/certified_continuous_collision_checker.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace certified_ccd {
namespace {

using drake::Parallelism;
using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Convex;
using drake::geometry::Cylinder;
using drake::geometry::Ellipsoid;
using drake::geometry::GeometryId;
using drake::geometry::HalfSpace;
using drake::geometry::QueryObject;
using drake::geometry::Shape;
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
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr int kNumCases = 200;
constexpr uint64_t kBaseSeed = 0x5eed'0000'0000'0000ull;
constexpr int kDenseSamples = 10000;
constexpr int kDeepDenseSamples = 100000;
/// Every kDeepEvery-th certified case gets the 10⁵-sample sweep.
constexpr int kDeepEvery = 10;
/// Samples used to locate a trajectory's minimum clearance when building a
/// deliberately grazing case.
constexpr int kGrazeProbeSamples = 2000;

/// The worst signed-distance accuracy Drake documents for any supported shape
/// combination (query_object.h Table 4, Cylinder–Ellipsoid). The checker
/// charges each pair its own τ_p ≥ Options::query_tolerance; the tests below
/// only ever need an upper bound on it, and this is it.
constexpr double kWorstTau = 5e-5;

// ---------------------------------------------------------------------------
// Recipes. Everything random about a case lives in these structs, and every
// one of them prints itself, so a failure message is a complete repro.
// ---------------------------------------------------------------------------

enum class ShapeKind {
  kSphere,
  kBox,
  kCapsule,
  kCylinder,
  kEllipsoid,
  kConvex
};

struct ShapeSpec {
  ShapeKind kind{ShapeKind::kSphere};
  /// Sphere: (r, ·, ·). Box: full (w, d, h). Capsule/Cylinder: (r, length, ·).
  /// Ellipsoid: (a, b, c). Convex: (scale, ·, ·) of a regular tetrahedron.
  Vector3d dims{Vector3d::Zero()};
};

std::string Name(ShapeKind kind) {
  switch (kind) {
    case ShapeKind::kSphere:
      return "Sphere";
    case ShapeKind::kBox:
      return "Box";
    case ShapeKind::kCapsule:
      return "Capsule";
    case ShapeKind::kCylinder:
      return "Cylinder";
    case ShapeKind::kEllipsoid:
      return "Ellipsoid";
    case ShapeKind::kConvex:
      return "ConvexTetra";
  }
  return "?";
}

/// A regular tetrahedron of circumradius √3·`scale`, as a vertex matrix; Drake
/// takes the convex hull of these points.
Eigen::Matrix3Xd TetrahedronPoints(double scale) {
  Eigen::Matrix3Xd points(3, 4);
  points.col(0) = scale * Vector3d(1, 1, 1);
  points.col(1) = scale * Vector3d(1, -1, -1);
  points.col(2) = scale * Vector3d(-1, 1, -1);
  points.col(3) = scale * Vector3d(-1, -1, 1);
  return points;
}

std::unique_ptr<Shape> MakeShape(const ShapeSpec& spec) {
  switch (spec.kind) {
    case ShapeKind::kSphere:
      return std::make_unique<Sphere>(spec.dims[0]);
    case ShapeKind::kBox:
      return std::make_unique<Box>(spec.dims[0], spec.dims[1], spec.dims[2]);
    case ShapeKind::kCapsule:
      return std::make_unique<Capsule>(spec.dims[0], spec.dims[1]);
    case ShapeKind::kCylinder:
      return std::make_unique<Cylinder>(spec.dims[0], spec.dims[1]);
    case ShapeKind::kEllipsoid:
      return std::make_unique<Ellipsoid>(spec.dims[0], spec.dims[1],
                                         spec.dims[2]);
    case ShapeKind::kConvex:
      return std::make_unique<Convex>(TetrahedronPoints(spec.dims[0]),
                                      "fuzz_tetra");
  }
  throw std::logic_error("unreachable");
}

enum class JointKind { kRevolute, kPrismatic };

struct LinkSpec {
  /// Index into WorldRecipe::links, or -1 for the world body.
  int parent{-1};
  JointKind joint{JointKind::kRevolute};
  Vector3d axis{Vector3d::UnitZ()};
  /// The joint's frame on the parent: rotation (rpy) and translation.
  Vector3d rpy_PF{Vector3d::Zero()};
  Vector3d p_PF{Vector3d::Zero()};
  /// The link geometry's pose in the link frame.
  Vector3d p_LG{Vector3d::Zero()};
  ShapeSpec shape;
};

struct ObstacleSpec {
  Vector3d p_W{Vector3d::Zero()};
  Vector3d rpy_W{Vector3d::Zero()};
  ShapeSpec shape;
};

struct WorldRecipe {
  uint64_t seed{0};
  std::vector<LinkSpec> links;
  std::vector<ObstacleSpec> obstacles;
  /// An anchored HalfSpace floor (exercises the analytic distance route).
  bool floor{false};
  double floor_z{-0.45};

  int num_positions() const { return static_cast<int>(links.size()); }
  std::string Describe() const;
};

enum class TrajectoryKind { kPwl, kBezier, kBspline };

struct TrajectoryRecipe {
  TrajectoryKind kind{TrajectoryKind::kBezier};
  /// Bézier order (1…5) or B-spline order (4). Unused for PWL.
  int order{1};
  /// n × K: waypoints (PWL) or control points (Bézier / B-spline).
  Eigen::MatrixXd points;
  std::string Describe() const;
};

std::string FormatVector(const Vector3d& v) {
  std::ostringstream out;
  out << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
  return out.str();
}

std::string WorldRecipe::Describe() const {
  std::ostringstream out;
  out.precision(17);
  out << "world seed=" << seed << " links=" << links.size()
      << " obstacles=" << obstacles.size()
      << " floor=" << (floor ? "yes" : "no") << "\n";
  for (std::size_t i = 0; i < links.size(); ++i) {
    const LinkSpec& link = links[i];
    out << "  link" << i << ": parent="
        << (link.parent < 0 ? std::string("world")
                            : "link" + std::to_string(link.parent))
        << " joint="
        << (link.joint == JointKind::kRevolute ? "revolute" : "prismatic")
        << " axis=" << FormatVector(link.axis)
        << " rpy_PF=" << FormatVector(link.rpy_PF)
        << " p_PF=" << FormatVector(link.p_PF)
        << " p_LG=" << FormatVector(link.p_LG)
        << " shape=" << Name(link.shape.kind) << FormatVector(link.shape.dims)
        << "\n";
  }
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const ObstacleSpec& obstacle = obstacles[i];
    out << "  obstacle" << i << ": p_W=" << FormatVector(obstacle.p_W)
        << " rpy_W=" << FormatVector(obstacle.rpy_W)
        << " shape=" << Name(obstacle.shape.kind)
        << FormatVector(obstacle.shape.dims) << "\n";
  }
  if (floor) out << "  floor: HalfSpace at z = " << floor_z << "\n";
  return out.str();
}

std::string TrajectoryRecipe::Describe() const {
  std::ostringstream out;
  out.precision(17);
  out << "trajectory kind="
      << (kind == TrajectoryKind::kPwl
              ? "PWL"
              : (kind == TrajectoryKind::kBezier ? "Bezier" : "Bspline"))
      << " order=" << order << " points(" << points.rows() << "x"
      << points.cols() << "):\n";
  for (int i = 0; i < points.rows(); ++i) {
    out << "    [";
    for (int j = 0; j < points.cols(); ++j) {
      out << (j > 0 ? ", " : "") << points(i, j);
    }
    out << "]\n";
  }
  return out.str();
}

// ---------------------------------------------------------------------------
// Random generation.
// ---------------------------------------------------------------------------

class Rng {
 public:
  explicit Rng(uint64_t seed) : engine_(seed) {}

  double Uniform(double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(engine_);
  }
  int Int(int lo, int hi) {
    return std::uniform_int_distribution<int>(lo, hi)(engine_);
  }
  bool Bernoulli(double p) { return std::bernoulli_distribution(p)(engine_); }
  /// Note the named locals: the order in which a compiler evaluates sibling
  /// constructor arguments is unspecified, so drawing three variates inline
  /// would make the corpus depend on the toolchain. Every draw in this file is
  /// sequenced explicitly for that reason.
  Vector3d UniformVector(double lo, double hi) {
    const double x = Uniform(lo, hi);
    const double y = Uniform(lo, hi);
    const double z = Uniform(lo, hi);
    return Vector3d(x, y, z);
  }
  /// A uniformly distributed direction (rejection-sampled, so no pole bias).
  Vector3d Direction() {
    while (true) {
      const Vector3d v = UniformVector(-1.0, 1.0);
      const double n = v.norm();
      if (n > 1e-3 && n <= 1.0) return v / n;
    }
  }
  /// A direction scaled by a length drawn *after* it.
  Vector3d Offset(double lo, double hi) {
    const Vector3d direction = Direction();
    const double length = Uniform(lo, hi);
    return direction * length;
  }

 private:
  std::mt19937_64 engine_;
};

/// Link geometries stay small (≤ 5 cm half-extent) and sit ~12–18 cm out along
/// the link, while joints are ~25–35 cm apart. Adjacent links therefore have
/// real clearance in most configurations but can genuinely fold into each
/// other, which is what makes the self-collision half of the corpus nontrivial.
/// (MultibodyPlant::Finalize only filters *welded* subgraphs, so every
/// parent/child pair here is a live, unfiltered pair.)
ShapeSpec RandomLinkShape(Rng* rng) {
  ShapeSpec spec;
  const int roll = rng->Int(0, 11);
  if (roll <= 2) {
    spec.kind = ShapeKind::kSphere;
    spec.dims[0] = rng->Uniform(0.02, 0.05);
  } else if (roll <= 5) {
    spec.kind = ShapeKind::kBox;
    spec.dims = rng->UniformVector(0.04, 0.10);
  } else if (roll <= 7) {
    spec.kind = ShapeKind::kCapsule;
    spec.dims[0] = rng->Uniform(0.02, 0.04);
    spec.dims[1] = rng->Uniform(0.04, 0.12);
  } else if (roll <= 9) {
    spec.kind = ShapeKind::kCylinder;
    spec.dims[0] = rng->Uniform(0.02, 0.04);
    spec.dims[1] = rng->Uniform(0.04, 0.12);
  } else if (roll == 10) {
    spec.kind = ShapeKind::kEllipsoid;
    spec.dims = rng->UniformVector(0.02, 0.06);
  } else {
    spec.kind = ShapeKind::kConvex;
    spec.dims[0] = rng->Uniform(0.02, 0.04);
  }
  return spec;
}

ShapeSpec RandomObstacleShape(Rng* rng) {
  ShapeSpec spec;
  const int roll = rng->Int(0, 9);
  if (roll <= 3) {
    spec.kind = ShapeKind::kBox;
    spec.dims = rng->UniformVector(0.06, 0.22);
  } else if (roll <= 6) {
    spec.kind = ShapeKind::kSphere;
    spec.dims[0] = rng->Uniform(0.04, 0.11);
  } else if (roll <= 8) {
    spec.kind = ShapeKind::kCapsule;
    spec.dims[0] = rng->Uniform(0.03, 0.08);
    spec.dims[1] = rng->Uniform(0.06, 0.20);
  } else {
    spec.kind = ShapeKind::kConvex;
    spec.dims[0] = rng->Uniform(0.05, 0.10);
  }
  return spec;
}

WorldRecipe RandomWorld(uint64_t seed) {
  Rng rng(seed);
  WorldRecipe recipe;
  recipe.seed = seed;
  const int num_links = rng.Int(2, 5);
  for (int i = 0; i < num_links; ++i) {
    LinkSpec link;
    // A chain most of the time, a small tree otherwise: link i hangs off a
    // uniformly chosen earlier link (or the world for link 0).
    link.parent =
        (i == 0) ? -1 : (rng.Bernoulli(0.72) ? i - 1 : rng.Int(0, i - 1));
    link.joint =
        rng.Bernoulli(0.7) ? JointKind::kRevolute : JointKind::kPrismatic;
    link.axis = rng.Direction();
    link.rpy_PF = rng.UniformVector(-0.6, 0.6);
    link.p_PF = rng.Offset(0.25, 0.35);
    link.p_LG = rng.Offset(0.12, 0.18);
    link.shape = RandomLinkShape(&rng);
    recipe.links.push_back(link);
  }
  const int num_obstacles = rng.Int(2, 6);
  for (int i = 0; i < num_obstacles; ++i) {
    ObstacleSpec obstacle;
    obstacle.p_W = rng.Offset(0.25, 0.80);
    obstacle.rpy_W = rng.UniformVector(-3.0, 3.0);
    obstacle.shape = RandomObstacleShape(&rng);
    recipe.obstacles.push_back(obstacle);
  }
  recipe.floor = rng.Bernoulli(0.3);
  recipe.floor_z = rng.Uniform(-0.6, -0.35);
  return recipe;
}

std::unique_ptr<RobotDiagram<double>> BuildWorld(const WorldRecipe& recipe) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto robot = plant.AddModelInstance("robot");
  const auto env = plant.AddModelInstance("env");
  const CoulombFriction<double> friction(1.0, 1.0);
  const SpatialInertia<double> inertia =
      SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);

  std::vector<const RigidBody<double>*> bodies;
  for (std::size_t i = 0; i < recipe.links.size(); ++i) {
    const LinkSpec& link = recipe.links[i];
    const std::string name = "link" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, robot, inertia);
    const RigidBody<double>& parent =
        link.parent < 0 ? plant.world_body() : *bodies[link.parent];
    const RigidTransformd X_PF(RollPitchYawd(link.rpy_PF), link.p_PF);
    if (link.joint == JointKind::kRevolute) {
      plant.AddJoint<RevoluteJoint>("j" + std::to_string(i), parent, X_PF, body,
                                    RigidTransformd(), link.axis);
    } else {
      plant.AddJoint<PrismaticJoint>("j" + std::to_string(i), parent, X_PF,
                                     body, RigidTransformd(), link.axis);
    }
    plant.RegisterCollisionGeometry(body, RigidTransformd(link.p_LG),
                                    *MakeShape(link.shape), name + "_geom",
                                    friction);
    bodies.push_back(&body);
  }
  for (std::size_t i = 0; i < recipe.obstacles.size(); ++i) {
    const ObstacleSpec& obstacle = recipe.obstacles[i];
    const std::string name = "obstacle" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, env, inertia);
    plant.WeldFrames(
        plant.world_frame(), body.body_frame(),
        RigidTransformd(RollPitchYawd(obstacle.rpy_W), obstacle.p_W));
    plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                    *MakeShape(obstacle.shape), name + "_geom",
                                    friction);
  }
  if (recipe.floor) {
    const RigidBody<double>& body = plant.AddRigidBody("floor", env, inertia);
    plant.WeldFrames(plant.world_frame(), body.body_frame(),
                     RigidTransformd(Vector3d(0.0, 0.0, recipe.floor_z)));
    plant.RegisterCollisionGeometry(body, RigidTransformd(), HalfSpace(),
                                    "floor_geom", friction);
  }
  return builder.Build();
}

/// Random control/waypoint columns around a random centre. `excursion` scales
/// the amplitude: small excursions mostly stay free, large ones sweep across
/// the obstacle field, and the range is chosen so the corpus lands on a mix of
/// certified / violating / grazing outcomes (asserted at the end of the run).
TrajectoryRecipe RandomTrajectory(const WorldRecipe& world, Rng* rng) {
  const int n = world.num_positions();
  VectorXd centre(n);
  VectorXd amplitude(n);
  const double excursion = rng->Uniform(0.15, 1.6);
  for (int i = 0; i < n; ++i) {
    const bool prismatic = world.links[i].joint == JointKind::kPrismatic;
    centre[i] = prismatic ? rng->Uniform(-0.10, 0.10) : rng->Uniform(-2.0, 2.0);
    amplitude[i] = excursion * (prismatic ? 0.15 : 1.2);
  }

  TrajectoryRecipe recipe;
  const int kind_roll = rng->Int(0, 2);
  int columns = 0;
  if (kind_roll == 0) {
    recipe.kind = TrajectoryKind::kPwl;
    recipe.order = 1;
    columns = rng->Int(2, 5);
  } else if (kind_roll == 1) {
    recipe.kind = TrajectoryKind::kBezier;
    recipe.order = rng->Int(1, 5);
    columns = recipe.order + 1;
  } else {
    recipe.kind = TrajectoryKind::kBspline;
    recipe.order = 4;
    columns = rng->Int(4, 7);
  }
  recipe.points.resize(n, columns);
  for (int j = 0; j < columns; ++j) {
    for (int i = 0; i < n; ++i) {
      recipe.points(i, j) = centre[i] + amplitude[i] * rng->Uniform(-1.0, 1.0);
    }
  }
  return recipe;
}

std::unique_ptr<Trajectory<double>> BuildTrajectory(
    const TrajectoryRecipe& recipe) {
  switch (recipe.kind) {
    case TrajectoryKind::kPwl: {
      // A first-order hold is a PiecewisePolynomial, so this also exercises
      // trajectory normalization's monomial → Bernstein conversion route.
      const int columns = static_cast<int>(recipe.points.cols());
      VectorXd breaks(columns);
      for (int j = 0; j < columns; ++j) breaks[j] = j;
      return std::make_unique<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(breaks, recipe.points));
    }
    case TrajectoryKind::kBezier:
      return std::make_unique<BezierCurve<double>>(0.0, 1.0, recipe.points);
    case TrajectoryKind::kBspline: {
      std::vector<Eigen::MatrixXd> control_points;
      for (int j = 0; j < recipe.points.cols(); ++j) {
        control_points.push_back(recipe.points.col(j));
      }
      return std::make_unique<BsplineTrajectory<double>>(
          drake::math::BsplineBasis<double>(
              recipe.order, static_cast<int>(control_points.size())),
          control_points);
    }
  }
  throw std::logic_error("unreachable");
}

// ---------------------------------------------------------------------------
// The independent dense cross-check.
// ---------------------------------------------------------------------------

/// Radius of the smallest sphere about the *geometry frame origin* containing
/// the shape. Deliberately re-derived here — six exact one-liners, each
/// obviously correct — rather than reused from the library, so the broadphase
/// this cross-check uses to skip far pairs cannot inherit a bug from the code
/// it is auditing. std::nullopt means "no finite radius available" (HalfSpace)
/// or "not worth deriving here" (Convex / Mesh); such pairs always take the
/// narrowphase.
std::optional<double> LocalRadius(const Shape& shape) {
  return shape.Visit<std::optional<double>>(
      [](const auto& s) -> std::optional<double> {
        using S = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<S, Sphere>) {
          return s.radius();
        } else if constexpr (std::is_same_v<S, Box>) {
          return 0.5 * s.size().norm();
        } else if constexpr (std::is_same_v<S, Capsule>) {
          return 0.5 * s.length() + s.radius();
        } else if constexpr (std::is_same_v<S, Cylinder>) {
          return std::hypot(0.5 * s.length(), s.radius());
        } else if constexpr (std::is_same_v<S, Ellipsoid>) {
          return std::max({s.a(), s.b(), s.c()});
        } else {
          return std::nullopt;
        }
      });
}

/// Per-checker scaffolding for the dense scan: a dense list of the geometries
/// that appear in some pair, their local radii, and each pair's two slots.
class DenseScanner {
 public:
  explicit DenseScanner(const CertifiedContinuousCollisionChecker& checker)
      : checker_(&checker),
        root_(checker.model().CreateDefaultContext()),
        plant_context_(
            &checker.model().plant().GetMyMutableContextFromRoot(root_.get())) {
    const auto& inspector = checker.model().scene_graph().model_inspector();
    const auto slot = [&](GeometryId id) {
      for (std::size_t i = 0; i < geometries_.size(); ++i) {
        if (geometries_[i] == id) return static_cast<int>(i);
      }
      geometries_.push_back(id);
      radius_.push_back(LocalRadius(inspector.GetShape(id)));
      centre_.push_back(Vector3d::Zero());
      return static_cast<int>(geometries_.size()) - 1;
    };
    for (const PairRecord& pair : checker.pairs()) {
      slot_a_.push_back(slot(pair.id.a));
      slot_b_.push_back(slot(pair.id.b));
    }
  }

  /// Worst (most negative) value of φ_p(q) − threshold over the dense samples,
  /// with the time and pair that attained it.
  struct Result {
    double min_slack{std::numeric_limits<double>::infinity()};
    double worst_time{std::numeric_limits<double>::quiet_NaN()};
    int worst_pair{-1};
  };

  /// `threshold` is m_p, which this fuzz keeps uniform across pairs because it
  /// never sets a PaddingSpec (the case loop asserts that).
  Result Scan(const PiecewiseBezierPath& path, int total_samples,
              double threshold) {
    const int num_segments = static_cast<int>(path.segments().size());
    const int per_segment = std::max(2, total_samples / num_segments);
    Result result;
    for (int k = 0; k < num_segments; ++k) {
      const BezierSegment& segment = path.segments()[k];
      for (int i = 0; i <= per_segment; ++i) {
        const double s = static_cast<double>(i) / per_segment;
        const double time =
            segment.t_start + s * (segment.t_end - segment.t_start);
        Evaluate(path.EvaluateSegment(k, s), time, threshold, &result);
      }
    }
    return result;
  }

  /// min over samples in [t − half_width, t + half_width] of |φ_p − m_p| for
  /// one pair: the "is this really grazing?" check for kInconclusive.
  double MinAbsSlackNear(const PiecewiseBezierPath& path, int pair_index,
                         double threshold, double time, double half_width,
                         int samples) {
    const double lo = std::max(path.start_time(), time - half_width);
    const double hi = std::min(path.end_time(), time + half_width);
    const PairRecord& pair = checker_->pairs()[pair_index];
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= samples; ++i) {
      const double u = (samples == 0) ? 0.0 : static_cast<double>(i) / samples;
      const double t = lo + u * (hi - lo);
      SetPositions(path.Value(t));
      ++narrowphase_queries_;
      best = std::min(best, std::abs(checker_->distance_oracle().SignedDistance(
                                         query_object(), pair) -
                                     threshold));
    }
    return best;
  }

  int64_t narrowphase_queries() const { return narrowphase_queries_; }

  /// Signed distance of one pair at an arbitrary configuration, from this
  /// scanner's own fresh context.
  double DistanceAt(const VectorXd& q, int pair_index) {
    SetPositions(q);
    ++narrowphase_queries_;
    return checker_->distance_oracle().SignedDistance(
        query_object(), checker_->pairs()[pair_index]);
  }

  /// Index of the checker's pair matching `id`, or -1.
  int FindPair(const PairId& id) const {
    const auto& pairs = checker_->pairs();
    for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
      if (pairs[p].id.a == id.a && pairs[p].id.b == id.b) return p;
    }
    return -1;
  }

 private:
  void SetPositions(const VectorXd& q) {
    checker_->model().plant().SetPositions(plant_context_, q);
  }

  const QueryObject<double>& query_object() const {
    const auto& scene_graph = checker_->model().scene_graph();
    return scene_graph.get_query_output_port().Eval<QueryObject<double>>(
        scene_graph.GetMyContextFromRoot(*root_));
  }

  void Evaluate(const VectorXd& q, double time, double threshold,
                Result* result) {
    SetPositions(q);
    const QueryObject<double>& query = query_object();
    for (std::size_t i = 0; i < geometries_.size(); ++i) {
      centre_[i] = query.GetPoseInWorld(geometries_[i]).translation();
    }
    const auto& pairs = checker_->pairs();
    for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
      const std::optional<double>& ra = radius_[slot_a_[p]];
      const std::optional<double>& rb = radius_[slot_b_[p]];
      if (ra.has_value() && rb.has_value()) {
        // φ_p ≥ ‖c_a − c_b‖ − R_a − R_b: a pair whose *lower bound* already
        // clears the threshold cannot be the worst one, so skip its
        // narrowphase. This is what makes 10⁴ (and 10⁵) samples per case
        // affordable; it can only ever cause the scan to miss a violation if
        // one of the five radius formulas above under-bounds its shape, which
        // is why they are exact circumradii and not estimates.
        const double lower =
            (centre_[slot_a_[p]] - centre_[slot_b_[p]]).norm() - *ra - *rb;
        if (lower > threshold) continue;
      }
      ++narrowphase_queries_;
      const double slack =
          checker_->distance_oracle().SignedDistance(query, pairs[p]) -
          threshold;
      if (slack < result->min_slack) {
        result->min_slack = slack;
        result->worst_time = time;
        result->worst_pair = p;
      }
    }
  }

  const CertifiedContinuousCollisionChecker* checker_{};
  std::unique_ptr<drake::systems::Context<double>> root_;
  drake::systems::Context<double>* plant_context_{};
  std::vector<GeometryId> geometries_;
  std::vector<std::optional<double>> radius_;
  std::vector<Vector3d> centre_;
  std::vector<int> slot_a_;
  std::vector<int> slot_b_;
  int64_t narrowphase_queries_{0};
};

// ---------------------------------------------------------------------------
// The fuzz itself.
// ---------------------------------------------------------------------------

struct Tally {
  int certified{0};
  int violation{0};
  int inconclusive{0};
  int budget{0};
  int deep_scans{0};
  int definite_findings{0};
  int inconclusive_findings{0};
  int graze_cases{0};
  int pwl{0};
  int bezier{0};
  int bspline{0};
  int64_t scan_queries{0};
  int floors{0};
  /// One counter per ShapeKind, over every geometry of every world built.
  std::vector<int> shapes = std::vector<int>(6, 0);
  /// Smallest clearance-over-threshold the dense scan *measured* on a case the
  /// checker certified (pairs its broadphase skipped are provably clear but may
  /// be closer than this, so it is an upper bound on the true minimum).
  /// Reported, not asserted: it says how close the corpus gets to the
  /// certificate boundary, i.e. how much teeth the cross-check has.
  double tightest_certified_slack{std::numeric_limits<double>::infinity()};
};

/// Base options shared by every case.
Options FuzzOptions(double margin) {
  Options options;
  options.margin = margin;
  options.mode = SearchMode::kCertifyAll;
  options.emit_certificate = true;
  options.parallelism = Parallelism::None();
  // A coarser resolution floor than the 1e-9 default: a grazing pair still ends
  // kInconclusive, but after ~20 bisections rather than ~30, which keeps the
  // pathological cases of a 200-case corpus affordable. The node budget is the
  // second guard; a case that hits it is counted and skipped, never silently
  // accepted.
  options.min_interval = 1e-6;
  options.max_nodes = 300000;
  return options;
}

CertifiedContinuousCollisionChecker MakeChecker(
    std::shared_ptr<const RobotDiagram<double>> model, const Options& options) {
  CertifiedContinuousCollisionChecker::Params params;
  params.model = std::move(model);
  params.default_options = options;
  return CertifiedContinuousCollisionChecker(params);
}

GTEST_TEST(SoundnessFuzzTest, RandomWorldsAndTrajectories) {
  Tally tally;
  for (int case_index = 0; case_index < kNumCases; ++case_index) {
    const uint64_t seed = kBaseSeed + case_index;
    const WorldRecipe world = RandomWorld(seed);
    for (const LinkSpec& link : world.links) {
      ++tally.shapes[static_cast<int>(link.shape.kind)];
    }
    for (const ObstacleSpec& obstacle : world.obstacles) {
      ++tally.shapes[static_cast<int>(obstacle.shape.kind)];
    }
    if (world.floor) ++tally.floors;
    Rng rng(seed ^ 0x9e37'79b9'7f4a'7c15ull);
    const TrajectoryRecipe trajectory_recipe = RandomTrajectory(world, &rng);
    switch (trajectory_recipe.kind) {
      case TrajectoryKind::kPwl:
        ++tally.pwl;
        break;
      case TrajectoryKind::kBezier:
        ++tally.bezier;
        break;
      case TrajectoryKind::kBspline:
        ++tally.bspline;
        break;
    }

    std::shared_ptr<const RobotDiagram<double>> model = BuildWorld(world);
    const std::unique_ptr<Trajectory<double>> trajectory =
        BuildTrajectory(trajectory_recipe);

    // Both halves of test-plan T4's margin sweep — a bare-contact threshold and
    // a 1 cm clearance requirement — plus, on every fifth case, a *grazing*
    // margin: the trajectory's own minimum clearance, located by a coarse
    // pre-scan. Setting m_p exactly there makes the tangency unavoidable, which
    // is the only reliable way to exercise the kInconclusive branch (and its
    // cross-check) on random geometry. Without it the corpus would never
    // produce a grazing case, because a random trajectory is tangent to a
    // random obstacle with probability zero.
    double margin = (case_index % 2 == 0) ? 0.0 : 0.01;
    bool grazing = (case_index % 5) == 3;
    if (grazing) {
      const Options probe_options = FuzzOptions(0.0);
      const CertifiedContinuousCollisionChecker probe =
          MakeChecker(model, probe_options);
      DenseScanner probe_scanner(probe);
      const DenseScanner::Result probe_scan = probe_scanner.Scan(
          probe.Normalize(*trajectory, probe_options), kGrazeProbeSamples, 0.0);
      tally.scan_queries += probe_scanner.narrowphase_queries();
      if (probe_scan.min_slack > 0.01 && probe_scan.min_slack < 0.5) {
        margin = probe_scan.min_slack;
        ++tally.graze_cases;
      } else {
        grazing = false;
      }
    }

    SCOPED_TRACE("REPRO: case " + std::to_string(case_index) + ", margin " +
                 std::to_string(margin) + (grazing ? " (grazing)" : "") + "\n" +
                 world.Describe() + trajectory_recipe.Describe());

    const Options options = FuzzOptions(margin);
    const CertifiedContinuousCollisionChecker checker =
        MakeChecker(model, options);
    // This fuzz never sets a PaddingSpec, so m_p = margin for every pair; the
    // dense scan relies on that to compare against one number.
    for (const PairRecord& pair : checker.pairs()) {
      ASSERT_EQ(pair.threshold, margin);
    }

    const PiecewiseBezierPath path = checker.Normalize(*trajectory, options);
    const CertificationResult result =
        checker.CheckTrajectory(*trajectory, options);

    DenseScanner scanner(checker);

    switch (result.verdict) {
      case Verdict::kCertifiedFree: {
        ++tally.certified;
        ASSERT_TRUE(result.findings.empty());
        // (a) Dense sampling must find no configuration at or below the
        //     threshold. A single one would be a false certificate.
        const bool deep = (tally.certified % kDeepEvery) == 0;
        if (deep) ++tally.deep_scans;
        const DenseScanner::Result scan = scanner.Scan(
            path, deep ? kDeepDenseSamples : kDenseSamples, margin);
        tally.tightest_certified_slack =
            std::min(tally.tightest_certified_slack, scan.min_slack);
        EXPECT_GT(scan.min_slack, 0.0)
            << "CERTIFIED FREE but dense sampling ("
            << (deep ? kDeepDenseSamples : kDenseSamples)
            << " configurations) found clearance " << scan.min_slack
            << " m below the threshold at t = " << scan.worst_time
            << " for pair " << scan.worst_pair;
        // (b) The audit trail must replay independently.
        ASSERT_TRUE(result.certificate.has_value());
        EXPECT_TRUE(VerifyCertificate(checker, path, *result.certificate))
            << "CERTIFIED FREE but the emitted certificate does not verify";
        break;
      }
      case Verdict::kViolationFound:
        ++tally.violation;
        EXPECT_FALSE(result.findings.empty());
        break;
      case Verdict::kInconclusive:
        ++tally.inconclusive;
        EXPECT_FALSE(result.findings.empty());
        break;
      case Verdict::kBudgetExhausted:
        // The node budget is a safety valve, not an expected outcome; a run
        // that hits it reports the earliest node it left uncovered, and there
        // is nothing to cross-check because nothing was proved. The corpus-wide
        // bound on how often this may happen is asserted after the loop.
        ++tally.budget;
        EXPECT_FALSE(result.findings.empty())
            << "budget exhaustion must report the uncovered remainder";
        break;
    }

    // Findings are earliest-first, always.
    for (std::size_t i = 1; i < result.findings.size(); ++i) {
      EXPECT_LE(result.findings[i - 1].time, result.findings[i].time);
    }

    for (const Finding& finding : result.findings) {
      const int pair_index = scanner.FindPair(finding.pair);
      ASSERT_GE(pair_index, 0) << "finding names an unknown pair";
      const double threshold = checker.pairs()[pair_index].threshold;
      ASSERT_EQ(finding.q.size(), world.num_positions());

      if (finding.definite) {
        ++tally.definite_findings;
        // The witness is exactly on the trajectory ...
        EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(),
                  1e-9)
            << "a definite witness must be an on-trajectory configuration, "
               "never an interpolation artifact";
        // ... and re-measuring its pair there, from a context this run never
        // touched, must confirm the violation to within the oracle contract.
        const double phi = scanner.DistanceAt(finding.q, pair_index);
        EXPECT_LT(phi, threshold + kWorstTau)
            << "definite violation at t = " << finding.time
            << " re-measures at phi = " << phi << " against threshold "
            << threshold;
        EXPECT_NEAR(phi, finding.distance, 1e-9)
            << "the reported distance is not reproducible at the witness";
      } else if (result.verdict != Verdict::kBudgetExhausted) {
        // Every non-definite finding that is *not* a budget remainder is a
        // resolution-floor grazing record, whether the run as a whole ended
        // kInconclusive or kViolationFound (in kCertifyAll the sink's
        // inconclusive list is appended to the definite one, so a violating run
        // can carry grazing records too). All of them get the same audit; only
        // the synthesized "here is where the budget stopped us" finding is
        // exempt, because its clearance carries no claim.
        ++tally.inconclusive_findings;
        // Test plan T4: a grazing record must be backed by a clearance that
        // sits within 10·(τ_p + ε) of the threshold somewhere near the
        // reported time.
        const double tolerance = 10.0 * (kWorstTau + options.certificate_slack);
        const double window =
            0.01 * std::max(1e-12, path.end_time() - path.start_time());
        const double best =
            scanner.MinAbsSlackNear(path, pair_index, threshold, finding.time,
                                    window, /* samples = */ 400);
        EXPECT_LE(best, tolerance)
            << "INCONCLUSIVE at t = " << finding.time
            << " but the closest sampled clearance near it is " << best
            << " m from the threshold, far outside 10*(tau + eps) = "
            << tolerance;
      }
    }
    tally.scan_queries += scanner.narrowphase_queries();
  }

  std::cout << "\n[ T4 FUZZ SUMMARY ] cases = " << kNumCases
            << "  certified = " << tally.certified
            << "  violation = " << tally.violation
            << "  inconclusive = " << tally.inconclusive
            << "  budget = " << tally.budget << "\n"
            << "                     trajectories: PWL = " << tally.pwl
            << ", Bezier = " << tally.bezier << ", B-spline = " << tally.bspline
            << ";  grazing-margin cases = " << tally.graze_cases << "\n"
            << "                     deep (1e5-sample) scans = "
            << tally.deep_scans
            << "  definite findings = " << tally.definite_findings
            << "  inconclusive findings = " << tally.inconclusive_findings
            << "\n                     cross-check narrowphase queries = "
            << tally.scan_queries
            << ";  tightest measured clearance above a certified threshold = "
            << tally.tightest_certified_slack << " m\n"
            << "                     geometries:";
  for (int kind = 0; kind < 6; ++kind) {
    std::cout << " " << Name(static_cast<ShapeKind>(kind)) << "="
              << tally.shapes[kind];
  }
  std::cout << ", anchored HalfSpace floors = " << tally.floors << "\n\n";

  // The corpus has to actually exercise the outcomes it claims to cross-check;
  // a fuzz that certified everything (or violated everything) would pass every
  // assertion above while testing nothing.
  static_assert(
      kNumCases >= 150,
      "test plan T4 asks for >= 150 (world, trajectory) cases per CI run");
  EXPECT_GE(tally.certified, 40);
  EXPECT_GE(tally.violation, 20);
  EXPECT_GE(tally.inconclusive, 5)
      << "the grazing-margin cases should have produced kInconclusive verdicts";
  EXPECT_GE(tally.definite_findings, 20);
  EXPECT_GE(tally.inconclusive_findings, 5);
  // The node budget exists to bound a pathological case, not to be the usual
  // answer: if it starts firing often, the corpus has stopped cross-checking
  // anything and the numbers above would quietly stop meaning what they say.
  EXPECT_LE(tally.budget, kNumCases / 20);
  // All three trajectory families of trajectory normalization must be
  // represented.
  EXPECT_GE(tally.pwl, 20);
  EXPECT_GE(tally.bezier, 20);
  EXPECT_GE(tally.bspline, 20);
  // The dense scan must really be measuring distances, not skipping everything
  // through its broadphase.
  EXPECT_GT(tally.scan_queries, 100000);
  // Every supported geometry class must have appeared somewhere in the corpus,
  // including the analytic HalfSpace route: a fuzz that only ever built spheres
  // and boxes would leave the τ_p table's expensive rows (capsule, cylinder,
  // ellipsoid) and the Convex path untested end to end.
  for (int kind = 0; kind < 6; ++kind) {
    EXPECT_GT(tally.shapes[kind], 0)
        << "no " << Name(static_cast<ShapeKind>(kind))
        << " was generated anywhere in the corpus";
  }
  EXPECT_GT(tally.floors, 0);
}

}  // namespace
}  // namespace certified_ccd
}  // namespace planning
}  // namespace drake
