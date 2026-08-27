/// @file
/// T3 (the test plan): distance oracle accuracy, capability-probe
/// classification, the analytic halfspace fallback, Mesh-as-convex-hull
/// semantics, and the V-polytope ingestion round trip.
///
/// Every world is built programmatically with RobotDiagramBuilder and every
/// randomized case uses a fixed seed, so the suite is deterministic.

#include "drake/planning/continuous_collision/distance_oracle.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/continuous_collision/vpolytope_ingestion.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Convex;
using drake::geometry::Cylinder;
using drake::geometry::Ellipsoid;
using drake::geometry::GeometryId;
using drake::geometry::HalfSpace;
using drake::geometry::Mesh;
using drake::geometry::QueryObject;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::geometry::optimization::VPolytope;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::BodyIndex;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::systems::Context;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;

constexpr double kTau = 1e-6;
/// Exactness bar for the analytic halfspace fallback and for round trips that
/// must land on identical code paths.
constexpr double kExact = 1e-12;
/// Accuracy bar for Drake's native (partly iterative GJK) narrowphase.
constexpr double kNative = 1e-6;

// --------------------------------------------------------------------------
// World construction helpers.
// --------------------------------------------------------------------------

/// A built RobotDiagram plus a context, with convenience accessors.
class World {
 public:
  explicit World(std::unique_ptr<RobotDiagram<double>> diagram)
      : diagram_(std::move(diagram)),
        context_(diagram_->CreateDefaultContext()) {}

  const RobotDiagram<double>& diagram() const { return *diagram_; }
  const MultibodyPlant<double>& plant() const { return diagram_->plant(); }

  Context<double>& plant_context() {
    return diagram_->plant().GetMyMutableContextFromRoot(context_.get());
  }

  /// Re-evaluates the query output port; call after every pose change.
  const QueryObject<double>& query() {
    const auto& scene_graph = diagram_->scene_graph();
    return scene_graph.get_query_output_port().Eval<QueryObject<double>>(
        scene_graph.GetMyContextFromRoot(*context_));
  }

  void SetPose(const RigidBody<double>& body, const RigidTransformd& X_WB) {
    diagram_->plant().SetFreeBodyPose(&plant_context(), body, X_WB);
  }

  /// Randomizes every floating body's pose.
  void RandomizeAll(std::mt19937* rng, double range);

 private:
  std::unique_ptr<RobotDiagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
};

CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

/// Deterministic random pose: uniform translation in [-range, range]^3 and a
/// uniformly distributed orientation.
RigidTransformd RandomPose(std::mt19937* rng, double range) {
  std::uniform_real_distribution<double> uniform(-range, range);
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Quaterniond q(normal(*rng), normal(*rng), normal(*rng), normal(*rng));
  if (q.norm() < 1e-8) q = Eigen::Quaterniond::Identity();
  q.normalize();
  return RigidTransformd(RotationMatrixd(q),
                         Vector3d(uniform(*rng), uniform(*rng), uniform(*rng)));
}

void World::RandomizeAll(std::mt19937* rng, double range) {
  for (BodyIndex i(1); i < plant().num_bodies(); ++i) {
    const RigidBody<double>& body = plant().get_body(i);
    if (body.is_floating_base_body()) SetPose(body, RandomPose(rng, range));
  }
}

/// Adds a floating body carrying `shape` as its only collision geometry. The
/// default pose spreads bodies out so the capability probe's default-context
/// queries do not run on a pile of coincident geometry.
const RigidBody<double>& AddShapeBody(MultibodyPlant<double>* plant,
                                      const std::string& name,
                                      const Shape& shape,
                                      const Vector3d& default_p_WB) {
  const RigidBody<double>& body = plant->AddRigidBody(
      name, SpatialInertia<double>::SolidSphereWithMass(1.0, 0.1));
  plant->RegisterCollisionGeometry(body, RigidTransformd::Identity(), shape,
                                   name + "_geometry", Friction());
  plant->SetDefaultFloatingBaseBodyPose(body, RigidTransformd(default_p_WB));
  return body;
}

/// The single collision geometry registered on `body_name`.
GeometryId GeometryOf(const MultibodyPlant<double>& plant,
                      const std::string& body_name) {
  const auto& ids =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName(body_name));
  EXPECT_EQ(ids.size(), 1u);
  return ids.front();
}

/// Finds the probe record for the unordered pair {a, b}.
const PairRecord& FindPair(const DistanceOracle& oracle, GeometryId a,
                           GeometryId b) {
  for (const PairRecord& p : oracle.pairs()) {
    if ((p.id.a == a && p.id.b == b) || (p.id.a == b && p.id.b == a)) {
      return p;
    }
  }
  ADD_FAILURE() << "pair not found in the probe's snapshot";
  return oracle.pairs().front();
}

// --------------------------------------------------------------------------
// Independently derived ground truth.
// --------------------------------------------------------------------------

/// Distance from a point to a box, both in the box's frame; zero inside.
double PointBoxDistance(const Vector3d& p_B, const Vector3d& half) {
  return (p_B.cwiseAbs() - half).cwiseMax(0.0).norm();
}

// Hand-derived halfspace distances: phi = min over the shape of n^T(x - p0),
// written here from the textbook support functions rather than from the
// oracle's support-point machinery, so the two derivations stay independent.
// n is the outward unit normal (R_WH's third column) and p0 a boundary point.

double HalfSpaceSphere(const Vector3d& n, const Vector3d& p0,
                       const RigidTransformd& X_WC, double r) {
  return n.dot(X_WC.translation() - p0) - r;
}

double HalfSpaceBox(const Vector3d& n, const Vector3d& p0,
                    const RigidTransformd& X_WC, const Vector3d& half) {
  const Vector3d n_C = X_WC.rotation().matrix().transpose() * n;
  return n.dot(X_WC.translation() - p0) - half.dot(n_C.cwiseAbs());
}

double HalfSpaceCapsule(const Vector3d& n, const Vector3d& p0,
                        const RigidTransformd& X_WC, double r, double length) {
  const Vector3d axis = X_WC.rotation().matrix().col(2);
  return n.dot(X_WC.translation() - p0) - 0.5 * length * std::abs(n.dot(axis)) -
         r;
}

double HalfSpaceCylinder(const Vector3d& n, const Vector3d& p0,
                         const RigidTransformd& X_WC, double r, double length) {
  const Vector3d axis = X_WC.rotation().matrix().col(2);
  const double axial = n.dot(axis);
  return n.dot(X_WC.translation() - p0) - 0.5 * length * std::abs(axial) -
         r * (n - axial * axis).norm();
}

double HalfSpaceEllipsoid(const Vector3d& n, const Vector3d& p0,
                          const RigidTransformd& X_WC, const Vector3d& radii) {
  const Vector3d n_C = X_WC.rotation().matrix().transpose() * n;
  return n.dot(X_WC.translation() - p0) -
         Vector3d(radii.asDiagonal() * n_C).norm();
}

double HalfSpaceVertices(const Vector3d& n, const Vector3d& p0,
                         const RigidTransformd& X_WC, const Matrix3Xd& v_C) {
  const Vector3d n_C = X_WC.rotation().matrix().transpose() * n;
  return n.dot(X_WC.translation() - p0) + (n_C.transpose() * v_C).minCoeff();
}

// --------------------------------------------------------------------------
// On-disk meshes (written once per process into Drake's temp directory).
// --------------------------------------------------------------------------

/// The 8 corners of a box centered on its frame origin.
Matrix3Xd BoxCorners(const Vector3d& half) {
  Matrix3Xd v(3, 8);
  int col = 0;
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      for (const double sz : {-1.0, 1.0}) {
        v.col(col++) = Vector3d(sx * half.x(), sy * half.y(), sz * half.z());
      }
    }
  }
  return v;
}

const Vector3d& CubeHalf() {
  static const Vector3d half(0.15, 0.20, 0.25);
  return half;
}

/// Writes a closed triangulated box OBJ; returns its path.
const std::string& CubeObjPath() {
  static const std::string path = [] {
    const std::filesystem::path p =
        std::filesystem::path(drake::temp_directory()) / "ccd_cube.obj";
    std::ofstream out(p);
    const Vector3d& h = CubeHalf();
    const double xs[8] = {-1, 1, 1, -1, -1, 1, 1, -1};
    const double ys[8] = {-1, -1, 1, 1, -1, -1, 1, 1};
    const double zs[8] = {-1, -1, -1, -1, 1, 1, 1, 1};
    for (int i = 0; i < 8; ++i) {
      out << "v " << xs[i] * h.x() << " " << ys[i] * h.y() << " "
          << zs[i] * h.z() << "\n";
    }
    const int faces[12][3] = {{1, 4, 3}, {1, 3, 2}, {5, 6, 7}, {5, 7, 8},
                              {1, 2, 6}, {1, 6, 5}, {3, 4, 8}, {3, 8, 7},
                              {4, 1, 5}, {4, 5, 8}, {2, 3, 7}, {2, 7, 6}};
    for (const auto& f : faces) {
      out << "f " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }
    return p.string();
  }();
  return path;
}

/// The L-shaped prism's cross-section, counter-clockwise. The reflex vertex is
/// (1, 1); the convex hull closes the notch with the edge x + y = 3.
const std::vector<Eigen::Vector2d>& LProfile() {
  static const std::vector<Eigen::Vector2d> profile = {
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {1.0, 1.0}, {1.0, 2.0}, {0.0, 2.0}};
  return profile;
}

constexpr double kLHalfHeight = 0.5;

/// Writes a closed, genuinely non-convex L-prism OBJ; returns its path.
const std::string& LPrismObjPath() {
  static const std::string path = [] {
    const std::filesystem::path p =
        std::filesystem::path(drake::temp_directory()) / "ccd_l_prism.obj";
    std::ofstream out(p);
    const auto& profile = LProfile();
    const int n = static_cast<int>(profile.size());
    for (const double z : {-kLHalfHeight, kLHalfHeight}) {
      for (const auto& xy : profile) {
        out << "v " << xy.x() << " " << xy.y() << " " << z << "\n";
      }
    }
    // Side quads, split into triangles. OBJ indices are 1-based.
    for (int i = 0; i < n; ++i) {
      const int j = (i + 1) % n;
      out << "f " << i + 1 << " " << j + 1 << " " << j + 1 + n << "\n";
      out << "f " << i + 1 << " " << j + 1 + n << " " << i + 1 + n << "\n";
    }
    // Caps: a fan from vertex 0, which is valid for this particular profile.
    for (int i = 1; i + 1 < n; ++i) {
      out << "f 1 " << i + 2 << " " << i + 1 << "\n";
      out << "f " << 1 + n << " " << i + 1 + n << " " << i + 2 + n << "\n";
    }
    return p.string();
  }();
  return path;
}

// ==========================================================================
// Accuracy vs analytic ground truth (native route).
// ==========================================================================

GTEST_TEST(DistanceOracleAccuracy, SphereSphereMatchesAnalyticDistance) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  const double r_a = 0.13;
  const double r_b = 0.21;
  const auto& body_a =
      AddShapeBody(&plant, "sphere_a", Sphere(r_a), Vector3d(-1, 0, 0));
  const auto& body_b =
      AddShapeBody(&plant, "sphere_b", Sphere(r_b), Vector3d(1, 0, 0));
  World world(builder.Build());

  const DistanceOracle oracle(world.diagram(), kTau);
  ASSERT_EQ(oracle.pairs().size(), 1u);
  const PairRecord& pair = oracle.pairs().front();
  EXPECT_EQ(pair.route, DistanceRoute::kNative);

  std::mt19937 rng(20260826);
  int penetrating = 0;
  for (int trial = 0; trial < 250; ++trial) {
    const RigidTransformd X_WA = RandomPose(&rng, 0.4);
    const RigidTransformd X_WB = RandomPose(&rng, 0.4);
    world.SetPose(body_a, X_WA);
    world.SetPose(body_b, X_WB);

    Vector3d p_a_W;
    Vector3d p_b_W;
    const double phi =
        oracle.SignedDistance(world.query(), pair, &p_a_W, &p_b_W);
    // Exact for two spheres on both branches.
    const double expected =
        (X_WB.translation() - X_WA.translation()).norm() - r_a - r_b;
    EXPECT_NEAR(phi, expected, kNative) << "trial " << trial;
    if (phi < 0.0) ++penetrating;
    if (phi > 1e-9) {
      EXPECT_NEAR((p_a_W - p_b_W).norm(), phi, kNative) << "trial " << trial;
    }
  }
  EXPECT_GT(penetrating, 0) << "the sweep never exercised the penetrating "
                               "branch";
}

GTEST_TEST(DistanceOracleAccuracy, SphereBoxMatchesAnalyticDistance) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  const double radius = 0.1;
  const Vector3d half(0.15, 0.2, 0.25);
  const auto& sphere_body =
      AddShapeBody(&plant, "sphere", Sphere(radius), Vector3d(-1, 0, 0));
  const auto& box_body =
      AddShapeBody(&plant, "box", Box(2 * half.x(), 2 * half.y(), 2 * half.z()),
                   Vector3d(1, 0, 0));
  World world(builder.Build());

  const DistanceOracle oracle(world.diagram(), kTau);
  ASSERT_EQ(oracle.pairs().size(), 1u);
  const PairRecord& pair = oracle.pairs().front();

  std::mt19937 rng(881);
  int separated = 0;
  int penetrating = 0;
  for (int trial = 0; trial < 250; ++trial) {
    const RigidTransformd X_WS = RandomPose(&rng, 0.5);
    const RigidTransformd X_WB = RandomPose(&rng, 0.5);
    world.SetPose(sphere_body, X_WS);
    world.SetPose(box_body, X_WB);

    const Vector3d p_B = X_WB.inverse() * X_WS.translation();
    const double center_distance = PointBoxDistance(p_B, half);
    if (center_distance <= 0.0) continue;  // center inside the box
    // For a sphere whose center lies outside a convex body, the signed
    // distance is exactly dist(center, body) - radius on both branches: the
    // sublevel sets of dist(., body) are the Minkowski sums body (+) ball.
    const double expected = center_distance - radius;

    Vector3d p_a_W;
    Vector3d p_b_W;
    const double phi =
        oracle.SignedDistance(world.query(), pair, &p_a_W, &p_b_W);
    EXPECT_NEAR(phi, expected, kNative) << "trial " << trial;
    if (phi > 1e-9) {
      ++separated;
      EXPECT_NEAR((p_a_W - p_b_W).norm(), phi, kNative) << "trial " << trial;
    } else {
      ++penetrating;
    }
  }
  EXPECT_GT(separated, 0);
  EXPECT_GT(penetrating, 0);
}

// ==========================================================================
// Analytic halfspace fallback: exact against hand-derived formulas.
// ==========================================================================

/// One world holding a halfspace plus one geometry of every partner class,
/// all on floating bodies so both sides can be posed arbitrarily.
class HalfSpaceFallbackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    RobotDiagramBuilder<double> builder(0.0);
    MultibodyPlant<double>& plant = builder.plant();
    AddShapeBody(&plant, "halfspace", HalfSpace(), Vector3d(0, 0, -3));
    AddShapeBody(&plant, "sphere", Sphere(kRadius), Vector3d(-3, 0, 0));
    AddShapeBody(&plant, "box",
                 Box(2 * kHalf.x(), 2 * kHalf.y(), 2 * kHalf.z()),
                 Vector3d(-1, 0, 0));
    AddShapeBody(&plant, "capsule", Capsule(kRadius, kLength),
                 Vector3d(1, 0, 0));
    AddShapeBody(&plant, "cylinder", Cylinder(kRadius, kLength),
                 Vector3d(3, 0, 0));
    AddShapeBody(&plant, "ellipsoid",
                 Ellipsoid(kRadii.x(), kRadii.y(), kRadii.z()),
                 Vector3d(0, 3, 0));
    AddShapeBody(&plant, "tetra", Convex(TetraVertices(), "tetra"),
                 Vector3d(0, -3, 0));
    world_ = std::make_unique<World>(builder.Build());
    oracle_ = std::make_unique<DistanceOracle>(world_->diagram(), kTau);
    halfspace_id_ = GeometryOf(world_->plant(), "halfspace");
  }

  /// A deliberately asymmetric tetrahedron: its hull vertices are exactly the
  /// four input points, so the reference minimum can be written down.
  static Matrix3Xd TetraVertices() {
    Matrix3Xd v(3, 4);
    v.col(0) = Vector3d(0.0, 0.0, 0.0);
    v.col(1) = Vector3d(0.31, 0.02, -0.05);
    v.col(2) = Vector3d(0.04, 0.27, 0.03);
    v.col(3) = Vector3d(-0.06, 0.05, 0.23);
    return v;
  }

  const RigidBody<double>& Body(const std::string& name) const {
    return world_->plant().GetBodyByName(name);
  }

  static constexpr double kRadius = 0.11;
  static constexpr double kLength = 0.34;
  static inline const Vector3d kHalf{0.15, 0.2, 0.25};
  static inline const Vector3d kRadii{0.12, 0.19, 0.07};

  std::unique_ptr<World> world_;
  std::unique_ptr<DistanceOracle> oracle_;
  GeometryId halfspace_id_;
};

TEST_F(HalfSpaceFallbackTest, EveryPartnerMatchesHandDerivedFormulaExactly) {
  using Reference = std::function<double(const Vector3d&, const Vector3d&,
                                         const RigidTransformd&)>;
  struct Partner {
    std::string name;
    Reference reference;
  };
  const std::vector<Partner> partners = {
      {"sphere",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceSphere(n, p0, X, kRadius);
       }},
      {"box",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceBox(n, p0, X, kHalf);
       }},
      {"capsule",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceCapsule(n, p0, X, kRadius, kLength);
       }},
      {"cylinder",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceCylinder(n, p0, X, kRadius, kLength);
       }},
      {"ellipsoid",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceEllipsoid(n, p0, X, kRadii);
       }},
      {"tetra",
       [](const Vector3d& n, const Vector3d& p0, const RigidTransformd& X) {
         return HalfSpaceVertices(n, p0, X, TetraVertices());
       }},
  };

  std::mt19937 rng(4242);
  int positive = 0;
  int negative = 0;
  for (int trial = 0; trial < 200; ++trial) {
    const RigidTransformd X_WH = RandomPose(&rng, 0.3);
    world_->SetPose(Body("halfspace"), X_WH);
    std::vector<RigidTransformd> poses;
    for (const Partner& p : partners) {
      poses.push_back(RandomPose(&rng, 0.4));
      world_->SetPose(Body(p.name), poses.back());
    }
    const QueryObject<double>& query = world_->query();
    const Vector3d n_W = X_WH.rotation().matrix().col(2);
    const Vector3d p0_W = X_WH.translation();

    for (size_t i = 0; i < partners.size(); ++i) {
      const PairRecord& pair =
          FindPair(*oracle_, halfspace_id_,
                   GeometryOf(world_->plant(), partners[i].name));
      ASSERT_NE(pair.route, DistanceRoute::kNative) << partners[i].name;

      Vector3d p_a_W;
      Vector3d p_b_W;
      const double phi = oracle_->SignedDistance(query, pair, &p_a_W, &p_b_W);
      const double expected = partners[i].reference(n_W, p0_W, poses[i]);
      EXPECT_NEAR(phi, expected, kExact)
          << partners[i].name << ", trial " << trial;
      // Witnesses: separated by exactly |phi|, with the halfspace-side witness
      // on the boundary plane.
      const Vector3d& on_plane = (pair.id.a == halfspace_id_) ? p_a_W : p_b_W;
      EXPECT_NEAR(n_W.dot(on_plane - p0_W), 0.0, kExact);
      EXPECT_NEAR((p_a_W - p_b_W).norm(), std::abs(phi), kExact);
      if (phi > 0.0) {
        ++positive;
      } else {
        ++negative;
      }
    }
  }
  EXPECT_GT(positive, 0);
  EXPECT_GT(negative, 0) << "no penetrating halfspace cases were exercised";
}

TEST_F(HalfSpaceFallbackTest, AxisParallelCylinderDirectionIsHandled) {
  // Degenerate support direction: the plane normal is parallel to the cylinder
  // axis, so every rim point ties. phi must still be exact.
  world_->SetPose(Body("halfspace"), RigidTransformd(Vector3d(0, 0, -0.5)));
  world_->SetPose(Body("cylinder"), RigidTransformd(Vector3d(0.0, 0.0, 0.4)));
  const PairRecord& pair = FindPair(*oracle_, halfspace_id_,
                                    GeometryOf(world_->plant(), "cylinder"));
  Vector3d p_a_W;
  Vector3d p_b_W;
  const double phi =
      oracle_->SignedDistance(world_->query(), pair, &p_a_W, &p_b_W);
  EXPECT_NEAR(phi, 0.4 + 0.5 - kLength / 2, kExact);
  EXPECT_NEAR((p_a_W - p_b_W).norm(), std::abs(phi), kExact);
}

TEST_F(HalfSpaceFallbackTest, ReportNamesEveryCombinationAndRoute) {
  const std::string report = oracle_->support_report();
  SCOPED_TRACE(report);
  // Rows are ordered by shape class, so HalfSpace is always the second name.
  EXPECT_NE(report.find("Sphere-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("Box-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("Capsule-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("Cylinder-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("Ellipsoid-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("Convex-HalfSpace"), std::string::npos);
  EXPECT_NE(report.find("halfspace analytic support-function fallback"),
            std::string::npos);
  EXPECT_NE(report.find("native (ComputeSignedDistancePairClosestPoints"),
            std::string::npos);
}

// ==========================================================================
// Capability probe: classification snapshot and refusals.
// ==========================================================================

/// A world with one geometry of every supported shape class.
class AllShapesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    RobotDiagramBuilder<double> builder(0.0);
    MultibodyPlant<double>& plant = builder.plant();
    AddShapeBody(&plant, "sphere", Sphere(0.1), Vector3d(-1.5, 0, 1));
    AddShapeBody(&plant, "box", Box(0.2, 0.3, 0.4), Vector3d(-0.5, 0, 1));
    AddShapeBody(&plant, "capsule", Capsule(0.08, 0.3), Vector3d(0.5, 0, 1));
    AddShapeBody(&plant, "cylinder", Cylinder(0.09, 0.25), Vector3d(1.5, 0, 1));
    AddShapeBody(&plant, "ellipsoid", Ellipsoid(0.12, 0.09, 0.07),
                 Vector3d(-1.5, 1.5, 1));
    AddShapeBody(&plant, "convex", Convex(BoxCorners(CubeHalf()), "convex"),
                 Vector3d(-0.5, 1.5, 1));
    AddShapeBody(&plant, "mesh", Mesh(CubeObjPath()), Vector3d(0.5, 1.5, 1));
    // The halfspace is anchored on the world body: z <= 0 is solid.
    halfspace_id_ = plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransformd::Identity(), HalfSpace(),
        "ground_geometry", Friction());
    world_ = std::make_unique<World>(builder.Build());
    oracle_ = std::make_unique<DistanceOracle>(world_->diagram(), kTau);
  }

  static constexpr int kDynamicBodies = 7;
  static constexpr int kNativePairs = kDynamicBodies * (kDynamicBodies - 1) / 2;

  std::unique_ptr<World> world_;
  std::unique_ptr<DistanceOracle> oracle_;
  GeometryId halfspace_id_;
};

TEST_F(AllShapesTest, ProbeClassifiesEveryPairSnapshot) {
  // 7 dynamic geometries pairwise, plus each against the anchored halfspace.
  ASSERT_EQ(static_cast<int>(oracle_->pairs().size()),
            kNativePairs + kDynamicBodies);

  int halfspace_pairs = 0;
  int native_pairs = 0;
  for (const PairRecord& pair : oracle_->pairs()) {
    if (pair.id.a == halfspace_id_) {
      EXPECT_EQ(pair.route, DistanceRoute::kHalfSpaceA);
      ++halfspace_pairs;
    } else if (pair.id.b == halfspace_id_) {
      EXPECT_EQ(pair.route, DistanceRoute::kHalfSpaceB);
      ++halfspace_pairs;
    } else {
      EXPECT_EQ(pair.route, DistanceRoute::kNative);
      ++native_pairs;
    }
    // Every record carries the bodies its geometries hang from.
    EXPECT_NE(pair.id.body_a, pair.id.body_b);
    EXPECT_EQ(pair.threshold, 0.0) << "the facade owns thresholds";
  }
  EXPECT_EQ(halfspace_pairs, kDynamicBodies);
  EXPECT_EQ(native_pairs, kNativePairs);
}

TEST_F(AllShapesTest, ReportAnnouncesMeshAsConvexHull) {
  const std::string report = oracle_->support_report();
  SCOPED_TRACE(report);
  EXPECT_NE(report.find("Mesh mesh_geometry: certified as its convex hull"),
            std::string::npos);
  EXPECT_NE(report.find("distinct shape-type combination(s)"),
            std::string::npos);
  // 8 distinct classes, each present once: 8*7/2 = 28 combinations.
  EXPECT_NE(report.find("28 distinct shape-type combination(s)"),
            std::string::npos);
}

TEST_F(AllShapesTest, WitnessPointsAreConsistentForSeparatedNativePairs) {
  std::mt19937 rng(31337);
  int checked = 0;
  for (int trial = 0; trial < 25; ++trial) {
    world_->RandomizeAll(&rng, 0.6);
    const QueryObject<double>& query = world_->query();
    for (const PairRecord& pair : oracle_->pairs()) {
      if (pair.route != DistanceRoute::kNative) continue;
      Vector3d p_a_W;
      Vector3d p_b_W;
      const double phi = oracle_->SignedDistance(query, pair, &p_a_W, &p_b_W);
      if (phi <= 1e-6) continue;
      // Drake's own worst-case table for these combinations tops out at 5e-5.
      EXPECT_NEAR((p_a_W - p_b_W).norm(), phi, 1e-4) << "trial " << trial;
      ++checked;
    }
  }
  EXPECT_GT(checked, 100);
}

TEST_F(AllShapesTest, IdOrderingIsSymmetricForNativePairs) {
  std::mt19937 rng(777);
  world_->RandomizeAll(&rng, 0.5);
  const QueryObject<double>& query = world_->query();

  int checked = 0;
  for (const PairRecord& pair : oracle_->pairs()) {
    if (pair.route != DistanceRoute::kNative) continue;
    PairRecord swapped = pair;
    std::swap(swapped.id.a, swapped.id.b);
    std::swap(swapped.id.body_a, swapped.id.body_b);

    Vector3d a1;
    Vector3d b1;
    Vector3d a2;
    Vector3d b2;
    const double phi = oracle_->SignedDistance(query, pair, &a1, &b1);
    const double phi_swapped =
        oracle_->SignedDistance(query, swapped, &a2, &b2);
    EXPECT_EQ(phi, phi_swapped);
    EXPECT_EQ(a1, b2);
    EXPECT_EQ(b1, a2);
    ++checked;
  }
  EXPECT_EQ(checked, kNativePairs);
}

TEST_F(AllShapesTest, IdOrderingIsSymmetricForHalfSpacePairs) {
  std::mt19937 rng(778);
  world_->RandomizeAll(&rng, 0.5);
  const QueryObject<double>& query = world_->query();

  int checked = 0;
  for (const PairRecord& pair : oracle_->pairs()) {
    if (pair.route == DistanceRoute::kNative) continue;
    PairRecord swapped = pair;
    std::swap(swapped.id.a, swapped.id.b);
    std::swap(swapped.id.body_a, swapped.id.body_b);
    swapped.route = (pair.route == DistanceRoute::kHalfSpaceA)
                        ? DistanceRoute::kHalfSpaceB
                        : DistanceRoute::kHalfSpaceA;

    Vector3d a1;
    Vector3d b1;
    Vector3d a2;
    Vector3d b2;
    const double phi = oracle_->SignedDistance(query, pair, &a1, &b1);
    const double phi_swapped =
        oracle_->SignedDistance(query, swapped, &a2, &b2);
    EXPECT_EQ(phi, phi_swapped);
    EXPECT_EQ(a1, b2);
    EXPECT_EQ(b1, a2);
    ++checked;
  }
  EXPECT_EQ(checked, kDynamicBodies);
}

/// Records, for the certifier and benchmark authors, which (shape, shape)
/// combinations Drake's native narrowphase actually supports on the pinned
/// build. The oracle routes every halfspace pair through the analytic fallback
/// precisely because of the rows this test prints.
TEST_F(AllShapesTest, NativeSupportTableSnapshot) {
  const auto& inspector = world_->diagram().scene_graph().model_inspector();
  const QueryObject<double>& query = world_->query();
  std::string table =
      "Native ComputeSignedDistancePairClosestPoints support:\n";
  int supported = 0;
  int threw = 0;
  for (const PairRecord& pair : oracle_->pairs()) {
    const std::string combo =
        std::string(inspector.GetShape(pair.id.a).type_name()) + "-" +
        std::string(inspector.GetShape(pair.id.b).type_name());
    try {
      query.ComputeSignedDistancePairClosestPoints(pair.id.a, pair.id.b);
      table += "  " + combo + ": supported\n";
      ++supported;
    } catch (const std::exception&) {
      table += "  " + combo + ": THROWS\n";
      ++threw;
    }
  }
  std::cout << table << std::flush;
  EXPECT_EQ(supported + threw, static_cast<int>(oracle_->pairs().size()));
  // All non-halfspace combinations must work natively -- exactly what the
  // capability probe asserted at construction.
  EXPECT_GE(supported, kNativePairs);
}

GTEST_TEST(DistanceOracleProbe, HalfSpaceHalfSpacePairThrowsAtConstruction) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransformd::Identity(), HalfSpace(),
                                  "ground_geometry", Friction());
  AddShapeBody(&plant, "ceiling", HalfSpace(), Vector3d(0, 0, 2));
  World world(builder.Build());

  try {
    const DistanceOracle oracle(world.diagram(), kTau);
    ADD_FAILURE() << "expected the capability probe to refuse the pair";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    SCOPED_TRACE(what);
    EXPECT_NE(what.find("HalfSpace"), std::string::npos);
    EXPECT_NE(what.find("ground_geometry"), std::string::npos);
    EXPECT_NE(what.find("ceiling_geometry"), std::string::npos);
  }
}

GTEST_TEST(DistanceOracleProbe, ProbeSnapshotIsStableAcrossConstructions) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  AddShapeBody(&plant, "sphere", Sphere(0.1), Vector3d(-1, 0, 0));
  AddShapeBody(&plant, "box", Box(0.2, 0.2, 0.2), Vector3d(1, 0, 0));
  World world(builder.Build());

  const DistanceOracle first(world.diagram(), kTau);
  const DistanceOracle second(world.diagram(), kTau);
  EXPECT_EQ(first.support_report(), second.support_report());
  EXPECT_EQ(first.tolerance(), kTau);
  ASSERT_EQ(first.pairs().size(), second.pairs().size());
  for (size_t i = 0; i < first.pairs().size(); ++i) {
    EXPECT_EQ(first.pairs()[i].id.a, second.pairs()[i].id.a);
    EXPECT_EQ(first.pairs()[i].id.b, second.pairs()[i].id.b);
  }
}

GTEST_TEST(DistanceOracleProbe, DeformableGeometryIsRefusedByName) {
  // Deformables need a discrete plant.
  RobotDiagramBuilder<double> builder(0.01);
  MultibodyPlant<double>& plant = builder.plant();
  AddShapeBody(&plant, "sphere", Sphere(0.1), Vector3d(0, 0, 1));

  auto instance = std::make_unique<drake::geometry::GeometryInstance>(
      RigidTransformd(Vector3d(0, 0, 0.4)), std::make_unique<Sphere>(0.05),
      "squishy");
  drake::geometry::ProximityProperties props;
  drake::geometry::AddContactMaterial(std::nullopt, std::nullopt, Friction(),
                                      &props);
  instance->set_proximity_properties(std::move(props));
  plant.mutable_deformable_model().RegisterDeformableBody(
      std::move(instance),
      drake::multibody::fem::DeformableBodyConfig<double>{}, 0.05);
  World world(builder.Build());

  try {
    const DistanceOracle oracle(world.diagram(), kTau);
    ADD_FAILURE() << "expected the probe to refuse the deformable geometry";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    SCOPED_TRACE(what);
    EXPECT_NE(what.find("deformable"), std::string::npos);
    EXPECT_NE(what.find("squishy"), std::string::npos);
  }
}

GTEST_TEST(DistanceOracleProbe, EmptyWorldProbesCleanly) {
  RobotDiagramBuilder<double> builder(0.0);
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);
  EXPECT_TRUE(oracle.pairs().empty());
  EXPECT_NE(oracle.support_report().find("0 unfiltered pair(s)"),
            std::string::npos);
}

// ==========================================================================
// Mesh is certified as its convex hull.
// ==========================================================================

GTEST_TEST(DistanceOracleMesh, MeshDistanceEqualsConvexHullDistance) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  // The same cube: once as a Mesh (Drake silently hulls it), once as a Convex
  // built from that hull's vertices.
  const auto& mesh_body =
      AddShapeBody(&plant, "mesh", Mesh(CubeObjPath()), Vector3d(-1, 0, 0));
  const auto& convex_body =
      AddShapeBody(&plant, "convex", Convex(BoxCorners(CubeHalf()), "cube"),
                   Vector3d(1, 0, 0));
  const auto& probe_body =
      AddShapeBody(&plant, "probe", Sphere(0.07), Vector3d(0, 2, 0));
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);

  const GeometryId mesh_id = GeometryOf(world.plant(), "mesh");
  const GeometryId convex_id = GeometryOf(world.plant(), "convex");
  const GeometryId probe_id = GeometryOf(world.plant(), "probe");
  const PairRecord& mesh_pair = FindPair(oracle, mesh_id, probe_id);
  const PairRecord& convex_pair = FindPair(oracle, convex_id, probe_id);

  std::mt19937 rng(9091);
  int separated = 0;
  int penetrating = 0;
  for (int trial = 0; trial < 100; ++trial) {
    // Pose both cubes identically, then probe with a sphere.
    const RigidTransformd X_WCube = RandomPose(&rng, 0.3);
    world.SetPose(mesh_body, X_WCube);
    world.SetPose(convex_body, X_WCube);
    world.SetPose(probe_body, RandomPose(&rng, 0.5));

    const QueryObject<double>& query = world.query();
    const double phi_mesh = oracle.SignedDistance(query, mesh_pair);
    const double phi_convex = oracle.SignedDistance(query, convex_pair);
    EXPECT_NEAR(phi_mesh, phi_convex, kExact) << "trial " << trial;
    if (phi_mesh > 0) {
      ++separated;
    } else {
      ++penetrating;
    }
  }
  EXPECT_GT(separated, 0);
  EXPECT_GT(penetrating, 0);
}

/// Documents the semantics loudly (the geometry-support scope; the risk
/// register): a *non-convex* Mesh is measured as its convex hull, so a probe
/// sitting in the L's concave notch -- genuinely 0.35 m clear of the solid --
/// is reported as penetrating.
GTEST_TEST(DistanceOracleMesh,
           NonconvexMeshIsMeasuredAsItsConvexHullNotItsSurface) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  const auto& l_mesh_body =
      AddShapeBody(&plant, "l_mesh", Mesh(LPrismObjPath()), Vector3d(0, 0, 0));
  const auto& l_convex_body = AddShapeBody(
      &plant, "l_convex", Convex(LPrismObjPath()), Vector3d(0, 0, 0));
  const double probe_radius = 0.05;
  const auto& probe_body =
      AddShapeBody(&plant, "probe", Sphere(probe_radius), Vector3d(5, 5, 5));
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);

  const GeometryId mesh_id = GeometryOf(world.plant(), "l_mesh");
  const GeometryId convex_id = GeometryOf(world.plant(), "l_convex");
  const GeometryId probe_id = GeometryOf(world.plant(), "probe");

  // (1.4, 1.4) sits in the notch: outside the L (the nearest solid points are
  // (1.4, 1.0) and (1.0, 1.4), so the true clearance is 0.4 - r = 0.35), but
  // inside the hull, whose closing edge is x + y = 3.
  const double true_surface_clearance = 0.4 - probe_radius;
  ASSERT_GT(true_surface_clearance, 0.0);

  world.SetPose(l_mesh_body, RigidTransformd::Identity());
  world.SetPose(l_convex_body, RigidTransformd::Identity());
  world.SetPose(probe_body, RigidTransformd(Vector3d(1.4, 1.4, 0.0)));

  {
    const QueryObject<double>& query = world.query();
    const double phi_mesh =
        oracle.SignedDistance(query, FindPair(oracle, mesh_id, probe_id));
    const double phi_convex =
        oracle.SignedDistance(query, FindPair(oracle, convex_id, probe_id));
    EXPECT_NEAR(phi_mesh, phi_convex, kExact);
    EXPECT_LT(phi_mesh, 0.0)
        << "the L-mesh must measure as its convex hull, which swallows the "
           "notch; got phi = "
        << phi_mesh;
  }

  // Outside the hull too => both agree and both are positive.
  world.SetPose(probe_body, RigidTransformd(Vector3d(1.9, 1.9, 0.0)));
  {
    const QueryObject<double>& query = world.query();
    const double phi_mesh =
        oracle.SignedDistance(query, FindPair(oracle, mesh_id, probe_id));
    const double phi_convex =
        oracle.SignedDistance(query, FindPair(oracle, convex_id, probe_id));
    EXPECT_GT(phi_mesh, 0.0);
    EXPECT_NEAR(phi_mesh, phi_convex, kExact);
  }
}

GTEST_TEST(DistanceOracleMesh, HalfSpaceFallbackAgainstMeshUsesTheSameHull) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  const auto& mesh_body =
      AddShapeBody(&plant, "mesh", Mesh(CubeObjPath()), Vector3d(0, 0, 1));
  const auto& convex_body =
      AddShapeBody(&plant, "convex", Convex(CubeObjPath()), Vector3d(0, 3, 1));
  const GeometryId ground = plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransformd::Identity(), HalfSpace(),
      "ground_geometry", Friction());
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);

  const GeometryId mesh_id = GeometryOf(world.plant(), "mesh");
  const GeometryId convex_id = GeometryOf(world.plant(), "convex");
  const Matrix3Xd corners = BoxCorners(CubeHalf());

  std::mt19937 rng(5150);
  for (int trial = 0; trial < 100; ++trial) {
    const RigidTransformd X_W = RandomPose(&rng, 0.3);
    world.SetPose(mesh_body, X_W);
    world.SetPose(convex_body, X_W);
    const QueryObject<double>& query = world.query();
    const double phi_mesh =
        oracle.SignedDistance(query, FindPair(oracle, ground, mesh_id));
    const double phi_convex =
        oracle.SignedDistance(query, FindPair(oracle, ground, convex_id));
    EXPECT_NEAR(phi_mesh, phi_convex, kExact) << "trial " << trial;
    // The ground plane is z = 0 with the solid below, so the reference is the
    // lowest transformed cube corner.
    double lowest = std::numeric_limits<double>::infinity();
    for (int i = 0; i < corners.cols(); ++i) {
      lowest = std::min(lowest, (X_W * Vector3d(corners.col(i))).z());
    }
    EXPECT_NEAR(phi_mesh, lowest, kExact) << "trial " << trial;
  }
}

// ==========================================================================
// V-polytope ingestion round trip.
// ==========================================================================

/// A deliberately lopsided polytope.
Matrix3Xd PolytopeVertices() {
  Matrix3Xd v(3, 6);
  v.col(0) = Vector3d(0.00, 0.00, 0.00);
  v.col(1) = Vector3d(0.28, 0.03, 0.01);
  v.col(2) = Vector3d(0.05, 0.31, -0.02);
  v.col(3) = Vector3d(-0.04, 0.06, 0.24);
  v.col(4) = Vector3d(0.22, 0.25, 0.19);
  v.col(5) = Vector3d(0.10, -0.18, 0.11);
  return v;
}

/// The same polytope with interior points that add nothing to the hull.
Matrix3Xd RedundantPolytopeVertices() {
  const Matrix3Xd v = PolytopeVertices();
  Matrix3Xd r(3, v.cols() + 3);
  r.leftCols(v.cols()) = v;
  r.col(v.cols() + 0) = v.rowwise().mean();
  r.col(v.cols() + 1) = 0.5 * (v.col(0) + v.col(4));
  r.col(v.cols() + 2) = 0.25 * (v.col(1) + v.col(2) + v.col(3) + v.col(5));
  return r;
}

GTEST_TEST(VPolytopeIngestion, RoundTripMatchesDirectConvexRegistration) {
  const Matrix3Xd vertices = PolytopeVertices();
  const VPolytope vpoly(vertices);
  const VPolytope redundant_vpoly{RedundantPolytopeVertices()};

  const RigidTransformd X_WG(RotationMatrixd(Eigen::AngleAxisd(
                                 0.7, Vector3d(0.3, -0.5, 0.8).normalized())),
                             Vector3d(0.2, -0.1, 0.35));

  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  const GeometryId ingested =
      AddVPolytopeObstacle(&plant, vpoly, X_WG, "ingested_vpolytope");
  const GeometryId ingested_redundant =
      AddVPolytopeObstacle(&plant, redundant_vpoly, X_WG, "ingested_redundant");
  const GeometryId direct = plant.RegisterCollisionGeometry(
      plant.world_body(), X_WG, Convex(vertices, "direct_convex"),
      "direct_convex", Friction());
  const auto& probe_body =
      AddShapeBody(&plant, "probe", Sphere(0.06), Vector3d(2, 2, 2));
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);
  const GeometryId probe_id = GeometryOf(world.plant(), "probe");

  // Three anchored obstacles against one dynamic probe. Anchored-anchored
  // pairs are filtered by SceneGraph, so exactly three pairs survive.
  ASSERT_EQ(oracle.pairs().size(), 3u);

  const PairRecord& p_ingested = FindPair(oracle, ingested, probe_id);
  const PairRecord& p_redundant =
      FindPair(oracle, ingested_redundant, probe_id);
  const PairRecord& p_direct = FindPair(oracle, direct, probe_id);

  std::mt19937 rng(2718);
  int separated = 0;
  int penetrating = 0;
  for (int trial = 0; trial < 120; ++trial) {
    world.SetPose(probe_body, RandomPose(&rng, 0.5));
    const QueryObject<double>& query = world.query();

    Vector3d a_i;
    Vector3d b_i;
    Vector3d a_d;
    Vector3d b_d;
    const double phi_ingested =
        oracle.SignedDistance(query, p_ingested, &a_i, &b_i);
    const double phi_direct =
        oracle.SignedDistance(query, p_direct, &a_d, &b_d);
    const double phi_redundant = oracle.SignedDistance(query, p_redundant);

    EXPECT_NEAR(phi_ingested, phi_direct, kExact) << "trial " << trial;
    EXPECT_NEAR(phi_redundant, phi_direct, kExact) << "trial " << trial;
    if (phi_direct > 1e-9) {
      ++separated;
      EXPECT_LT((a_i - a_d).norm(), kExact) << "trial " << trial;
      EXPECT_LT((b_i - b_d).norm(), kExact) << "trial " << trial;
    } else {
      ++penetrating;
    }
  }
  EXPECT_GT(separated, 0);
  EXPECT_GT(penetrating, 0);
}

GTEST_TEST(VPolytopeIngestion, RoundTripAlsoHoldsOnTheHalfSpaceFallbackRoute) {
  const Matrix3Xd vertices = PolytopeVertices();
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  // Both copies ride floating bodies so the anchored halfspace can see them.
  const VPolytope vpoly(vertices);
  const auto& ingested_body = AddShapeBody(
      &plant, "ingested", vpoly.ToShapeConvex("ingested"), Vector3d(0, 0, 1));
  const auto& direct_body = AddShapeBody(
      &plant, "direct", Convex(vertices, "direct"), Vector3d(0, 2, 1));
  const GeometryId ground = plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransformd::Identity(), HalfSpace(),
      "ground_geometry", Friction());
  World world(builder.Build());
  const DistanceOracle oracle(world.diagram(), kTau);

  const GeometryId ingested_id = GeometryOf(world.plant(), "ingested");
  const GeometryId direct_id = GeometryOf(world.plant(), "direct");

  std::mt19937 rng(1618);
  for (int trial = 0; trial < 100; ++trial) {
    const RigidTransformd X_W = RandomPose(&rng, 0.3);
    world.SetPose(ingested_body, X_W);
    world.SetPose(direct_body, X_W);
    const QueryObject<double>& query = world.query();
    const double phi_ingested =
        oracle.SignedDistance(query, FindPair(oracle, ground, ingested_id));
    const double phi_direct =
        oracle.SignedDistance(query, FindPair(oracle, ground, direct_id));
    EXPECT_NEAR(phi_ingested, phi_direct, kExact) << "trial " << trial;
    // Reference: the lowest transformed vertex, since the ground is z = 0.
    double lowest = std::numeric_limits<double>::infinity();
    for (int i = 0; i < vertices.cols(); ++i) {
      lowest = std::min(lowest, (X_W * Vector3d(vertices.col(i))).z());
    }
    EXPECT_NEAR(phi_direct, lowest, kExact) << "trial " << trial;
  }
}

GTEST_TEST(VPolytopeIngestion, RejectsBadArguments) {
  const VPolytope vpoly(PolytopeVertices());
  EXPECT_THROW(
      AddVPolytopeObstacle(nullptr, vpoly, RigidTransformd::Identity(), "x"),
      std::exception);

  {  // 2-D polytope.
    RobotDiagramBuilder<double> builder(0.0);
    Eigen::MatrixXd square(2, 4);
    square << 0, 1, 1, 0, 0, 0, 1, 1;
    const VPolytope flat(square);
    EXPECT_THROW(AddVPolytopeObstacle(&builder.plant(), flat,
                                      RigidTransformd::Identity(), "flat"),
                 std::exception);
  }
  {  // Post-finalize.
    RobotDiagramBuilder<double> builder(0.0);
    MultibodyPlant<double>& plant = builder.plant();
    plant.Finalize();
    EXPECT_THROW(AddVPolytopeObstacle(&plant, vpoly,
                                      RigidTransformd::Identity(), "late"),
                 std::exception);
  }
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
