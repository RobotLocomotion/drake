/* The displacement lemma and the J(p) subtree logic. An under-bounding λ(j, p)
 makes the certifier certify a colliding path, with no other symptom, so a
 failure here is a soundness bug in the kinematics module, not a test to loosen.

 Three property tests share one random-plant corpus: (1) move a single
 coordinate j ∈ J(p) and check that every sampled point of the distal side
 D(j, p) displaces by at most λ(j,p)·|Δq_j| in the other body's frame; (2) move
 all coordinates and check that every A-point-to-B-point distance changes by at
 most Σ λ(j,p)·|Δq_j|; (3) for pairs whose whole J(p) shares one distal side,
 check the stronger one-sided form.

 (2) is stated on distances rather than on B's points in A's frame because the
 distal side changes from joint to joint along a self-collision pair's J(p).
 Each telescoping step is bounded in the frame of that step's static side, and
 point-to-point distance is frame invariant. */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/planning/continuous_collision/motion_bound_table.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::GeometryId;
using drake::geometry::HalfSpace;
using drake::geometry::Sphere;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::BodyIndex;
using drake::multibody::CoulombFriction;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PlanarJoint;
using drake::multibody::PrismaticJoint;
using drake::multibody::QuaternionFloatingJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::ScrewJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::WeldJoint;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using Rng = std::mt19937_64;

/* Absolute slack on every displacement assertion. The claims are exact
 mathematics; this only absorbs floating-point noise in Drake's forward
 kinematics and in this test's own accumulation (both ~1e-15 here). */
constexpr double kSlack = 1e-9;

/* Options::continuity_tolerance's default: the width below which the curve
 module flags a coordinate constant and the carve-out removes it from every
 J(p). A coordinate carved on that *tolerance* can still move by up to this
 much, which is what MotionBoundTable::carveout_slack() charges for. */
constexpr double kContinuityTolerance = 1e-7;

// ---------------------------------------------------------------------------
// Small random utilities (seeded, deterministic).
// ---------------------------------------------------------------------------

double Uniform(Rng* rng, double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(*rng);
}

int UniformInt(Rng* rng, int lo, int hi) {
  return std::uniform_int_distribution<int>(lo, hi)(*rng);
}

Vector3d RandomUnitVector(Rng* rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Vector3d v;
  do {
    v = Vector3d(normal(*rng), normal(*rng), normal(*rng));
  } while (v.norm() < 1e-6);
  return v.normalized();
}

RotationMatrix<double> RandomRotation(Rng* rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Quaterniond q;
  do {
    q = Eigen::Quaterniond(normal(*rng), normal(*rng), normal(*rng),
                           normal(*rng));
  } while (q.norm() < 1e-6);
  q.normalize();
  return RotationMatrix<double>(q);
}

RigidTransform<double> RandomTransform(Rng* rng, double scale) {
  return RigidTransform<double>(
      RandomRotation(rng),
      Vector3d(Uniform(rng, -scale, scale), Uniform(rng, -scale, scale),
               Uniform(rng, -scale, scale)));
}

SpatialInertia<double> UnitInertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

// ---------------------------------------------------------------------------
// Surface sampling for the primitives the random worlds use.
// ---------------------------------------------------------------------------

Matrix3Xd SampleSphereSurface(Rng* rng, double r, int n) {
  Matrix3Xd p(3, n);
  for (int i = 0; i < n; ++i) p.col(i) = r * RandomUnitVector(rng);
  return p;
}

Matrix3Xd SampleBoxSurface(Rng* rng, const Vector3d& size, int n) {
  const Vector3d half = 0.5 * size;
  Matrix3Xd p(3, n);
  for (int i = 0; i < n; ++i) {
    Vector3d v(Uniform(rng, -half.x(), half.x()),
               Uniform(rng, -half.y(), half.y()),
               Uniform(rng, -half.z(), half.z()));
    const int axis = UniformInt(rng, 0, 2);
    v(axis) = (UniformInt(rng, 0, 1) == 0 ? -1.0 : 1.0) * half(axis);
    p.col(i) = v;
  }
  return p;
}

Matrix3Xd SampleCapsuleSurface(Rng* rng, double r, double length, int n) {
  const double half = 0.5 * length;
  Matrix3Xd p(3, n);
  for (int i = 0; i < n; ++i) {
    if (UniformInt(rng, 0, 1) == 0) {
      const double phi = Uniform(rng, 0.0, 2.0 * M_PI);
      p.col(i) = Vector3d(r * std::cos(phi), r * std::sin(phi),
                          Uniform(rng, -half, half));
    } else {
      const Vector3d u = RandomUnitVector(rng);
      const double z0 = u.z() >= 0.0 ? half : -half;
      p.col(i) = Vector3d(r * u.x(), r * u.y(), z0 + r * u.z());
    }
  }
  return p;
}

// ---------------------------------------------------------------------------
// A random world: a random tree of bodies with random joints, random fixed
// frame offsets on both sides of every joint, and random primitive geometries
// at random body-frame poses.
// ---------------------------------------------------------------------------

struct RandomWorld {
  std::unique_ptr<RobotDiagram<double>> diagram;
  /* Surface samples of each proximity geometry, expressed in its BODY frame
   (that is, X_BG already applied). */
  std::unordered_map<GeometryId, Matrix3Xd> points_B;
  int num_screw_joints{0};
};

/* Adds one random primitive geometry to `body`; records its surface samples in
 the body frame. */
void AddRandomGeometry(Rng* rng, MultibodyPlant<double>* plant,
                       const RigidBody<double>& body, const std::string& name,
                       int num_samples, RandomWorld* world) {
  const RigidTransform<double> X_BG = RandomTransform(rng, 0.2);
  GeometryId gid;
  Matrix3Xd p_G;
  switch (UniformInt(rng, 0, 2)) {
    case 0: {
      const double r = Uniform(rng, 0.02, 0.15);
      gid = plant->RegisterCollisionGeometry(body, X_BG, Sphere(r), name,
                                             CoulombFriction<double>(1.0, 1.0));
      p_G = SampleSphereSurface(rng, r, num_samples);
      break;
    }
    case 1: {
      const Vector3d size(Uniform(rng, 0.02, 0.3), Uniform(rng, 0.02, 0.3),
                          Uniform(rng, 0.02, 0.3));
      gid = plant->RegisterCollisionGeometry(body, X_BG, Box(size), name,
                                             CoulombFriction<double>(1.0, 1.0));
      p_G = SampleBoxSurface(rng, size, num_samples);
      break;
    }
    default: {
      const double r = Uniform(rng, 0.02, 0.1);
      const double length = Uniform(rng, 0.05, 0.4);
      gid =
          plant->RegisterCollisionGeometry(body, X_BG, Capsule(r, length), name,
                                           CoulombFriction<double>(1.0, 1.0));
      p_G = SampleCapsuleSurface(rng, r, length, num_samples);
      break;
    }
  }
  Matrix3Xd p_B(3, p_G.cols());
  for (int i = 0; i < p_G.cols(); ++i) p_B.col(i) = X_BG * p_G.col(i);
  world->points_B.emplace(gid, std::move(p_B));
}

RandomWorld MakeRandomWorld(Rng* rng, bool allow_screw, int num_samples) {
  RandomWorld world;
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();

  const int num_bodies = UniformInt(rng, 3, 7);
  std::vector<const RigidBody<double>*> bodies{&plant.world_body()};
  for (int i = 0; i < num_bodies; ++i) {
    const RigidBody<double>& body =
        plant.AddRigidBody(fmt::format("b{}", i), UnitInertia());
    // Parent is any earlier body (including the world), so the corpus mixes
    // serial chains with branching trees.
    const RigidBody<double>& parent =
        *bodies[UniformInt(rng, 0, static_cast<int>(bodies.size()) - 1)];
    const RigidTransform<double> X_PF = RandomTransform(rng, 0.25);
    const RigidTransform<double> X_CM = RandomTransform(rng, 0.25);
    const std::string jn = fmt::format("j{}", i);
    const int kind = UniformInt(rng, 0, allow_screw ? 4 : 3);
    switch (kind) {
      case 0:
        plant.AddJoint<RevoluteJoint>(jn, parent, X_PF, body, X_CM,
                                      RandomUnitVector(rng));
        break;
      case 1:
        plant.AddJoint<PrismaticJoint>(jn, parent, X_PF, body, X_CM,
                                       RandomUnitVector(rng));
        break;
      case 2:
        plant.AddJoint<PlanarJoint>(jn, parent, X_PF, body, X_CM,
                                    Vector3d::Zero());
        break;
      case 3:
        plant.AddJoint<WeldJoint>(jn, parent, X_PF, body, X_CM,
                                  RandomTransform(rng, 0.2));
        break;
      default:
        plant.AddJoint<ScrewJoint>(jn, parent, X_PF, body, X_CM,
                                   RandomUnitVector(rng),
                                   Uniform(rng, 0.05, 0.6), 0.0);
        ++world.num_screw_joints;
        break;
    }
    bodies.push_back(&body);
  }

  // Always give the world and the first body a geometry so every world has at
  // least one pair; sprinkle the rest randomly (some bodies get none, which
  // exercises geometry-free bodies contributing chain hops only).
  AddRandomGeometry(rng, &plant, plant.world_body(), "g_world", num_samples,
                    &world);
  for (size_t i = 1; i < bodies.size(); ++i) {
    const int count = (i == 1) ? 1 : UniformInt(rng, 0, 2);
    for (int g = 0; g < count; ++g) {
      AddRandomGeometry(rng, &plant, *bodies[i], fmt::format("g{}_{}", i, g),
                        num_samples, &world);
    }
  }

  world.diagram = builder.Build();
  return world;
}

// ---------------------------------------------------------------------------
// Plant introspection helpers used by the tests (independent of the module
// under test, so a bug in the module cannot hide behind them).
// ---------------------------------------------------------------------------

/* Subtree membership S_j for every joint that has velocities, straight from
 Drake. */
std::map<JointIndex, std::vector<bool>> SubtreeSets(
    const MultibodyPlant<double>& plant) {
  std::map<JointIndex, std::vector<bool>> out;
  for (JointIndex ji : plant.GetJointIndices()) {
    const auto& joint = plant.get_joint(ji);
    if (joint.num_velocities() == 0) continue;
    std::vector<bool> members(plant.num_bodies(), false);
    for (BodyIndex b : plant.GetBodiesKinematicallyAffectedBy({ji})) {
      members[b] = true;
    }
    out.emplace(ji, std::move(members));
  }
  return out;
}

/* Position coordinate -> owning JointIndex. */
std::vector<JointIndex> CoordinateOwners(const MultibodyPlant<double>& plant) {
  std::vector<JointIndex> owner(plant.num_positions());
  for (JointIndex ji : plant.GetJointIndices()) {
    const auto& joint = plant.get_joint(ji);
    for (int c = 0; c < joint.num_positions(); ++c) {
      owner[joint.position_start() + c] = ji;
    }
  }
  return owner;
}

/* True for coordinates that parameterize a rotation (used only to pick
 sensible random control-box widths). */
std::vector<bool> AngularCoordinates(const MultibodyPlant<double>& plant) {
  std::vector<bool> angular(plant.num_positions(), false);
  for (JointIndex ji : plant.GetJointIndices()) {
    const auto& joint = plant.get_joint(ji);
    const int ps = joint.position_start();
    if (joint.type_name() == "revolute" || joint.type_name() == "screw") {
      for (int c = 0; c < joint.num_positions(); ++c) angular[ps + c] = true;
    } else if (joint.type_name() == "planar") {
      angular[ps + 2] = true;
    }
  }
  return angular;
}

std::vector<PairId> CollisionPairs(const RobotDiagram<double>& diagram) {
  const MultibodyPlant<double>& plant = diagram.plant();
  const auto& inspector = diagram.scene_graph().model_inspector();
  std::vector<PairId> pairs;
  for (const auto& [ga, gb] : inspector.GetCollisionCandidates()) {
    const BodyIndex ba =
        plant.GetBodyFromFrameId(inspector.GetFrameId(ga))->index();
    const BodyIndex bb =
        plant.GetBodyFromFrameId(inspector.GetFrameId(gb))->index();
    pairs.push_back(PairId{ga, gb, ba, bb});
  }
  return pairs;
}

// ---------------------------------------------------------------------------
// Part 1. J(p) subtree logic on hand-built plants.
// ---------------------------------------------------------------------------

/* Convenience: the position coordinates of a named joint. */
std::vector<int> CoordsOf(const MultibodyPlant<double>& plant,
                          const std::string& joint_name) {
  const auto& joint = plant.GetJointByName(joint_name);
  std::vector<int> out;
  for (int c = 0; c < joint.num_positions(); ++c) {
    out.push_back(joint.position_start() + c);
  }
  return out;
}

std::vector<int> Merge(std::vector<std::vector<int>> groups) {
  std::vector<int> out;
  for (const auto& g : groups) out.insert(out.end(), g.begin(), g.end());
  std::sort(out.begin(), out.end());
  return out;
}

GTEST_TEST(JointSupportTest, SerialChain) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& env = plant.AddRigidBody("env", UnitInertia());
  const auto& l1 = plant.AddRigidBody("l1", UnitInertia());
  const auto& l2 = plant.AddRigidBody("l2", UnitInertia());
  const auto& l3 = plant.AddRigidBody("l3", UnitInertia());
  plant.AddJoint<WeldJoint>("w_env", plant.world_body(), {}, env, {},
                            RigidTransform<double>(Vector3d(1.0, 0.0, 0.0)));
  plant.AddJoint<RevoluteJoint>("j1", plant.world_body(), {}, l1, {},
                                Vector3d::UnitZ());
  plant.AddJoint<RevoluteJoint>("j2", l1, {}, l2, {}, Vector3d::UnitY());
  plant.AddJoint<PrismaticJoint>("j3", l2, {}, l3, {}, Vector3d::UnitX());
  auto diagram = builder.Build();
  const KinematicsEngine engine(*diagram);
  const auto& p = diagram->plant();

  const std::vector<int> j1 = CoordsOf(p, "j1");
  const std::vector<int> j2 = CoordsOf(p, "j2");
  const std::vector<int> j3 = CoordsOf(p, "j3");

  // Robot vs. anchored environment: the ancestors of the robot body.
  EXPECT_EQ(engine.CoordinatesAffectingPair(env.index(), l3.index()),
            Merge({j1, j2, j3}));
  EXPECT_EQ(engine.CoordinatesAffectingPair(p.world_body().index(), l2.index()),
            Merge({j1, j2}));
  // Self pair through the common ancestor: the path between the two bodies.
  EXPECT_EQ(engine.CoordinatesAffectingPair(l1.index(), l3.index()),
            Merge({j2, j3}));
  // A body against itself, and two anchored bodies, are static.
  EXPECT_TRUE(engine.CoordinatesAffectingPair(l3.index(), l3.index()).empty());
  EXPECT_TRUE(
      engine.CoordinatesAffectingPair(p.world_body().index(), env.index())
          .empty());
}

GTEST_TEST(JointSupportTest, BranchingTree) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& b1 = plant.AddRigidBody("b1", UnitInertia());
  const auto& left = plant.AddRigidBody("left", UnitInertia());
  const auto& right = plant.AddRigidBody("right", UnitInertia());
  const auto& left_tip = plant.AddRigidBody("left_tip", UnitInertia());
  plant.AddJoint<RevoluteJoint>("j0", plant.world_body(), {}, b1, {},
                                Vector3d::UnitZ());
  plant.AddJoint<RevoluteJoint>("jl", b1, {}, left, {}, Vector3d::UnitY());
  plant.AddJoint<PlanarJoint>("jr", b1, {}, right, {}, Vector3d::Zero());
  plant.AddJoint<RevoluteJoint>("jt", left, {}, left_tip, {},
                                Vector3d::UnitX());
  auto diagram = builder.Build();
  const KinematicsEngine engine(*diagram);
  const auto& p = diagram->plant();

  const std::vector<int> j0 = CoordsOf(p, "j0");
  const std::vector<int> jl = CoordsOf(p, "jl");
  const std::vector<int> jr = CoordsOf(p, "jr");
  const std::vector<int> jt = CoordsOf(p, "jt");
  ASSERT_EQ(jr.size(), 3);  // A planar joint contributes three coordinates.

  // Symmetric difference across the common ancestor b1: j0 affects both sides
  // and drops out.
  EXPECT_EQ(engine.CoordinatesAffectingPair(left_tip.index(), right.index()),
            Merge({jl, jr, jt}));
  EXPECT_EQ(
      engine.CoordinatesAffectingPair(p.world_body().index(), left_tip.index()),
      Merge({j0, jl, jt}));
  EXPECT_EQ(engine.CoordinatesAffectingPair(left.index(), left_tip.index()),
            Merge({jt}));
}

GTEST_TEST(JointSupportTest, WeldedClusterMovesAsOneBody) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& arm = plant.AddRigidBody("arm", UnitInertia());
  const auto& hand = plant.AddRigidBody("hand", UnitInertia());
  const auto& finger = plant.AddRigidBody("finger", UnitInertia());
  const auto& anchored = plant.AddRigidBody("anchored", UnitInertia());
  plant.AddJoint<RevoluteJoint>("j0", plant.world_body(), {}, arm, {},
                                Vector3d::UnitZ());
  plant.AddJoint<WeldJoint>("w1", arm, {}, hand, {},
                            RigidTransform<double>(Vector3d(0.2, 0.0, 0.0)));
  plant.AddJoint<WeldJoint>("w2", hand, {}, finger, {},
                            RigidTransform<double>(Vector3d(0.05, 0.0, 0.0)));
  plant.AddJoint<WeldJoint>("w3", plant.world_body(), {}, anchored, {},
                            RigidTransform<double>(Vector3d(0.0, 1.0, 0.0)));
  auto diagram = builder.Build();
  const KinematicsEngine engine(*diagram);
  const auto& p = diagram->plant();

  // Everything inside a welded cluster is mutually static ...
  EXPECT_TRUE(
      engine.CoordinatesAffectingPair(arm.index(), finger.index()).empty());
  EXPECT_TRUE(
      engine.CoordinatesAffectingPair(hand.index(), finger.index()).empty());
  EXPECT_TRUE(
      engine.CoordinatesAffectingPair(p.world_body().index(), anchored.index())
          .empty());
  // ... and the whole cluster inherits the revolute coordinate of its chain.
  EXPECT_EQ(engine.CoordinatesAffectingPair(anchored.index(), finger.index()),
            CoordsOf(p, "j0"));
}

GTEST_TEST(JointSupportTest, ConstantCoordinateCarveOutEmptiesJp) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& l1 = plant.AddRigidBody("l1", UnitInertia());
  const auto& l2 = plant.AddRigidBody("l2", UnitInertia());
  plant.AddJoint<RevoluteJoint>("j1", plant.world_body(), {}, l1, {},
                                Vector3d::UnitZ());
  plant.AddJoint<RevoluteJoint>("j2", l1, {}, l2, {}, Vector3d::UnitY());
  plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransform<double>::Identity(), Sphere(0.1),
      "g_world", CoulombFriction<double>(1.0, 1.0));
  plant.RegisterCollisionGeometry(
      l2, RigidTransform<double>(Vector3d(0.3, 0, 0)), Sphere(0.05), "g_tip",
      CoulombFriction<double>(1.0, 1.0));
  auto diagram = builder.Build();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);

  const int nq = diagram->plant().num_positions();
  const VectorXd lower = VectorXd::Constant(nq, -0.5);
  const VectorXd upper = VectorXd::Constant(nq, 0.5);

  {  // Nothing constant: both coordinates appear.
    const MotionBoundTable table = engine.ComputeMotionBoundTable(
        lower, upper, std::vector<bool>(nq, false), pairs);
    ASSERT_EQ(table.num_pairs(), 1);
    EXPECT_FALSE(table.pair_is_static(0));
    EXPECT_EQ(table.GetEntries(0).size(), 2);
  }
  {  // One constant: only the other survives.
    std::vector<bool> constant(nq, false);
    constant[0] = true;
    const MotionBoundTable table =
        engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
    ASSERT_EQ(table.GetEntries(0).size(), 1);
    EXPECT_EQ(table.GetEntries(0)[0].first, 1);
  }
  {  // All constant, and *exactly* so (the box collapses with the flags, as it
     // does for a real path): the pair becomes static and its motion bound is
     // exactly zero.
    const VectorXd pinned = VectorXd::Constant(nq, 0.25);
    const MotionBoundTable table = engine.ComputeMotionBoundTable(
        pinned, pinned, std::vector<bool>(nq, true), pairs);
    EXPECT_TRUE(table.pair_is_static(0));
    EXPECT_EQ(table.carveout_slack(0), 0.0);
    EXPECT_EQ(table.MotionBound(0, VectorXd::Constant(nq, 1.0)), 0.0);
  }
}

// ---------------------------------------------------------------------------
// Part 1b. The joint-type and half-space carve-outs.
// ---------------------------------------------------------------------------

GTEST_TEST(JointSupportTest, ReversedJointThrowsWithAnActionableMessage) {
  // A joint whose declared parent ends up OUTBOARD of its declared child once
  // the tree is rooted at the world. Drake reverses the mobilizer internally;
  // the reach chain does not model that, so the library rejects it by name.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& a = plant.AddRigidBody("body_a", UnitInertia());
  const auto& b = plant.AddRigidBody("body_b", UnitInertia());
  plant.AddJoint<WeldJoint>("w", plant.world_body(), {}, b, {},
                            RigidTransform<double>(Vector3d(0.1, 0.0, 0.0)));
  // Parent is `a` (which hangs off `b`), child is `b` (already anchored).
  plant.AddJoint<RevoluteJoint>("reversed", a, {}, b, {}, Vector3d::UnitZ());
  std::unique_ptr<RobotDiagram<double>> diagram;
  try {
    diagram = builder.Build();
  } catch (const std::exception& e) {
    GTEST_SKIP() << "this Drake refuses the model outright: " << e.what();
  }
  try {
    const KinematicsEngine engine(*diagram);
    GTEST_FAIL() << "expected a throw for a reversed joint";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("reversed"), std::string::npos) << what;
  }
}

/* world --(revolute)--> link, with a half space on `halfspace_on_link` and a
 sphere on the other body. */
std::unique_ptr<RobotDiagram<double>> MakeHalfSpaceModel(bool halfspace_on_link,
                                                         bool prismatic) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& link = plant.AddRigidBody("link", UnitInertia());
  if (prismatic) {
    plant.AddJoint<PrismaticJoint>("j", plant.world_body(), {}, link, {},
                                   Vector3d::UnitZ());
  } else {
    plant.AddJoint<RevoluteJoint>("j", plant.world_body(), {}, link, {},
                                  Vector3d::UnitY());
  }
  const CoulombFriction<double> mu(1.0, 1.0);
  const RigidTransform<double> I = RigidTransform<double>::Identity();
  if (halfspace_on_link) {
    plant.RegisterCollisionGeometry(link, I, HalfSpace(), "hs", mu);
    plant.RegisterCollisionGeometry(plant.world_body(),
                                    RigidTransform<double>(Vector3d(0, 0, 1.0)),
                                    Sphere(0.1), "ball", mu);
  } else {
    plant.RegisterCollisionGeometry(plant.world_body(), I, HalfSpace(), "hs",
                                    mu);
    plant.RegisterCollisionGeometry(link,
                                    RigidTransform<double>(Vector3d(0.3, 0, 0)),
                                    Sphere(0.1), "ball", mu);
  }
  return builder.Build();
}

GTEST_TEST(HalfSpaceRuleTest, AnchoredGroundPlaneIsAccepted) {
  // The canonical case: a ground plane on the world with a rotating arm above
  // it. The half space is never the *distal* side, so λ bounds the arm's
  // points and the pair is perfectly certifiable.
  auto diagram = MakeHalfSpaceModel(/* halfspace_on_link = */ false, false);
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = diagram->plant().num_positions();
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      VectorXd::Constant(nq, -1.0), VectorXd::Constant(nq, 1.0),
      std::vector<bool>(nq, false), pairs);
  ASSERT_EQ(table.GetEntries(0).size(), 1);
  EXPECT_GT(table.GetEntries(0)[0].second, 0.0);
  EXPECT_TRUE(std::isfinite(table.GetEntries(0)[0].second));
}

GTEST_TEST(HalfSpaceRuleTest, RotatingHalfSpaceThrowsAtConstruction) {
  auto diagram = MakeHalfSpaceModel(/* halfspace_on_link = */ true, false);
  try {
    const KinematicsEngine engine(*diagram);
    GTEST_FAIL()
        << "expected a throw for a half space with revolute relative motion";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("hs"), std::string::npos) << what;
    EXPECT_NE(what.find("HalfSpace"), std::string::npos) << what;
  }
}

GTEST_TEST(HalfSpaceRuleTest, TranslatingHalfSpaceIsAccepted) {
  // Pure translation keeps every point of the half space moving by |Δq|, so
  // λ = 1 is finite and correct even though the reach is not.
  auto diagram = MakeHalfSpaceModel(/* halfspace_on_link = */ true, true);
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = diagram->plant().num_positions();
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      VectorXd::Constant(nq, -1.0), VectorXd::Constant(nq, 1.0),
      std::vector<bool>(nq, false), pairs);
  ASSERT_EQ(table.GetEntries(0).size(), 1);
  EXPECT_EQ(table.GetEntries(0)[0].second, 1.0);
}

/* world --(revolute j0)--> b1 --(quaternion floating)--> b2 --(revolute j1)-->
 b3, with geometry on the world and on b3. The floating joint sits mid-chain so
 that the reach for j0 has to cross it, which is where its X_FM translation
 must be picked up from the control box. */
std::unique_ptr<RobotDiagram<double>> MakeMidChainFloatingModel() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& b1 = plant.AddRigidBody("b1", UnitInertia());
  const auto& b2 = plant.AddRigidBody("b2", UnitInertia());
  const auto& b3 = plant.AddRigidBody("b3", UnitInertia());
  plant.AddJoint<RevoluteJoint>("j0", plant.world_body(), {}, b1, {},
                                Vector3d::UnitZ());
  plant.AddJoint<QuaternionFloatingJoint>(
      "jf", b1, RigidTransform<double>(Vector3d(0.1, 0.0, 0.0)), b2,
      RigidTransform<double>(Vector3d(0.0, 0.05, 0.0)));
  plant.AddJoint<RevoluteJoint>(
      "j1", b2, RigidTransform<double>(Vector3d(0.0, 0.0, 0.15)), b3,
      RigidTransform<double>(Vector3d(0.07, 0.0, 0.0)), Vector3d::UnitY());
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>::Identity(),
                                  Sphere(0.1), "g_world", mu);
  plant.RegisterCollisionGeometry(b3,
                                  RigidTransform<double>(Vector3d(0.2, 0, 0)),
                                  Sphere(0.05), "g_tip", mu);
  return builder.Build();
}

GTEST_TEST(JointSupportTest, MovingQuaternionFloatingJointThrows) {
  auto diagram = MakeMidChainFloatingModel();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  const int nq = diagram->plant().num_positions();
  try {
    engine.ComputeMotionBoundTable(VectorXd::Constant(nq, -0.5),
                                   VectorXd::Constant(nq, 0.5),
                                   std::vector<bool>(nq, false), pairs);
    GTEST_FAIL() << "expected a throw for a moving quaternion floating joint";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("jf"), std::string::npos) << what;
    EXPECT_NE(what.find("quaternion_floating"), std::string::npos) << what;
    EXPECT_NE(what.find("constant"), std::string::npos) << what;
  }
}

GTEST_TEST(JointSupportTest, ConstantFloatingBaseCarveOutIsSoundMidChain) {
  auto diagram = MakeMidChainFloatingModel();
  const MultibodyPlant<double>& plant = diagram->plant();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);

  const auto& jf = plant.GetJointByName("jf");
  const int nq = plant.num_positions();
  ASSERT_EQ(jf.num_positions(), 7);

  // The floating pose is pinned: identity orientation, a large offset.
  const Vector3d p_FM(0.9, -0.7, 0.4);
  VectorXd q0 = VectorXd::Zero(nq);
  std::vector<bool> constant(nq, false);
  const int fs = jf.position_start();
  q0[fs] = 1.0;  // w of the wxyz quaternion.
  q0.segment<3>(fs + 4) = p_FM;
  for (int c = fs; c < fs + 7; ++c) constant[c] = true;

  VectorXd lower = q0;
  VectorXd upper = q0;
  const auto& j0 = plant.GetJointByName("j0");
  const auto& j1 = plant.GetJointByName("j1");
  for (int c : {j0.position_start(), j1.position_start()}) {
    lower[c] = -1.0;
    upper[c] = 1.0;
  }
  const MotionBoundTable table =
      engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
  ASSERT_EQ(table.GetEntries(0).size(), 2);

  // The reach for j0 must include the floating joint's 1.22 m offset; a bound
  // that silently dropped it would be far too small.
  double lambda_j0 = 0.0;
  for (const auto& [c, lam] : table.GetEntries(0)) {
    if (c == j0.position_start()) lambda_j0 = lam;
  }
  EXPECT_GT(lambda_j0, p_FM.norm());

  // And the displacement lemma must hold on this model.
  auto root = diagram->CreateDefaultContext();
  auto& ctx = plant.GetMyMutableContextFromRoot(root.get());
  Rng rng(0xF10A7);
  const Matrix3Xd points_B =
      SampleSphereSurface(&rng, 0.05, 128).colwise() + Vector3d(0.2, 0, 0);
  const auto& frame_tip = plant.GetBodyByName("b3").body_frame();
  const auto& frame_world = plant.world_frame();
  for (int trial = 0; trial < 200; ++trial) {
    VectorXd q = q0;
    VectorXd qp = q0;
    for (int c : {j0.position_start(), j1.position_start()}) {
      q[c] = Uniform(&rng, lower[c], upper[c]);
      qp[c] = Uniform(&rng, lower[c], upper[c]);
    }
    Matrix3Xd out_q(3, points_B.cols());
    Matrix3Xd out_qp(3, points_B.cols());
    plant.SetPositions(&ctx, q);
    plant.CalcPointsPositions(ctx, frame_tip, points_B, frame_world, &out_q);
    plant.SetPositions(&ctx, qp);
    plant.CalcPointsPositions(ctx, frame_tip, points_B, frame_world, &out_qp);
    const double displacement = (out_qp - out_q).colwise().norm().maxCoeff();
    const double bound = table.MotionBound(0, (qp - q).cwiseAbs());
    ASSERT_LE(displacement, bound + kSlack)
        << "trial " << trial << ": displacement " << displacement << " > bound "
        << bound;
  }
}

// ---------------------------------------------------------------------------
// Part 1c. An exactly tight reach chain.
//
// The chain walk's triangle inequalities are slack at random poses, so a term
// that is merely too small hides inside that slack in the randomized corpus.
// Here every offset lies along +x with identity rotation, giving
// ‖a + b‖ = ‖a‖ + ‖b‖ at every hop, so each contribution to r shows up in λ.
//
//   world --j_top(axis ẑ)--> b1 --j_slide(axis x̂)--> b2 --weld--> b3(sphere)
//
// with, all along x̂: ‖p_CM(j_top)‖ = L1, ‖p_PF(j_slide)‖ = d1, the slide's box
// maximum s, ‖p_CM(j_slide)‖ = d2, the weld's ‖p_PF‖ = e1, ‖X_FM‖ = e2,
// ‖p_CM‖ = e3, and the sphere reaching L3 + ρ from b3's origin. At the slide's
// box maximum the farthest sphere point sits at exactly
//   r = (L3 + ρ) + (e3 + e2 + e1) + (d2 + s + d1) + L1
// from j_top's M-frame origin, in the plane normal to the joint axis.
// ---------------------------------------------------------------------------

struct TightChain {
  std::unique_ptr<RobotDiagram<double>> diagram;
  double expected_reach{};
  double slide_max{};
  Vector3d far_point_b3;  // The exactly-reaching material point, in b3's frame.
};

TightChain MakeTightChain(bool screw_top, double screw_pitch) {
  constexpr double kL1 = 0.37, kD1 = 0.29, kS = 0.53, kD2 = 0.19;
  constexpr double kE1 = 0.11, kE2 = 0.23, kE3 = 0.17;
  constexpr double kL3 = 0.31, kRho = 0.13;
  const auto tx = [](double x) {
    return RigidTransform<double>(Vector3d(x, 0.0, 0.0));
  };

  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& b1 = plant.AddRigidBody("b1", UnitInertia());
  const auto& b2 = plant.AddRigidBody("b2", UnitInertia());
  const auto& b3 = plant.AddRigidBody("b3", UnitInertia());
  if (screw_top) {
    plant.AddJoint<ScrewJoint>("j_top", plant.world_body(), tx(0.0), b1,
                               tx(-kL1), Vector3d::UnitZ(), screw_pitch, 0.0);
  } else {
    plant.AddJoint<RevoluteJoint>("j_top", plant.world_body(), tx(0.0), b1,
                                  tx(-kL1), Vector3d::UnitZ());
  }
  plant.AddJoint<PrismaticJoint>("j_slide", b1, tx(kD1), b2, tx(-kD2),
                                 Vector3d::UnitX());
  plant.AddJoint<WeldJoint>("j_weld", b2, tx(kE1), b3, tx(-kE3), tx(kE2));
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>::Identity(),
                                  Sphere(0.02), "g_world", mu);
  plant.RegisterCollisionGeometry(b3, tx(kL3), Sphere(kRho), "g_tip", mu);

  TightChain out;
  out.diagram = builder.Build();
  out.expected_reach =
      (kL3 + kRho) + (kE3 + kE2 + kE1) + (kD2 + kS + kD1) + kL1;
  out.slide_max = kS;
  out.far_point_b3 = Vector3d(kL3 + kRho, 0.0, 0.0);
  return out;
}

/* Locates the (world geometry, tip geometry) pair. */
int FindPairIndex(const RobotDiagram<double>& diagram,
                  const std::vector<PairId>& pairs, const std::string& name_a,
                  const std::string& name_b) {
  const auto& inspector = diagram.scene_graph().model_inspector();
  for (int k = 0; k < static_cast<int>(pairs.size()); ++k) {
    const std::string a = inspector.GetName(pairs[k].a);
    const std::string b = inspector.GetName(pairs[k].b);
    if ((a.find(name_a) != std::string::npos &&
         b.find(name_b) != std::string::npos) ||
        (a.find(name_b) != std::string::npos &&
         b.find(name_a) != std::string::npos)) {
      return k;
    }
  }
  return -1;
}

GTEST_TEST(ReachTest, RevoluteChainIsExactAndTight) {
  const TightChain chain = MakeTightChain(/* screw_top = */ false, 0.0);
  const MultibodyPlant<double>& plant = chain.diagram->plant();
  const KinematicsEngine engine(*chain.diagram);
  const std::vector<PairId> pairs = CollisionPairs(*chain.diagram);
  const int k = FindPairIndex(*chain.diagram, pairs, "g_world", "g_tip");
  ASSERT_GE(k, 0);

  const int nq = plant.num_positions();
  const auto& j_top = plant.GetJointByName("j_top");
  const auto& j_slide = plant.GetJointByName("j_slide");
  VectorXd lower = VectorXd::Zero(nq);
  VectorXd upper = VectorXd::Zero(nq);
  lower[j_top.position_start()] = -1.0;
  upper[j_top.position_start()] = 1.0;
  upper[j_slide.position_start()] = chain.slide_max;
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      lower, upper, std::vector<bool>(nq, false), pairs);

  double lambda_top = 0.0;
  double lambda_slide = 0.0;
  for (const auto& [c, lam] : table.GetEntries(k)) {
    if (c == j_top.position_start()) lambda_top = lam;
    if (c == j_slide.position_start()) lambda_slide = lam;
  }
  // Every hop contributes digit for digit: p_CM at the top, both frame offsets
  // and the box maximum of the slide, all three legs of the weld (including
  // its X_FM translation), and the geometry's own reach past b3's origin.
  EXPECT_NEAR(lambda_top, chain.expected_reach, 1e-12);
  EXPECT_EQ(lambda_slide, 1.0);

  // And the bound is attained: with the slide at its box maximum and a small
  // Δθ, the chord 2r·sin(Δθ/2) recovers r·Δθ to eight digits.
  auto root = chain.diagram->CreateDefaultContext();
  auto& ctx = plant.GetMyMutableContextFromRoot(root.get());
  VectorXd q = VectorXd::Zero(nq);
  q[j_slide.position_start()] = chain.slide_max;
  VectorXd qp = q;
  const double dtheta = 1e-4;
  qp[j_top.position_start()] = dtheta;
  Matrix3Xd p_B3(3, 1);
  p_B3.col(0) = chain.far_point_b3;
  Matrix3Xd before(3, 1);
  Matrix3Xd after(3, 1);
  const auto& frame_tip = plant.GetBodyByName("b3").body_frame();
  plant.SetPositions(&ctx, q);
  plant.CalcPointsPositions(ctx, frame_tip, p_B3, plant.world_frame(), &before);
  plant.SetPositions(&ctx, qp);
  plant.CalcPointsPositions(ctx, frame_tip, p_B3, plant.world_frame(), &after);
  const double displacement = (after - before).norm();
  const double bound = lambda_top * dtheta;
  EXPECT_LE(displacement, bound + kSlack);
  EXPECT_GT(displacement / bound, 1.0 - 1e-8)
      << "the reach must be exactly attained on this chain; a slack bound here "
         "would mean a term is over-counted, and a violated bound would mean a "
         "term is missing";

  // The whole-box motion bound must dominate the true displacement too.
  const VectorXd dq = (qp - q).cwiseAbs();
  EXPECT_LE(displacement, table.MotionBound(k, dq) + kSlack);
}

GTEST_TEST(ReachTest, ScrewLambdaIncludesPitchAndIsNecessary) {
  constexpr double kPitch = 8.0;  // meters of travel per revolution.
  const TightChain chain = MakeTightChain(/* screw_top = */ true, kPitch);
  const MultibodyPlant<double>& plant = chain.diagram->plant();
  const KinematicsEngine engine(*chain.diagram);
  const std::vector<PairId> pairs = CollisionPairs(*chain.diagram);
  const int k = FindPairIndex(*chain.diagram, pairs, "g_world", "g_tip");
  ASSERT_GE(k, 0);

  const int nq = plant.num_positions();
  const auto& j_top = plant.GetJointByName("j_top");
  const auto& j_slide = plant.GetJointByName("j_slide");
  VectorXd lower = VectorXd::Zero(nq);
  VectorXd upper = VectorXd::Zero(nq);
  lower[j_top.position_start()] = -1.0;
  upper[j_top.position_start()] = 1.0;
  upper[j_slide.position_start()] = chain.slide_max;
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      lower, upper, std::vector<bool>(nq, false), pairs);

  double lambda_top = 0.0;
  for (const auto& [c, lam] : table.GetEntries(k)) {
    if (c == j_top.position_start()) lambda_top = lam;
  }
  const double pitch_term = kPitch / (2.0 * M_PI);
  EXPECT_NEAR(lambda_top, chain.expected_reach + pitch_term, 1e-12);

  auto root = chain.diagram->CreateDefaultContext();
  auto& ctx = plant.GetMyMutableContextFromRoot(root.get());
  VectorXd q = VectorXd::Zero(nq);
  q[j_slide.position_start()] = chain.slide_max;
  VectorXd qp = q;
  const double dtheta = 1e-4;
  qp[j_top.position_start()] = dtheta;
  Matrix3Xd p_B3(3, 1);
  p_B3.col(0) = chain.far_point_b3;
  Matrix3Xd before(3, 1);
  Matrix3Xd after(3, 1);
  const auto& frame_tip = plant.GetBodyByName("b3").body_frame();
  plant.SetPositions(&ctx, q);
  plant.CalcPointsPositions(ctx, frame_tip, p_B3, plant.world_frame(), &before);
  plant.SetPositions(&ctx, qp);
  plant.CalcPointsPositions(ctx, frame_tip, p_B3, plant.world_frame(), &after);
  const double displacement = (after - before).norm();

  EXPECT_LE(displacement, lambda_top * dtheta + kSlack);
  // The helix's axial travel is orthogonal to the chord it sweeps, so the true
  // displacement is √(r² + (pitch/2π)²)·Δθ, strictly larger than r·Δθ, so
  // dropping the pitch term would be unsound, not merely conservative.
  EXPECT_GT(displacement, chain.expected_reach * dtheta * (1.0 + 1e-6))
      << "a screw λ of r alone would under-bound this motion";
  const double exact = std::hypot(chain.expected_reach, pitch_term) * dtheta;
  EXPECT_NEAR(displacement, exact, 1e-11);
}

// ---------------------------------------------------------------------------
// Part 2. The displacement lemma property test.
// ---------------------------------------------------------------------------

struct LemmaStats {
  int plants{0};
  int pairs{0};
  int atomic_checks{0};
  int aggregate_checks{0};
  int one_sided_checks{0};
  int screw_joints{0};
  /* Largest observed displacement / bound ratio. A corpus in which this stays
   near zero would pass no matter how wrong λ is, so the tests assert it gets
   close to 1: the bound must be *tight somewhere*, which is what makes the
   property test sensitive to an under-bound. */
  double max_tightness{0.0};
  /* Pairs that were charged a nonzero carve-out slack, and the largest such
   slack seen. Guards against the sub-tolerance corpus degenerating into the
   exactly-constant one, which would test nothing new. */
  int slack_charged_pairs{0};
  double max_slack{0.0};

  void Observe(double achieved, double bound) {
    if (bound > 1e-12) {
      max_tightness = std::max(max_tightness, achieved / bound);
    }
  }
};

/* How the random control box treats the coordinates it flags constant. */
enum class CarveOut {
  /* No coordinate is flagged constant. */
  kNone,
  /* Flagged coordinates collapse to a single point: the carve-out is exact and
   the slack must be bit-exactly zero. */
  kExact,
  /* Flagged coordinates keep a random *sub-tolerance* width, which is what the
   curve module's tolerance-based flag actually admits. The carve-out then owes
   a residual, and MotionBoundTable::carveout_slack() must pay for it. */
  kSubTolerance,
};

/* Runs every displacement-lemma check on one random world. */
void CheckWorld(Rng* rng, const RandomWorld& world, CarveOut carve_out,
                LemmaStats* stats) {
  const RobotDiagram<double>& diagram = *world.diagram;
  const MultibodyPlant<double>& plant = diagram.plant();
  const KinematicsEngine engine(diagram);
  const std::vector<PairId> pairs = CollisionPairs(diagram);
  if (pairs.empty()) return;

  const int nq = plant.num_positions();
  const std::vector<bool> angular = AngularCoordinates(plant);
  const std::map<JointIndex, std::vector<bool>> subtrees = SubtreeSets(plant);
  const std::vector<JointIndex> owner = CoordinateOwners(plant);

  // A random control box around a random nominal configuration.
  VectorXd q0(nq);
  VectorXd lower(nq);
  VectorXd upper(nq);
  std::vector<bool> constant(nq, false);
  for (int c = 0; c < nq; ++c) {
    q0[c] = angular[c] ? Uniform(rng, -M_PI, M_PI) : Uniform(rng, -0.5, 0.5);
    const bool is_constant =
        carve_out != CarveOut::kNone && Uniform(rng, 0.0, 1.0) < 0.3;
    constant[c] = is_constant;
    double half;
    if (is_constant) {
      // A tolerance-carved coordinate keeps a nonzero, sub-tolerance width:
      // the box the curve module would hand us, not a collapsed point. The
      // samples below draw from that width too, so the residual is genuinely
      // exercised rather than assumed away.
      half = (carve_out == CarveOut::kSubTolerance)
                 ? 0.5 * Uniform(rng, 0.02 * kContinuityTolerance,
                                 kContinuityTolerance)
                 : 0.0;
    } else {
      half = angular[c] ? Uniform(rng, 0.05, 1.2) : Uniform(rng, 0.02, 0.4);
    }
    lower[c] = q0[c] - half;
    upper[c] = q0[c] + half;
  }

  const MotionBoundTable table =
      engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
  ASSERT_EQ(table.num_pairs(), static_cast<int>(pairs.size()));
  ++stats->plants;

  auto root = diagram.CreateDefaultContext();
  auto& ctx = plant.GetMyMutableContextFromRoot(root.get());

  // The λ table's coordinate sets must be exactly J(p) minus the constants.
  for (int k = 0; k < table.num_pairs(); ++k) {
    std::vector<int> expected;
    for (int c :
         engine.CoordinatesAffectingPair(pairs[k].body_a, pairs[k].body_b)) {
      if (!constant[c]) expected.push_back(c);
    }
    std::vector<int> actual;
    for (const auto& [c, lam] : table.GetEntries(k)) {
      actual.push_back(c);
      ASSERT_TRUE(std::isfinite(lam));
      ASSERT_GE(lam, 0.0);
    }
    ASSERT_EQ(actual, expected) << "pair " << k;
    ASSERT_EQ(table.pair_is_static(k), expected.empty());

    // The carve-out slack is a *residual*, so it is exactly zero unless some
    // carved coordinate genuinely keeps a nonzero width.
    const double pair_slack = table.carveout_slack(k);
    ASSERT_TRUE(std::isfinite(pair_slack));
    ASSERT_GE(pair_slack, 0.0);
    if (carve_out != CarveOut::kSubTolerance) {
      ASSERT_EQ(pair_slack, 0.0) << "pair " << k;
    }
    if (pair_slack > 0.0) {
      ++stats->slack_charged_pairs;
      stats->max_slack = std::max(stats->max_slack, pair_slack);
    }
  }

  for (int sample = 0; sample < 4; ++sample) {
    VectorXd q(nq);
    VectorXd qp(nq);
    for (int c = 0; c < nq; ++c) {
      q[c] = Uniform(rng, lower[c], upper[c]);
      qp[c] = Uniform(rng, lower[c], upper[c]);
    }
    const VectorXd dq = (qp - q).cwiseAbs();

    for (int k = 0; k < table.num_pairs(); ++k) {
      const PairId& pair = pairs[k];
      const Matrix3Xd& pts_a = world.points_B.at(pair.a);
      const Matrix3Xd& pts_b = world.points_B.at(pair.b);
      const auto& frame_a = plant.get_body(pair.body_a).body_frame();
      const auto& frame_b = plant.get_body(pair.body_b).body_frame();
      const double bound = table.MotionBound(k, dq);
      ASSERT_TRUE(std::isfinite(bound));
      ++stats->pairs;

      // ---- (1) Atomic, one coordinate at a time. -----------------------
      bool single_distal_side = true;
      BodyIndex common_distal;
      for (const auto& [c, lam] : table.GetEntries(k)) {
        const std::vector<bool>& S = subtrees.at(owner[c]);
        ASSERT_NE(S[pair.body_a], S[pair.body_b]);
        const BodyIndex distal = S[pair.body_a] ? pair.body_a : pair.body_b;
        const BodyIndex other = S[pair.body_a] ? pair.body_b : pair.body_a;
        if (!common_distal.is_valid()) {
          common_distal = distal;
        } else if (common_distal != distal) {
          single_distal_side = false;
        }
        const Matrix3Xd& pts = (distal == pair.body_a) ? pts_a : pts_b;
        const auto& frame_d = (distal == pair.body_a) ? frame_a : frame_b;
        const auto& frame_o = (distal == pair.body_a) ? frame_b : frame_a;

        VectorXd q_step = q;
        q_step[c] = qp[c];
        Matrix3Xd before(3, pts.cols());
        Matrix3Xd after(3, pts.cols());
        plant.SetPositions(&ctx, q);
        plant.CalcPointsPositions(ctx, frame_d, pts, frame_o, &before);
        plant.SetPositions(&ctx, q_step);
        plant.CalcPointsPositions(ctx, frame_d, pts, frame_o, &after);
        const double displacement =
            (after - before).colwise().norm().maxCoeff();
        ASSERT_LE(displacement, lam * dq[c] + kSlack)
            << "atomic step: pair " << k << ", coordinate " << c << ", λ "
            << lam << ", |Δq| " << dq[c] << ", distal body "
            << plant.get_body(distal).name() << ", other body "
            << plant.get_body(other).name();
        stats->Observe(displacement, lam * dq[c]);
        ++stats->atomic_checks;
      }

      if (table.pair_is_static(k)) {
        // A static pair may move only by the carve-out residual, which is
        // exactly zero when every carved coordinate is exactly constant.
        Matrix3Xd before(3, pts_b.cols());
        Matrix3Xd after(3, pts_b.cols());
        plant.SetPositions(&ctx, q);
        plant.CalcPointsPositions(ctx, frame_b, pts_b, frame_a, &before);
        plant.SetPositions(&ctx, qp);
        plant.CalcPointsPositions(ctx, frame_b, pts_b, frame_a, &after);
        ASSERT_LE((after - before).colwise().norm().maxCoeff(), bound + kSlack)
            << "pair " << k << " has empty J(p) but its relative pose moved by "
            << "more than its carve-out slack " << bound;
        continue;
      }

      // ---- (2) Aggregate: material-point distances. ---------------------
      // Subsample: 24 × 24 point pairs is plenty to catch an under-bound and
      // keeps the whole corpus inside the time budget.
      const int na = std::min<int>(28, pts_a.cols());
      const int nb = std::min<int>(28, pts_b.cols());
      const Matrix3Xd a_sub = pts_a.leftCols(na);
      const Matrix3Xd b_sub = pts_b.leftCols(nb);
      Matrix3Xd b_in_a_q(3, nb);
      Matrix3Xd b_in_a_qp(3, nb);
      plant.SetPositions(&ctx, q);
      plant.CalcPointsPositions(ctx, frame_b, b_sub, frame_a, &b_in_a_q);
      plant.SetPositions(&ctx, qp);
      plant.CalcPointsPositions(ctx, frame_b, b_sub, frame_a, &b_in_a_qp);
      for (int i = 0; i < na; ++i) {
        for (int j = 0; j < nb; ++j) {
          const double d_q = (a_sub.col(i) - b_in_a_q.col(j)).norm();
          const double d_qp = (a_sub.col(i) - b_in_a_qp.col(j)).norm();
          ASSERT_LE(std::abs(d_qp - d_q), bound + kSlack)
              << "aggregate: pair " << k << ", points (" << i << ", " << j
              << "), bound " << bound;
        }
      }
      ++stats->aggregate_checks;

      // ---- (3) One-sided aggregate when J(p) has a single distal side. ---
      if (single_distal_side && common_distal.is_valid()) {
        const Matrix3Xd& pts = (common_distal == pair.body_a) ? pts_a : pts_b;
        const auto& frame_d =
            (common_distal == pair.body_a) ? frame_a : frame_b;
        const auto& frame_o =
            (common_distal == pair.body_a) ? frame_b : frame_a;
        Matrix3Xd before(3, pts.cols());
        Matrix3Xd after(3, pts.cols());
        plant.SetPositions(&ctx, q);
        plant.CalcPointsPositions(ctx, frame_d, pts, frame_o, &before);
        plant.SetPositions(&ctx, qp);
        plant.CalcPointsPositions(ctx, frame_d, pts, frame_o, &after);
        const double displacement =
            (after - before).colwise().norm().maxCoeff();
        ASSERT_LE(displacement, bound + kSlack)
            << "one-sided aggregate: pair " << k << ", bound " << bound;
        stats->Observe(displacement, bound);
        ++stats->one_sided_checks;
      }
    }
  }
}

GTEST_TEST(DisplacementLemmaTest, RandomPlants) {
  Rng rng(0xD15B0);
  LemmaStats stats;
  constexpr int kNumPlants = 1500;
  for (int trial = 0; trial < kNumPlants; ++trial) {
    SCOPED_TRACE(fmt::format("random plant #{}", trial));
    // Screw joints in every third world. The carve-out cycles through its
    // three regimes so the exactly-constant and sub-tolerance cases each get
    // ~500 plants; in the sub-tolerance case the carved coordinates still move
    // and carveout_slack() has to pay for them.
    const RandomWorld world =
        MakeRandomWorld(&rng, /* allow_screw = */ trial % 3 == 0, 128);
    stats.screw_joints += world.num_screw_joints;
    const CarveOut carve_out = (trial % 3 == 1)   ? CarveOut::kExact
                               : (trial % 3 == 2) ? CarveOut::kSubTolerance
                                                  : CarveOut::kNone;
    CheckWorld(&rng, world, carve_out, &stats);
    if (HasFatalFailure()) return;
  }
  // Guard against the corpus silently degenerating into nothing.
  EXPECT_GE(stats.plants, 1000);
  EXPECT_GE(stats.pairs, 20000);
  EXPECT_GE(stats.atomic_checks, 20000);
  EXPECT_GE(stats.aggregate_checks, 10000);
  EXPECT_GE(stats.one_sided_checks, 2000);
  // Screw joints must actually appear in the corpus, or the screw λ rule is
  // never exercised.
  EXPECT_GT(stats.screw_joints, 0);
  // The bound must be near-tight somewhere, or this test would pass against an
  // arbitrarily wrong λ.
  EXPECT_GT(stats.max_tightness, 0.9);
  EXPECT_LE(stats.max_tightness, 1.0 + 1e-9);
  // The sub-tolerance third of the corpus must actually be charging residuals,
  // or the assertions above would be testing the exactly-constant case twice.
  EXPECT_GE(stats.slack_charged_pairs, 200);
  EXPECT_GT(stats.max_slack, 0.0);
  GTEST_LOG_(INFO) << fmt::format(
      "plants={} pairs={} atomic={} aggregate={} one_sided={} screw_joints={} "
      "max_tightness={:.6f} slack_pairs={} max_slack={:.3e}",
      stats.plants, stats.pairs, stats.atomic_checks, stats.aggregate_checks,
      stats.one_sided_checks, stats.screw_joints, stats.max_tightness,
      stats.slack_charged_pairs, stats.max_slack);
}

/* A dedicated screw-joint world, so the screw λ = r + |pitch|/2π rule is
 exercised densely rather than incidentally. */
GTEST_TEST(DisplacementLemmaTest, ScrewChain) {
  Rng rng(0x5C2E7);
  LemmaStats stats;
  for (int trial = 0; trial < 80; ++trial) {
    SCOPED_TRACE(fmt::format("screw world #{}", trial));
    RandomWorld world;
    RobotDiagramBuilder<double> builder;
    MultibodyPlant<double>& plant = builder.plant();
    std::vector<const RigidBody<double>*> bodies{&plant.world_body()};
    for (int i = 0; i < 3; ++i) {
      const auto& body =
          plant.AddRigidBody(fmt::format("b{}", i), UnitInertia());
      plant.AddJoint<ScrewJoint>(
          fmt::format("j{}", i), *bodies.back(), RandomTransform(&rng, 0.25),
          body, RandomTransform(&rng, 0.25), RandomUnitVector(&rng),
          Uniform(&rng, -0.8, 0.8), 0.0);
      bodies.push_back(&body);
    }
    AddRandomGeometry(&rng, &plant, plant.world_body(), "g_world", 128, &world);
    for (size_t i = 1; i < bodies.size(); ++i) {
      AddRandomGeometry(&rng, &plant, *bodies[i], fmt::format("g{}", i), 128,
                        &world);
    }
    world.diagram = builder.Build();
    CheckWorld(&rng, world, CarveOut::kNone, &stats);
    if (HasFatalFailure()) return;
  }
  EXPECT_GE(stats.plants, 75);
  EXPECT_GT(stats.atomic_checks, 500);
  EXPECT_GT(stats.max_tightness, 0.5);
  GTEST_LOG_(INFO) << fmt::format("screw: plants={} atomic={} tightness={:.6f}",
                                  stats.plants, stats.atomic_checks,
                                  stats.max_tightness);
}

// ---------------------------------------------------------------------------
// Part 3. The constant-coordinate carve-out's residual.
//
// The curve module flags a coordinate constant when its whole control-point
// range fits inside Options::continuity_tolerance. That is a tolerance, not an
// identity: such a coordinate is removed from every J(p) but may still move by
// up to its range, displacing the pair's distal side by λ̃·range. Uncharged,
// that residual would let the certificate inequality pass with the true
// clearance ~1e-7 m below threshold, two orders of magnitude above
// Options::certificate_slack. MotionBoundTable::carveout_slack() pays for it,
// and these tests pin it.
// ---------------------------------------------------------------------------

/* world --j_rot(revolute, ẑ)--> l1 --j_slide(prismatic, x̂)--> l2, with a
 sphere on the world and one offset out along l2. */
std::unique_ptr<RobotDiagram<double>> MakeCarveOutChain() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& l1 = plant.AddRigidBody("l1", UnitInertia());
  const auto& l2 = plant.AddRigidBody("l2", UnitInertia());
  plant.AddJoint<RevoluteJoint>("j_rot", plant.world_body(), {}, l1,
                                RigidTransform<double>(Vector3d(-0.2, 0, 0)),
                                Vector3d::UnitZ());
  plant.AddJoint<PrismaticJoint>("j_slide", l1,
                                 RigidTransform<double>(Vector3d(0.15, 0, 0)),
                                 l2, {}, Vector3d::UnitX());
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>::Identity(),
                                  Sphere(0.1), "g_world", mu);
  plant.RegisterCollisionGeometry(l2,
                                  RigidTransform<double>(Vector3d(0.3, 0, 0)),
                                  Sphere(0.05), "g_tip", mu);
  return builder.Build();
}

GTEST_TEST(CarveOutSlackTest, ToleranceConstantCoordinateIsChargedAtLambda) {
  auto diagram = MakeCarveOutChain();
  const MultibodyPlant<double>& plant = diagram->plant();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = plant.num_positions();
  ASSERT_EQ(nq, 2);
  const int rot = plant.GetJointByName("j_rot").position_start();
  const int slide = plant.GetJointByName("j_slide").position_start();

  // The slide's box is the same in every build below, so the reach, and with it
  // λ(j_rot), is identical throughout; a revolute λ does not depend on the
  // revolute's own box, which is the only thing that changes.
  VectorXd lower = VectorXd::Zero(nq);
  VectorXd upper = VectorXd::Zero(nq);
  lower[slide] = 0.1;
  upper[slide] = 0.4;

  // (a) Nothing carved: no residual at all, and λ(j_rot) is read off here.
  lower[rot] = -0.5;
  upper[rot] = 0.5;
  const MotionBoundTable moving = engine.ComputeMotionBoundTable(
      lower, upper, std::vector<bool>(nq, false), pairs);
  ASSERT_EQ(moving.GetEntries(0).size(), 2);
  EXPECT_EQ(moving.carveout_slack(0), 0.0);
  double lambda_rot = 0.0;
  for (const auto& [c, lam] : moving.GetEntries(0)) {
    if (c == rot) lambda_rot = lam;
  }
  ASSERT_GT(lambda_rot, 0.0);

  // (b) j_rot carved on the tolerance: it leaves J(p), and exactly λ·range
  //     takes its place in the slack.
  constexpr double kRange = 5e-8;
  static_assert(kRange <= kContinuityTolerance);
  std::vector<bool> constant(nq, false);
  constant[rot] = true;
  lower[rot] = 0.0;
  upper[rot] = kRange;  // upper − lower is exactly kRange in binary FP.
  const MotionBoundTable carved =
      engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
  ASSERT_EQ(carved.GetEntries(0).size(), 1);
  EXPECT_EQ(carved.GetEntries(0)[0].first, slide);
  const double expected = lambda_rot * kRange;
  EXPECT_NEAR(carved.carveout_slack(0), expected, 1e-15 * expected);
  // MotionBound() charges it unconditionally, on top of the CSR row.
  VectorXd w = VectorXd::Zero(nq);
  w[slide] = 0.02;
  EXPECT_DOUBLE_EQ(carved.MotionBound(0, w), carved.carveout_slack(0) + 0.02);

  // (c) Exactly constant: nothing to charge, bit for bit.
  lower[rot] = 0.0;
  upper[rot] = 0.0;
  const MotionBoundTable exact =
      engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
  EXPECT_EQ(exact.carveout_slack(0), 0.0);
  EXPECT_EQ(exact.MotionBound(0, w), 0.02);

  // (d) A *moving* coordinate never contributes to the slack, however wide.
  const MotionBoundTable wide = engine.ComputeMotionBoundTable(
      VectorXd::Constant(nq, -2.0), VectorXd::Constant(nq, 2.0),
      std::vector<bool>(nq, false), pairs);
  EXPECT_EQ(wide.carveout_slack(0), 0.0);
}

/* world --(base joint)--> link, with a HalfSpace on `link`, the distal side,
 and a sphere on the world. The base joint's kind is excluded, so the
 construction-time half-space rule, which knows only the supported rotational
 kinds, lets this model through; the carve-out is then the only thing between it
 and an unbounded λ̃. `rpy` picks a 6-dof rpy floating base over a 3-dof ball
 joint. */
std::unique_ptr<RobotDiagram<double>> MakeCarvedHalfSpaceModel(bool rpy) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& link = plant.AddRigidBody("link", UnitInertia());
  if (rpy) {
    plant.AddJoint<drake::multibody::RpyFloatingJoint>(
        "base", plant.world_body(), {}, link, {});
  } else {
    plant.AddJoint<drake::multibody::BallRpyJoint>("base", plant.world_body(),
                                                   {}, link, {});
  }
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(link, RigidTransform<double>::Identity(),
                                  HalfSpace(), "hs", mu);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>(Vector3d(0, 0, 1.0)),
                                  Sphere(0.1), "ball", mu);
  return builder.Build();
}

GTEST_TEST(CarveOutSlackTest, HalfSpaceAcrossAToleranceConstantRotationThrows) {
  // The unsound case the residual exposes: a half space has no finite reach,
  // so a rotational coordinate carrying it has no finite λ̃ and its residual
  // cannot be charged at all. Such a coordinate must be EXACTLY constant.
  auto diagram = MakeCarvedHalfSpaceModel(/* rpy = */ false);
  const KinematicsEngine engine(*diagram);  // Must not throw: it is not a
                                            // *supported* rotational kind.
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = diagram->plant().num_positions();
  ASSERT_EQ(nq, 3);

  VectorXd lower = VectorXd::Zero(nq);
  VectorXd upper = VectorXd::Constant(nq, 5e-8);
  try {
    engine.ComputeMotionBoundTable(lower, upper, std::vector<bool>(nq, true),
                                   pairs);
    GTEST_FAIL()
        << "expected a throw for a half space across a tolerance-constant "
           "rotational coordinate";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("hs"), std::string::npos) << what;
    EXPECT_NE(what.find("base"), std::string::npos) << what;
    EXPECT_NE(what.find("EXACTLY constant"), std::string::npos) << what;
  }
}

GTEST_TEST(CarveOutSlackTest,
           HalfSpaceAcrossAnExactlyConstantRotationIsAccepted) {
  auto diagram = MakeCarvedHalfSpaceModel(/* rpy = */ false);
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = diagram->plant().num_positions();
  const VectorXd pinned = VectorXd::Constant(nq, 0.3);
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      pinned, pinned, std::vector<bool>(nq, true), pairs);
  EXPECT_TRUE(table.pair_is_static(0));
  EXPECT_EQ(table.carveout_slack(0), 0.0);
}

GTEST_TEST(CarveOutSlackTest,
           HalfSpaceAcrossToleranceConstantTranslationIsAccepted) {
  // λ̃ = 1 for a translation coordinate is finite and correct even for a half
  // space (every point of it moves by |Δq|), so only the *rotational*
  // coordinates have to be exactly constant.
  auto diagram = MakeCarvedHalfSpaceModel(/* rpy = */ true);
  const MultibodyPlant<double>& plant = diagram->plant();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = plant.num_positions();
  ASSERT_EQ(nq, 6);  // q = (rpy, p_FM).

  constexpr double kRange = 4e-8;
  VectorXd lower = VectorXd::Zero(nq);
  VectorXd upper = VectorXd::Zero(nq);
  for (int c = 3; c < 6; ++c) upper[c] = kRange;  // Only the translation.
  const MotionBoundTable table = engine.ComputeMotionBoundTable(
      lower, upper, std::vector<bool>(nq, true), pairs);
  EXPECT_TRUE(table.pair_is_static(0));
  EXPECT_DOUBLE_EQ(table.carveout_slack(0), 3.0 * kRange);
}

// ---------------------------------------------------------------------------
// Part 3b. The residual of a floating base held constant on the tolerance.
//
// This is where λ̃ is not simply the λ the CSR row would have carried: these
// joint kinds are excluded and have no λ at all, only a carve-out λ̃. Each test
// drives Drake's own forward kinematics from configurations sampled inside the
// box, the carved base coordinates included, and checks the FK displacement
// against carveout_slack() directly: first with the arm coordinate pinned so
// the slack is the whole bound, then again with everything moving.
// ---------------------------------------------------------------------------

/* world --base(rpy or quaternion floating)--> b1 --jr(revolute ŷ)--> b2, with
 a sphere on the world and one offset out along b2. */
std::unique_ptr<RobotDiagram<double>> MakeFloatingBaseChain(Rng* rng,
                                                            bool quaternion) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& b1 = plant.AddRigidBody("b1", UnitInertia());
  const auto& b2 = plant.AddRigidBody("b2", UnitInertia());
  const RigidTransform<double> X_PF = RandomTransform(rng, 0.2);
  const RigidTransform<double> X_CM = RandomTransform(rng, 0.2);
  if (quaternion) {
    plant.AddJoint<QuaternionFloatingJoint>("base", plant.world_body(), X_PF,
                                            b1, X_CM);
  } else {
    plant.AddJoint<drake::multibody::RpyFloatingJoint>(
        "base", plant.world_body(), X_PF, b1, X_CM);
  }
  plant.AddJoint<RevoluteJoint>("jr", b1, RandomTransform(rng, 0.2), b2,
                                RandomTransform(rng, 0.2),
                                RandomUnitVector(rng));
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>::Identity(),
                                  Sphere(0.1), "g_world", mu);
  plant.RegisterCollisionGeometry(b2,
                                  RigidTransform<double>(Vector3d(0.25, 0, 0)),
                                  Sphere(0.06), "g_tip", mu);
  return builder.Build();
}

/* Shared body of the two floating-base property tests. `quaternion` picks the
 base parameterization; `seed` keeps the two corpora independent. */
void RunFloatingBaseCarveOutCorpus(bool quaternion, std::uint64_t seed) {
  Rng rng(seed);
  constexpr int kTrials = 250;
  int atomic_cases = 0;
  int aggregate_cases = 0;
  double max_ratio = 0.0;
  double max_slack = 0.0;

  for (int trial = 0; trial < kTrials; ++trial) {
    SCOPED_TRACE(fmt::format("floating-base carve-out #{}", trial));
    auto diagram = MakeFloatingBaseChain(&rng, quaternion);
    const MultibodyPlant<double>& plant = diagram->plant();
    const KinematicsEngine engine(*diagram);
    const std::vector<PairId> pairs = CollisionPairs(*diagram);
    ASSERT_EQ(pairs.size(), 1);
    const int nq = plant.num_positions();
    const int bs = plant.GetJointByName("base").position_start();
    const int nb = plant.GetJointByName("base").num_positions();
    const int jr = plant.GetJointByName("jr").position_start();
    ASSERT_EQ(nb, quaternion ? 7 : 6);

    // The base pose the trajectory holds. For the quaternion case it is a
    // random unit quaternion perturbed by at most the continuity tolerance,
    // exactly the box the curve module would flag constant. Drake normalizes
    // the quaternion internally, so a raw sample from that box and its
    // renormalization produce identical forward kinematics; sampling raw keeps
    // the sample inside the box the bound is stated over.
    VectorXd q0(nq);
    if (quaternion) {
      const Eigen::Quaterniond qb = RandomRotation(&rng).ToQuaternion();
      q0.segment<4>(bs) << qb.w(), qb.x(), qb.y(), qb.z();
      for (int i = 0; i < 3; ++i) q0[bs + 4 + i] = Uniform(&rng, -0.5, 0.5);
    } else {
      for (int i = 0; i < 3; ++i) q0[bs + i] = Uniform(&rng, -M_PI, M_PI);
      for (int i = 0; i < 3; ++i) q0[bs + 3 + i] = Uniform(&rng, -0.5, 0.5);
    }
    q0[jr] = Uniform(&rng, -1.0, 1.0);

    auto root = diagram->CreateDefaultContext();
    auto& ctx = plant.GetMyMutableContextFromRoot(root.get());
    const Matrix3Xd points_B =
        SampleSphereSurface(&rng, 0.06, 64).colwise() + Vector3d(0.25, 0, 0);
    const auto& frame_tip = plant.GetBodyByName("b2").body_frame();
    const auto& frame_world = plant.world_frame();
    Matrix3Xd out_q(3, points_B.cols());
    Matrix3Xd out_qp(3, points_B.cols());
    const auto displacement = [&](const VectorXd& q, const VectorXd& qp) {
      plant.SetPositions(&ctx, q);
      plant.CalcPointsPositions(ctx, frame_tip, points_B, frame_world, &out_q);
      plant.SetPositions(&ctx, qp);
      plant.CalcPointsPositions(ctx, frame_tip, points_B, frame_world, &out_qp);
      return (out_qp - out_q).colwise().norm().maxCoeff();
    };

    std::vector<bool> constant(nq, false);
    for (int c = bs; c < bs + nb; ++c) constant[c] = true;

    // ---- (A) Atomic: exactly one carved base coordinate has a width. -----
    // Every other base coordinate is exactly constant, so the pair's whole
    // slack is λ̃_c · range_c. The probe below drives that one coordinate from
    // one end of its interval to the other, pinning that single coefficient
    // rather than a seven-term sum, which is what makes the corpus sensitive to
    // an under-bound in any one λ̃.
    {
      const int c = bs + UniformInt(&rng, 0, nb - 1);
      const double width =
          Uniform(&rng, 0.2 * kContinuityTolerance, kContinuityTolerance);
      VectorXd lower = q0;
      VectorXd upper = q0;
      lower[c] = q0[c] - 0.5 * width;
      upper[c] = q0[c] + 0.5 * width;
      lower[jr] = q0[jr] - 0.8;
      upper[jr] = q0[jr] + 0.8;

      const MotionBoundTable table =
          engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
      ASSERT_EQ(table.GetEntries(0).size(), 1);  // Only the revolute survives.
      const double slack = table.carveout_slack(0);
      ASSERT_GT(slack, 0.0);
      ASSERT_LT(slack, 1e-4) << "a metre-scale reach against a 1e-7 box cannot "
                                "produce a residual this large";
      ++atomic_cases;
      max_slack = std::max(max_slack, slack);

      VectorXd q = q0;
      q[jr] = Uniform(&rng, lower[jr], upper[jr]);
      VectorXd qp = q;
      q[c] = lower[c];
      qp[c] = upper[c];
      const double atomic = displacement(q, qp);
      ASSERT_LE(atomic, slack + kSlack)
          << "atomic carve-out residual: coordinate " << c << " moved by "
          << (upper[c] - lower[c]) << ", displacement " << atomic << " > slack "
          << slack;
      max_ratio = std::max(max_ratio, atomic / slack);
    }

    // ---- (B) Aggregate: every base coordinate carved, everything moving. --
    {
      VectorXd lower = q0;
      VectorXd upper = q0;
      for (int c = bs; c < bs + nb; ++c) {
        const double half = 0.5 * Uniform(&rng, 0.2 * kContinuityTolerance,
                                          kContinuityTolerance);
        lower[c] = q0[c] - half;
        upper[c] = q0[c] + half;
      }
      lower[jr] = q0[jr] - 0.8;
      upper[jr] = q0[jr] + 0.8;

      const MotionBoundTable table =
          engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
      const double slack = table.carveout_slack(0);
      ASSERT_GT(slack, 0.0);
      max_slack = std::max(max_slack, slack);
      ++aggregate_cases;

      VectorXd q(nq);
      VectorXd qp(nq);
      for (int c = 0; c < nq; ++c) {
        q[c] = Uniform(&rng, lower[c], upper[c]);
        qp[c] = Uniform(&rng, lower[c], upper[c]);
      }
      // Σ_{uncarved} λ|Δq| + carveout_slack, with q and q′ drawn from the whole
      // box, the carved coordinates' tiny ranges included.
      const double full = displacement(q, qp);
      const double bound = table.MotionBound(0, (qp - q).cwiseAbs());
      ASSERT_LE(full, bound + kSlack)
          << "full bound: displacement " << full << " > bound " << bound;

      // ... and again with the revolute pinned, so the slack alone carries it.
      qp[jr] = q[jr];
      const double base_only = displacement(q, qp);
      ASSERT_LE(base_only, slack + kSlack)
          << "carved base residual: displacement " << base_only << " > slack "
          << slack;
    }
  }

  EXPECT_GE(atomic_cases, 200);
  EXPECT_GE(aggregate_cases, 200);
  // λ̃ must be near-tight somewhere, or these assertions would hold against an
  // arbitrarily inflated coefficient. (The dedicated tight model below pins
  // every coefficient individually; here the chain walk's own triangle
  // inequalities are slack at random poses, so only the translation rule
  // reaches 1.) The tolerance on the upper check is relative to a bound of
  // ~1e-7 m, where Drake's forward kinematics rounds at ~1e-15 m absolute.
  EXPECT_GT(max_ratio, 0.9);
  EXPECT_LE(max_ratio, 1.0 + 1e-6);
  GTEST_LOG_(INFO) << fmt::format(
      "{} base: atomic={} aggregate={} max_slack={:.3e} m max_ratio={:.6f}",
      quaternion ? "quaternion floating" : "rpy floating", atomic_cases,
      aggregate_cases, max_slack, max_ratio);
}

GTEST_TEST(CarveOutSlackTest, ToleranceConstantRpyFloatingBase) {
  RunFloatingBaseCarveOutCorpus(/* quaternion = */ false, 0x12F0BA5Eull);
}

GTEST_TEST(CarveOutSlackTest, ToleranceConstantQuaternionFloatingBase) {
  RunFloatingBaseCarveOutCorpus(/* quaternion = */ true, 0x9A7E48A5Eull);
}

// ---------------------------------------------------------------------------
// Part 3c. An exactly tight floating-base λ̃.
//
// The random corpus above catches structural errors but the chain walk's
// triangle inequalities are slack at random poses, so a λ̃ that is merely too
// small can hide inside that slack for the rotation rules. This model removes
// the slack, the way MakeTightChain() does for the supported kinds: both joint
// frames are identity, so the joint's M-frame origin is the link's body origin,
// and the link's single sphere is centred on it. The reach is then exactly R in
// every direction, so whatever axis a carved rotation coordinate turns the link
// about, a material point sits at the full reach perpendicular to that axis and
// the chord 2R·sin(θ/2) recovers R·θ to fifteen digits at θ ~ 1e-7. Every λ̃
// shows up digit for digit.
// ---------------------------------------------------------------------------

std::unique_ptr<RobotDiagram<double>> MakeTightFloatingChain(bool quaternion,
                                                             double radius) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const auto& link = plant.AddRigidBody("link", UnitInertia());
  if (quaternion) {
    plant.AddJoint<QuaternionFloatingJoint>("base", plant.world_body(), {},
                                            link, {});
  } else {
    plant.AddJoint<drake::multibody::RpyFloatingJoint>(
        "base", plant.world_body(), {}, link, {});
  }
  const CoulombFriction<double> mu(1.0, 1.0);
  plant.RegisterCollisionGeometry(link, RigidTransform<double>::Identity(),
                                  Sphere(radius), "g_link", mu);
  plant.RegisterCollisionGeometry(plant.world_body(),
                                  RigidTransform<double>(Vector3d(0, 0, 3.0)),
                                  Sphere(0.05), "g_world", mu);
  return builder.Build();
}

void RunTightFloatingBaseLambda(bool quaternion) {
  constexpr double kRadius = 0.4;
  Rng rng(quaternion ? 0x7168A7ull : 0x51DE12ull);
  auto diagram = MakeTightFloatingChain(quaternion, kRadius);
  const MultibodyPlant<double>& plant = diagram->plant();
  const KinematicsEngine engine(*diagram);
  const std::vector<PairId> pairs = CollisionPairs(*diagram);
  ASSERT_EQ(pairs.size(), 1);
  const int nq = plant.num_positions();
  const auto& base = plant.GetJointByName("base");
  const int bs = base.position_start();
  const int nb = base.num_positions();
  ASSERT_EQ(nb, quaternion ? 7 : 6);
  ASSERT_EQ(nq, nb);

  // Dense enough that some sample lands within ~1e-6 of the equator of any
  // rotation axis, which is what makes the chord recover R·θ.
  const Matrix3Xd points_B = SampleSphereSurface(&rng, kRadius, 4096);
  auto root = diagram->CreateDefaultContext();
  auto& ctx = plant.GetMyMutableContextFromRoot(root.get());
  Matrix3Xd out_q(3, points_B.cols());
  Matrix3Xd out_qp(3, points_B.cols());
  const auto& frame_link = plant.GetBodyByName("link").body_frame();
  const auto& frame_world = plant.world_frame();

  constexpr double kWidth = 8e-8;  // ≤ Options::continuity_tolerance.
  const std::vector<bool> constant(nq, true);

  for (int off = 0; off < nb; ++off) {
    SCOPED_TRACE(fmt::format("base coordinate offset {}", off));
    VectorXd q0 = VectorXd::Zero(nq);
    if (quaternion) {
      // A unit quaternion with a *zero* in the coordinate being perturbed, so
      // the perturbation is entirely orthogonal to it: normalization then
      // absorbs none of it and the induced rotation is the full 2‖Δq‖ that
      // λ̃ = 2r/m charges for. (A perturbation parallel to q induces no
      // rotation at all, which is why the bound has to be stated for the
      // worst case and cannot be tight in every direction at once.)
      Eigen::Vector4d qb(0.31, 0.53, -0.62, 0.49);
      if (off < 4) qb[off] = 0.0;
      qb.normalize();
      q0.head<4>() = qb;
      for (int i = 0; i < 3; ++i) q0[4 + i] = Uniform(&rng, -0.5, 0.5);
    } else {
      // rpy = 0: each angle then turns the link about a coordinate axis
      // through Mo, and the sphere is centred there.
      for (int i = 0; i < 3; ++i) q0[3 + i] = Uniform(&rng, -0.5, 0.5);
    }

    VectorXd lower = q0;
    VectorXd upper = q0;
    lower[bs + off] = q0[bs + off] - 0.5 * kWidth;
    upper[bs + off] = q0[bs + off] + 0.5 * kWidth;
    const MotionBoundTable table =
        engine.ComputeMotionBoundTable(lower, upper, constant, pairs);
    ASSERT_TRUE(table.pair_is_static(0));
    const double slack = table.carveout_slack(0);
    ASSERT_GT(slack, 0.0);

    VectorXd q = lower;
    VectorXd qp = upper;
    plant.SetPositions(&ctx, q);
    plant.CalcPointsPositions(ctx, frame_link, points_B, frame_world, &out_q);
    plant.SetPositions(&ctx, qp);
    plant.CalcPointsPositions(ctx, frame_link, points_B, frame_world, &out_qp);
    const double displacement = (out_qp - out_q).colwise().norm().maxCoeff();

    ASSERT_LE(displacement, slack + kSlack)
        << "displacement " << displacement << " > slack " << slack;
    EXPECT_GT(displacement / slack, 0.999)
        << "the residual must be exactly attained on this model; a slack ratio "
           "here would mean λ̃ is over-counted, and a violated bound would mean "
           "it is under-counted. displacement "
        << displacement << ", slack " << slack;
  }
}

GTEST_TEST(CarveOutSlackTest, RpyFloatingLambdaTildeIsExactAndTight) {
  RunTightFloatingBaseLambda(/* quaternion = */ false);
}

GTEST_TEST(CarveOutSlackTest, QuaternionFloatingLambdaTildeIsExactAndTight) {
  RunTightFloatingBaseLambda(/* quaternion = */ true);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
