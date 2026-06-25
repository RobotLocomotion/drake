#include "drake/multibody/meshcat/meshcat_mouse_spring.h"

#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using Eigen::Vector3d;
using geometry::Meshcat;
using geometry::SceneGraph;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::Context;

// Packs an injected websocket "mouse_drag" message in the same wire format that
// meshcat.html produces.
std::string PackDrag(const std::string& name, const std::vector<double>& anchor,
                     const std::vector<double>& target) {
  std::stringstream ss;
  msgpack::packer<std::stringstream> o(ss);
  o.pack_map(4);
  o.pack(std::string("type"));
  o.pack(std::string("mouse_drag"));
  o.pack(std::string("name"));
  o.pack(name);
  o.pack(std::string("drag_anchor"));
  o.pack(anchor);
  o.pack(std::string("drag_target"));
  o.pack(target);
  return ss.str();
}

class MeshcatMouseSpringTest : public ::testing::Test {
 protected:
  // Build a plant with a free "ball" body in the default model instance plus a
  // free "link" and a second "ball" inside a named "robot" model instance. The
  // bodies have distinct (non-unit) masses so that the mass-scaling of the
  // spring force is exercised, and the two "ball" bodies (scoped "ball" and
  // "robot/ball") let us verify longest-match body identification.
  MeshcatMouseSpringTest()
      : meshcat_(geometry::GetTestEnvironmentMeshcat()),
        plant_(0.0),
        robot_(plant_.AddModelInstance("robot")),
        ball_(&plant_.AddRigidBody(
            "ball", default_model_instance(),
            SpatialInertia<double>::SolidSphereWithMass(kBallMass, 0.1))),
        link_(&plant_.AddRigidBody(
            "link", robot_,
            SpatialInertia<double>::SolidSphereWithMass(kLinkMass, 0.1))),
        robot_ball_(&plant_.AddRigidBody(
            "ball", robot_,
            SpatialInertia<double>::SolidSphereWithMass(kRobotBallMass, 0.1))) {
    // Registering the plant as a geometry source gives every body a SceneGraph
    // frame named by its scoped name; MeshcatMouseSpring reads those names to
    // map dragged Meshcat paths back to bodies.
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
    plant_.Finalize();
  }

  // The test-environment Meshcat is a shared singleton, so clear any drag state
  // left over from a previous test before each test runs.
  void SetUp() override {
    meshcat_->InjectWebsocketMessage(PackDrag("", {}, {}));
    ASSERT_FALSE(meshcat_->GetVirtualSpringKinematics().has_value());
  }

  // Returns the forces computed by a freshly built spring, given the current
  // meshcat drag state and the provided body poses/velocities.
  std::vector<ExternallyAppliedSpatialForce<double>> CalcForces(
      const std::vector<RigidTransformd>& X_WBs,
      const std::vector<SpatialVelocity<double>>& V_WBs) {
    MeshcatMouseSpring spring(meshcat_, &plant_, scene_graph_, kStiffness);
    auto context = spring.CreateDefaultContext();
    spring.get_body_poses_input_port().FixValue(context.get(), X_WBs);
    spring.get_body_spatial_velocities_input_port().FixValue(context.get(),
                                                             V_WBs);
    return spring.get_spatial_forces_output_port()
        .Eval<std::vector<ExternallyAppliedSpatialForce<double>>>(*context);
  }

  std::vector<RigidTransformd> DefaultPoses() const {
    return std::vector<RigidTransformd>(plant_.num_bodies(),
                                        RigidTransformd::Identity());
  }
  std::vector<SpatialVelocity<double>> ZeroVelocities() const {
    return std::vector<SpatialVelocity<double>>(
        plant_.num_bodies(), SpatialVelocity<double>::Zero());
  }

  static constexpr double kStiffness = 100.0;    // 1/s².
  static constexpr double kBallMass = 2.0;       // kg.
  static constexpr double kLinkMass = 3.0;       // kg.
  static constexpr double kRobotBallMass = 5.0;  // kg.

  std::shared_ptr<Meshcat> meshcat_;
  SceneGraph<double> scene_graph_;
  MultibodyPlant<double> plant_;
  ModelInstanceIndex robot_;
  const RigidBody<double>* ball_;
  const RigidBody<double>* link_;
  const RigidBody<double>* robot_ball_;
};

// With no active drag, the output force vector is empty.
TEST_F(MeshcatMouseSpringTest, NoDrag) {
  EXPECT_FALSE(meshcat_->GetVirtualSpringKinematics().has_value());
  EXPECT_TRUE(CalcForces(DefaultPoses(), ZeroVelocities()).empty());
}

// A drag on the default-instance "ball" body produces a spring force at the
// expected body point, with the expected magnitude.
TEST_F(MeshcatMouseSpringTest, DragBall) {
  // Place the ball at a non-identity pose so the inverse transform matters.
  const RigidTransformd X_WB(RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
  auto poses = DefaultPoses();
  poses[ball_->index()] = X_WB;

  const Vector3d anchor(1.2, 2.0, 3.1);  // A point near the ball, in world.
  const Vector3d target(1.5, 2.0, 3.1);  // The cursor target, in world.
  meshcat_->InjectWebsocketMessage(
      PackDrag("/drake/visualizer/ball", {anchor.x(), anchor.y(), anchor.z()},
               {target.x(), target.y(), target.z()}));

  const auto drag = meshcat_->GetVirtualSpringKinematics();
  ASSERT_TRUE(drag.has_value());
  EXPECT_EQ(drag->path, "/drake/visualizer/ball");

  const auto forces = CalcForces(poses, ZeroVelocities());
  ASSERT_EQ(forces.size(), 1);
  EXPECT_EQ(forces[0].body_index, ball_->index());
  // The application point, in the body frame.
  EXPECT_TRUE(
      CompareMatrices(forces[0].p_BoBq_B, X_WB.inverse() * anchor, 1e-12));
  // Zero velocity => pure mass-scaled spring force mass*k*(target - anchor),
  // no torque.
  EXPECT_TRUE(CompareMatrices(forces[0].F_Bq_W.translational(),
                              kBallMass * kStiffness * (target - anchor),
                              1e-12));
  EXPECT_TRUE(
      CompareMatrices(forces[0].F_Bq_W.rotational(), Vector3d::Zero(), 1e-12));
}

// Body identification: a dragged Meshcat path resolves to the plant body whose
// scoped scene-tree name it contains, delimited by '/' (or the end of the path)
// on both sides. This exercises the full pipeline (SceneGraph frame names ->
// path map -> matching) via CalcForces, covering prefix independence, the
// longest-match preference, a trailing slash, and a variety of non-matches.
TEST_F(MeshcatMouseSpringTest, DragScopedBodyAnyPrefix) {
  struct Case {
    std::string path;
    // The body the drag should resolve to, or nullopt for "no match".
    std::optional<BodyIndex> expected;
  };
  const std::vector<Case> cases = {
      // "robot/link" matches regardless of the visualization-layer prefix and
      // any trailing geometry path -- mirroring the real paths the browser
      // sends for the different MeshcatVisualizer layers: illustration...
      {"/drake/illustration/robot/link/some_geometry", link_->index()},
      // ...proximity (whose geometry name repeats the model scope)...
      {"/drake/proximity/robot/link/robot/Mesh", link_->index()},
      // ...and inertia (which inserts an extra "InertiaVisualizer" node).
      {"/drake/inertia/InertiaVisualizer/robot/link/$inertia", link_->index()},
      // The prefix is irrelevant, even one that no Drake code would produce.
      {"/some/external/viewer/robot/link", link_->index()},

      // A trailing '/' still delimits the segment (not a path we expect to
      // receive, but the match should succeed).
      {"/drake/foo/ball/", ball_->index()},

      // Longest-match preference: "/robot/ball" contains both "ball" and the
      // more-specific "robot/ball"; the longer segment wins.
      {"/drake/foo/robot/ball", robot_ball_->index()},
      // A different model instance ("robot2") doesn't match "robot/ball", so
      // only the default-instance "ball" matches.
      {"/drake/foo/robot2/ball", ball_->index()},

      // No substring matching: the stored segment must be bounded by '/' (or
      // the end of the path) on both sides.
      {"/drake/foo/robot/link2", std::nullopt},      // "link2" != "link".
      {"/drake/foo/some_robot/link", std::nullopt},  // "some_robot" != "robot".
      {"/drake/foo/not_a_body", std::nullopt},
  };

  for (const Case& c : cases) {
    SCOPED_TRACE(c.path);
    SetUp();  // Clear any prior drag.
    meshcat_->InjectWebsocketMessage(PackDrag(c.path, {0, 0, 0}, {0, 0, 1}));
    const auto forces = CalcForces(DefaultPoses(), ZeroVelocities());
    if (c.expected.has_value()) {
      ASSERT_EQ(forces.size(), 1);
      EXPECT_EQ(forces[0].body_index, *c.expected);
    } else {
      EXPECT_TRUE(forces.empty());
    }
  }
}

// Damping opposes the velocity of the attachment point, scaled by mass and
// sqrt(stiffness).
TEST_F(MeshcatMouseSpringTest, Damping) {
  auto poses = DefaultPoses();  // ball at identity
  meshcat_->InjectWebsocketMessage(
      PackDrag("/drake/visualizer/ball", {0, 0, 0}, {0, 0, 0}));
  // Give the ball a pure translational velocity; the anchor is at Bo so the
  // point velocity equals the body's translational velocity.
  auto vels = ZeroVelocities();
  const Vector3d v_WB(0.5, 0.0, 0.0);
  vels[ball_->index()] = SpatialVelocity<double>(Vector3d::Zero(), v_WB);

  const auto forces = CalcForces(poses, vels);
  ASSERT_EQ(forces.size(), 1);
  const double expected_damping = kBallMass * std::sqrt(kStiffness);
  EXPECT_TRUE(CompareMatrices(forces[0].F_Bq_W.translational(),
                              -expected_damping * v_WB, 1e-12));
}

// A drag on a path that doesn't correspond to any plant body (or the world
// body) produces no force.
TEST_F(MeshcatMouseSpringTest, UnknownPath) {
  meshcat_->InjectWebsocketMessage(
      PackDrag("/drake/visualizer/not_a_body", {0, 0, 0}, {0, 0, 1}));
  EXPECT_TRUE(CalcForces(DefaultPoses(), ZeroVelocities()).empty());

  // The world body is never draggable.
  meshcat_->InjectWebsocketMessage(
      PackDrag("/drake/visualizer", {0, 0, 0}, {0, 0, 1}));
  EXPECT_TRUE(CalcForces(DefaultPoses(), ZeroVelocities()).empty());
}

// An empty payload clears the drag.
TEST_F(MeshcatMouseSpringTest, DragEnd) {
  meshcat_->InjectWebsocketMessage(
      PackDrag("/drake/visualizer/ball", {0, 0, 0}, {0, 0, 1}));
  ASSERT_TRUE(meshcat_->GetVirtualSpringKinematics().has_value());
  meshcat_->InjectWebsocketMessage(PackDrag("", {}, {}));
  EXPECT_FALSE(meshcat_->GetVirtualSpringKinematics().has_value());
  EXPECT_TRUE(CalcForces(DefaultPoses(), ZeroVelocities()).empty());
}

// Construction precondition checks.
TEST_F(MeshcatMouseSpringTest, ConstructorErrors) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MeshcatMouseSpring(meshcat_, &plant_, scene_graph_, -1.0 /* stiffness */),
      ".*stiffness.*");
  MultibodyPlant<double> unfinalized(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MeshcatMouseSpring(meshcat_, &unfinalized, scene_graph_),
      ".*is_finalized.*");
  // A finalized plant that was never registered with a SceneGraph has no body
  // frames to read.
  MultibodyPlant<double> no_scene_graph(0.0);
  no_scene_graph.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MeshcatMouseSpring(meshcat_, &no_scene_graph, scene_graph_),
      ".*geometry_source_is_registered.*");
}

// When a shorter segment ("body") and a longer, more-specific one ("m1/body")
// both match a path, the longest wins.
GTEST_TEST(FindBodyForPathTest, PrefersLongestSegment) {
  const BodyIndex unscoped(1);
  const BodyIndex scoped(2);
  const std::map<std::string, BodyIndex> path_to_body = {
      {"body", unscoped},
      {"m1/body", scoped},
  };

  // Both "body" and "m1/body" match here; the longer one is chosen.
  EXPECT_EQ(internal::FindBodyForPath(path_to_body, "/drake/foo/m1/body"),
            scoped);
  // Only the shorter segment is present here.
  EXPECT_EQ(internal::FindBodyForPath(path_to_body, "/drake/foo/body"),
            unscoped);
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
