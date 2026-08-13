#include "drake/multibody/meshcat/meshcat_mouse_spring.h"

#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

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

// Grants the tests below access to the force computation for dummy drag
// scenarios.
class MeshcatMouseSpringTester {
 public:
  static std::vector<ExternallyAppliedSpatialForce<double>>
  CalcAppliedSpatialForce(
      const MeshcatMouseSpring& spring,
      const std::optional<geometry::Meshcat::VirtualSpringKinematics>& drag,
      const std::vector<math::RigidTransform<double>>& X_WB_all,
      const std::vector<SpatialVelocity<double>>& V_WB_all) {
    return spring.CalcAppliedSpatialForceFromDrag(drag, X_WB_all, V_WB_all);
  }
};

namespace {

using Eigen::Vector3d;
using geometry::Meshcat;
using geometry::SceneGraph;
using math::RigidTransformd;
using math::RollPitchYawd;

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

  // Returns the forces computed by a freshly built spring, given the drag state
  // and the provided body poses/velocities.
  std::vector<ExternallyAppliedSpatialForce<double>> CalcForces(
      const std::optional<Meshcat::VirtualSpringKinematics>& drag,
      const std::vector<RigidTransformd>& X_WBs,
      const std::vector<SpatialVelocity<double>>& V_WBs) {
    MeshcatMouseSpring spring(meshcat_, &plant_, scene_graph_, kStiffness);
    return MeshcatMouseSpringTester::CalcAppliedSpatialForce(spring, drag,
                                                             X_WBs, V_WBs);
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
  const RigidBody<double>* const ball_;
  const RigidBody<double>* const link_;
  const RigidBody<double>* const robot_ball_;
};

// With no active drag, the output force vector is empty.
TEST_F(MeshcatMouseSpringTest, NoDrag) {
  EXPECT_TRUE(
      CalcForces(std::nullopt, DefaultPoses(), ZeroVelocities()).empty());
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
  const Meshcat::VirtualSpringKinematics drag{.path = "/drake/visualizer/ball",
                                              .body_point_in_world = anchor,
                                              .target_point_in_world = target};

  const auto forces = CalcForces(drag, poses, ZeroVelocities());
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
    const Meshcat::VirtualSpringKinematics drag{
        .path = c.path,
        .body_point_in_world = Vector3d::Zero(),
        .target_point_in_world = Vector3d::UnitZ()};
    const auto forces = CalcForces(drag, DefaultPoses(), ZeroVelocities());
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
  // With the ball at identity and the anchor at Bo, the anchor's velocity
  // equals the body's translational velocity.
  const Meshcat::VirtualSpringKinematics drag{
      .path = "/drake/visualizer/ball",
      .body_point_in_world = Vector3d::Zero(),
      .target_point_in_world = Vector3d::Zero()};
  auto vels = ZeroVelocities();
  const Vector3d v_WB(0.5, 0.0, 0.0);
  vels[ball_->index()] = SpatialVelocity<double>(Vector3d::Zero(), v_WB);

  const auto forces = CalcForces(drag, DefaultPoses(), vels);
  ASSERT_EQ(forces.size(), 1);
  const double expected_damping = kBallMass * std::sqrt(kStiffness);
  EXPECT_TRUE(CompareMatrices(forces[0].F_Bq_W.translational(),
                              -expected_damping * v_WB, 1e-12));
}

// The output port reports no force when Meshcat reports no drag. (The rest of
// the port's calculation is the force computation tested above.)
TEST_F(MeshcatMouseSpringTest, OutputPort) {
  ASSERT_FALSE(meshcat_->GetVirtualSpringKinematics().has_value());
  MeshcatMouseSpring spring(meshcat_, &plant_, scene_graph_, kStiffness);
  auto context = spring.CreateDefaultContext();
  spring.get_body_poses_input_port().FixValue(context.get(), DefaultPoses());
  spring.get_body_spatial_velocities_input_port().FixValue(context.get(),
                                                           ZeroVelocities());
  EXPECT_TRUE(
      spring.get_applied_spatial_force_output_port()
          .Eval<std::vector<ExternallyAppliedSpatialForce<double>>>(*context)
          .empty());
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
  // Neither segment matches here.
  EXPECT_EQ(internal::FindBodyForPath(path_to_body, "/drake/foo/m1"),
            std::nullopt);
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
