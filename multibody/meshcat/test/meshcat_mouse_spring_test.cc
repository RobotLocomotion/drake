#include "drake/multibody/meshcat/meshcat_mouse_spring.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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
  MeshcatMouseSpringTest() : meshcat_(geometry::GetTestEnvironmentMeshcat()) {
    // Build a plant with a free "ball" body in the default model instance and a
    // free "link" body inside a named "robot" model instance. The two bodies
    // have distinct (non-unit) masses so that the mass-scaling of the spring
    // force is exercised.
    ball_ = &plant_.AddRigidBody(
        "ball", SpatialInertia<double>::SolidSphereWithMass(kBallMass, 0.1));
    const ModelInstanceIndex robot = plant_.AddModelInstance("robot");
    link_ = &plant_.AddRigidBody(
        "link", robot,
        SpatialInertia<double>::SolidSphereWithMass(kLinkMass, 0.1));
    plant_.Finalize();
  }

  // The test-environment Meshcat is a shared singleton, so clear any drag state
  // left over from a previous test before each test runs.
  void SetUp() override {
    meshcat_->InjectWebsocketMessage(PackDrag("", {}, {}));
    ASSERT_FALSE(meshcat_->GetObjectDrag().has_value());
  }

  // Returns the forces computed by a freshly built spring, given the current
  // meshcat drag state and the provided body poses/velocities.
  std::vector<ExternallyAppliedSpatialForce<double>> CalcForces(
      const std::vector<RigidTransformd>& X_WB,
      const std::vector<SpatialVelocity<double>>& V_WB) {
    MeshcatMouseSpring spring(meshcat_, &plant_, kStiffness);
    auto context = spring.CreateDefaultContext();
    spring.get_body_poses_input_port().FixValue(context.get(), X_WB);
    spring.get_body_spatial_velocities_input_port().FixValue(context.get(),
                                                             V_WB);
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

  static constexpr double kStiffness = 100.0;  // 1/s².
  static constexpr double kBallMass = 2.0;     // kg.
  static constexpr double kLinkMass = 3.0;     // kg.

  std::shared_ptr<Meshcat> meshcat_;
  MultibodyPlant<double> plant_{0.0};
  const RigidBody<double>* ball_{};
  const RigidBody<double>* link_{};
};

// With no active drag, the output force vector is empty.
TEST_F(MeshcatMouseSpringTest, NoDrag) {
  EXPECT_FALSE(meshcat_->GetObjectDrag().has_value());
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

  const auto drag = meshcat_->GetObjectDrag();
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

// A drag on a model-instance-scoped body resolves the "model::body" -> body
// mapping by matching the scoped frame name as a run of path segments,
// independent of the visualization-layer prefix and any surrounding geometry
// path. This mirrors the real paths the browser sends for the different
// MeshcatVisualizer layers (illustration / proximity / inertia).
TEST_F(MeshcatMouseSpringTest, DragScopedBodyAnyPrefix) {
  const Vector3d target(0.0, 0.0, 1.0);
  for (const std::string& path : {
           // Illustration layer.
           std::string("/drake/illustration/robot/link/some_geometry"),
           // Proximity layer (geometry name repeats the model scope).
           std::string("/drake/proximity/robot/link/robot/Mesh"),
           // Inertia layer (an extra "InertiaVisualizer" node is inserted).
           std::string("/drake/inertia/InertiaVisualizer/robot/link/$inertia"),
       }) {
    SetUp();  // Clear any prior drag.
    meshcat_->InjectWebsocketMessage(PackDrag(path, {0, 0, 0}, {0, 0, 1}));
    const auto forces = CalcForces(DefaultPoses(), ZeroVelocities());
    SCOPED_TRACE(path);
    ASSERT_EQ(forces.size(), 1);
    EXPECT_EQ(forces[0].body_index, link_->index());
    EXPECT_TRUE(CompareMatrices(forces[0].F_Bq_W.translational(),
                                kLinkMass * kStiffness * target, 1e-12));
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
  ASSERT_TRUE(meshcat_->GetObjectDrag().has_value());
  meshcat_->InjectWebsocketMessage(PackDrag("", {}, {}));
  EXPECT_FALSE(meshcat_->GetObjectDrag().has_value());
  EXPECT_TRUE(CalcForces(DefaultPoses(), ZeroVelocities()).empty());
}

// Construction precondition checks.
TEST_F(MeshcatMouseSpringTest, ConstructorErrors) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MeshcatMouseSpring(meshcat_, &plant_, -1.0 /* stiffness */),
      ".*stiffness.*");
  MultibodyPlant<double> unfinalized(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatMouseSpring(meshcat_, &unfinalized),
                              ".*is_finalized.*");
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
