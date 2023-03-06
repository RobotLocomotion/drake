#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/meshcat/contact_visualizer.h"

namespace drake {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using systems::Context;
using systems::Simulator;
using geometry::Meshcat;
using geometry::MeshcatVisualizerParams;
using geometry::MeshcatVisualizer;

namespace multibody {

using meshcat::ContactVisualizer;

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static const std::vector<int>& EvalJointLockingIndices(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalJointLockingIndices(context);
  }
};

namespace {

const double kTimestep = 0.01;
const double kElbowPosition = 0.3;
const double kArmLength = 0.1;

// Set up a plant with 2 trees, one tree having a single floating body, the
// second tree a serial chain of two bodies attached to each other and world by
// revolute joints.
class JointLockingTest : public ::testing::TestWithParam<int> {
 public:
  void SetUp() {
    plant_ = std::make_unique<MultibodyPlant<double>>(kTimestep);

    // Each permutation of adding trees (children of world body) to the plant.
    switch (GetParam()) {
      case 0:
        AddFloatingPendulum();
        AddDoublePendulum();
        break;
      case 1:
        AddDoublePendulum();
        AddFloatingPendulum();
        break;
    }

    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;

 private:
  void AddFloatingPendulum() {
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    plant_->AddRigidBody("body1", SpatialInertia<double>::MakeUnitary());
    plant_->AddRigidBody("body2", SpatialInertia<double>::MakeUnitary());

    std::unique_ptr<RevoluteJoint<double>> body1_body2 =
        std::make_unique<RevoluteJoint<double>>(
            "body1_body2", plant_->GetRigidBodyByName("body1").body_frame(),
            plant_->GetRigidBodyByName("body2").body_frame(),
            Vector3d::UnitZ());
    plant_->AddJoint(std::move(body1_body2));
  }

  void AddDoublePendulum() {
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    plant_->AddRigidBody("body3", SpatialInertia<double>::MakeUnitary());
    plant_->AddRigidBody("body4", SpatialInertia<double>::MakeUnitary());

    std::unique_ptr<RevoluteJoint<double>> world_body3 =
        std::make_unique<RevoluteJoint<double>>(
            "world_body3", plant_->world_frame(),
            plant_->GetRigidBodyByName("body3").body_frame(),
            Vector3d::UnitZ());
    std::unique_ptr<RevoluteJoint<double>> body3_body4 =
        std::make_unique<RevoluteJoint<double>>(
            "body3_body4", plant_->GetRigidBodyByName("body3").body_frame(),
            plant_->GetRigidBodyByName("body4").body_frame(),
            Vector3d::UnitZ());

    plant_->AddJoint(std::move(world_body3));
    plant_->AddJoint(std::move(body3_body4));
  }
};

// Test that regardless of the order the joints were added to the plant (i.e.
// different joint indexing orders), the established joint locking velocity
// indexing remains correct.
TEST_P(JointLockingTest, JointLockingIndicesTest) {
  const RigidBody<double>& body1 = plant_->GetRigidBodyByName("body1");

  const RevoluteJoint<double>& body1_body2 =
      plant_->GetJointByName<RevoluteJoint>("body1_body2");
  const RevoluteJoint<double>& world_body3 =
      plant_->GetJointByName<RevoluteJoint>("world_body3");
  RevoluteJoint<double>& body3_body4 =
      plant_->GetMutableJointByName<RevoluteJoint>("body3_body4");

  const int body1_velocity_start =
      body1.floating_velocities_start() - plant_->num_positions();

  // No joints/bodies are locked, all joint/body velocity indices should exist.
  {
    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 9);

    const std::vector<int>& expected_unlocked_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }

  // Lock body3_body4 and re-evaluate joint locking indices
  {
    body3_body4.Lock(context_.get());
    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 8);

    const std::vector<int>& expected_unlocked_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }

  // Unlock body3_body4 and lock body1 and re-evaluate joint locking indices.
  {
    body3_body4.Unlock(context_.get());
    body1.Lock(context_.get());

    const std::vector<int>& unlocked_indices =
        MultibodyPlantTester::EvalJointLockingIndices(*plant_, *context_);

    EXPECT_EQ(unlocked_indices.size(), 3);

    const std::vector<int>& expected_unlocked_indices = {
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};

    EXPECT_THAT(unlocked_indices,
                testing::UnorderedElementsAreArray(expected_unlocked_indices));
  }
}

INSTANTIATE_TEST_SUITE_P(IndexPermutations, JointLockingTest,
                         ::testing::Values(0, 1));

// Create a plant with a XZ-planar double pendulum where the masses are
// concentrated at the lower ends of the two links. The 0-configuration has the
// upper arm sticking out horizontally at (-kArmLength, 0, 0) and the lower
// arm placed at (-kArmLength, 0, 0.0) relative to the upper arm's frame. If the
// `weld_elbow` parameter is true, then the revolute joint between bodies
// `upper_arm` and `lower_arm` is replaced with a fixed weld corresponding to
// the configuration (0, kElbowPosition) in the model with two joints.
std::unique_ptr<MultibodyPlant<double>> MakeDoublePendulumPlant(
    bool weld_elbow) {
  std::unique_ptr<MultibodyPlant<double>> plant;
  plant = std::make_unique<MultibodyPlant<double>>(kTimestep);

  const RigidBody<double>& body1 =
      plant->AddRigidBody("upper_arm", SpatialInertia<double>::MakeUnitary());
  const RigidBody<double>& body2 =
      plant->AddRigidBody("lower_arm", SpatialInertia<double>::MakeUnitary());

  plant->AddJoint<RevoluteJoint>("shoulder", plant->world_body(), {}, body1,
                                 RigidTransformd(Vector3d(kArmLength, 0, 0)),
                                 Vector3d::UnitY());

  if (weld_elbow) {
    plant->WeldFrames(
        body1.body_frame(), body2.body_frame(),
        RigidTransformd(Vector3d(-kArmLength * cos(kElbowPosition), 0.0,
                                  kArmLength * sin(kElbowPosition))));
  } else {
    plant->AddJoint<RevoluteJoint>(
        "elbow", body1, {}, body2,
        RigidTransformd(Vector3d(kArmLength, 0, 0.0)), Vector3d::UnitY());
  }
  plant->Finalize();
  return plant;
}

// To verify that the physical behavior of a locked joint is identical to a weld
// joint, we construct two plants. Each plant consists of a double pendulum with
// the shoulder welded to the world. The elbow joint of `plant_welded` is
// replaced with a WeldJoint fixed at configuration (0, kElbowPosition). The
// elbow joint of `plant_locked` is a revolute joint that has been locked at the
// configuration (0, kElbowPosition). We verify that the generalized
// accelerations, velocities and positions match to a given accuracy at each
// timestep.
GTEST_TEST(JointLockingTest, TrajectoryTest) {
  // Allow 2 digits of precision loss to account for roundoff differences
  // between the two code paths. This value was determined empircally by
  // observing the maximum error between the two trajectories.
  const double kEps = 1e2 * std::numeric_limits<double>::epsilon();
  const int kNumTimesteps = 100;
  auto plant_welded = MakeDoublePendulumPlant(true);
  auto plant_locked = MakeDoublePendulumPlant(false);

  std::unique_ptr<systems::Context<double>> context_welded =
      plant_welded->CreateDefaultContext();
  std::unique_ptr<systems::Context<double>> context_locked =
      plant_locked->CreateDefaultContext();

  // Lock the elbow in the unwelded plant.
  const RevoluteJoint<double>& elbow =
      plant_locked->GetJointByName<RevoluteJoint>("elbow");
  elbow.set_angle(context_locked.get(), kElbowPosition);
  elbow.Lock(context_locked.get());

  // Sanity check.
  ASSERT_EQ(plant_welded->num_velocities(), 1);
  ASSERT_EQ(plant_locked->num_velocities(), 2);

  auto simulator_welded = std::make_unique<Simulator<double>>(
      *plant_welded, std::move(context_welded));
  auto simulator_locked = std::make_unique<Simulator<double>>(
      *plant_locked, std::move(context_locked));
  simulator_welded->Initialize();
  simulator_locked->Initialize();

  // Simulate for kNumTimesteps and verify acceleration, velocity, and positions
  // at each iteration.
  for (int i = 1; i <= kNumTimesteps; ++i) {
    simulator_welded->AdvanceTo(i * kTimestep);
    simulator_locked->AdvanceTo(i * kTimestep);

    const auto& welded_context =
        plant_welded->GetMyContextFromRoot(simulator_welded->get_context());
    const auto& locked_context =
        plant_locked->GetMyContextFromRoot(simulator_locked->get_context());

    int shoulder_index_welded =
        plant_welded->GetJointByName<RevoluteJoint>("shoulder")
            .velocity_start();
    int shoulder_index_locked =
        plant_locked->GetJointByName<RevoluteJoint>("shoulder")
            .velocity_start();

    // The welded plant has only one dof, shoulder. The acceleration of each
    // plant at the corresponding dof should match.
    const auto welded_accelerations =
        plant_welded->get_generalized_acceleration_output_port().Eval(
            welded_context);
    const auto locked_accelerations =
        plant_locked->get_generalized_acceleration_output_port().Eval(
            locked_context);
    EXPECT_NEAR(welded_accelerations[shoulder_index_welded],
                locked_accelerations[shoulder_index_locked], kEps);
    // The locked elbow dof should have 0 acceleration.
    int elbow_index_locked =
        plant_locked->GetJointByName<RevoluteJoint>("elbow").velocity_start();
    EXPECT_EQ(locked_accelerations[elbow_index_locked], 0);

    // Check that the velocities and positions of corresponding dofs match.
    Eigen::VectorBlock<const VectorX<double>> welded_positions_and_velocities =
        plant_welded->GetPositionsAndVelocities(welded_context);
    Eigen::VectorBlock<const VectorX<double>> locked_positions_and_velocities =
        plant_locked->GetPositionsAndVelocities(locked_context);

    EXPECT_NEAR(welded_positions_and_velocities[shoulder_index_welded],
                locked_positions_and_velocities[shoulder_index_locked], kEps);
    EXPECT_NEAR(welded_positions_and_velocities[plant_welded->num_positions() +
                                                shoulder_index_welded],
                locked_positions_and_velocities[plant_locked->num_positions() +
                                                shoulder_index_locked],
                kEps);

    // Position of the locked joint should be identically kElbowPosition.
    // Velocity of the locked joint should be identically 0.
    EXPECT_EQ(locked_positions_and_velocities[elbow_index_locked],
              kElbowPosition);
    EXPECT_EQ(locked_positions_and_velocities[plant_locked->num_positions() +
                                              elbow_index_locked],
              0);
  }
}

struct JointLockingSapTestConfig {
  std::string joint_typename;
  bool with_visualization{false};
};

std::ostream& operator<<(std::ostream& out, const JointLockingSapTestConfig& c) {
    out << c.joint_typename;
    return out;
}

const double kLadderMass = 10;      // [kg]
const double kLadderLength = 0.5;  // [m]
const double kLadderRadius = 0.05;  // [m]
const double kLadderAngle = 0.8;   // [rad]
const double kLadderVolume = M_PI * kLadderRadius * kLadderRadius *
                             ((4. / 3.) * kLadderRadius + kLadderLength);
class JointLockingSapTest
    : public ::testing::TestWithParam<JointLockingSapTestConfig> {
 public:
  void SetUp() {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, kTimestep);
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);

    JointLockingSapTestConfig config = GetParam();

    AddGround();
    AddLadderModel(config.joint_typename);

    plant_->Finalize();

    if (config.with_visualization) {
      std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
      MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph_, meshcat);
      ContactVisualizer<double>::AddToBuilder(&builder, *plant_, meshcat);
    }

    diagram_ = builder.Build();
  }

  void AddGround() {
    const geometry::Box G_shape(10, 10, 1);
    const CoulombFriction<double> G_mu(1.0, 1.0);

    const SpatialInertia<double> I_GGcm =
        SpatialInertia<double>::SolidBoxWithDensity(1, 10, 10, 1);

    const RigidBody<double>& ground = plant_->AddRigidBody("ground", I_GGcm);
    plant_->RegisterCollisionGeometry(ground, RigidTransformd::Identity(),
                                      G_shape, "ground_collision", G_mu);
    plant_->RegisterVisualGeometry(ground, RigidTransformd::Identity(), G_shape,
                                   "ground_visual",
                                   Vector4<double>(0.3, 0.3, 0.3, 1.0));

    plant_->WeldFrames(plant_->world_frame(), ground.body_frame(),
                       RigidTransformd(Vector3d{0, 0, -0.5}));
  }

  void AddLadderModel(std::string joint_typename) {
    const geometry::Capsule L_shape(kLadderRadius, kLadderLength);
    const CoulombFriction<double> L_mu(1.0, 1.0);

    const SpatialInertia<double> I_LLcm =
        SpatialInertia<double>::SolidCapsuleWithDensity(
            kLadderMass / kLadderVolume, kLadderRadius, kLadderLength,
            Vector3d{0, 0, 1});

    const RigidBody<double>& L_left = plant_->AddRigidBody("L_left", I_LLcm);
    const RigidBody<double>& L_right = plant_->AddRigidBody("L_right", I_LLcm);

    plant_->RegisterCollisionGeometry(L_left, RigidTransformd::Identity(),
                                      L_shape, "L_left_collision", L_mu);
    plant_->RegisterCollisionGeometry(L_right, RigidTransformd::Identity(),
                                      L_shape, "L_right_collision", L_mu);
    plant_->RegisterVisualGeometry(L_left, RigidTransformd::Identity(), L_shape,
                                   "L_left_visual",
                                   Vector4<double>(1, 0, 0, 1));
    plant_->RegisterVisualGeometry(L_right, RigidTransformd::Identity(),
                                   L_shape, "L_right_visual",
                                   Vector4<double>(0, 1, 0, 1));

    RigidTransformd X_LP(Vector3d(0, 0, kLadderLength / 2));
    RigidTransformd X_PC(RotationMatrix<double>::MakeYRotation(kLadderAngle));

    if (joint_typename == WeldJoint<double>::kTypeName) {
      plant_->AddJoint<WeldJoint>("left_right", L_left, X_LP, L_right, X_LP,
                                  X_PC);
    } else if (joint_typename == RevoluteJoint<double>::kTypeName) {
      const RevoluteJoint<double>& joint = plant_->AddJoint<RevoluteJoint>(
          "left_right", L_left, X_LP, L_right, X_LP, Vector3d{0, 1, 0});
      plant_->GetMutableJointByName<RevoluteJoint>(joint.name())
          .set_default_angle(kLadderAngle);
    }

    const RigidTransformd X_L_left(
        RotationMatrix<double>::MakeYRotation(-kLadderAngle / 2),
        Vector3d(0, 0, kLadderRadius + 0.5 * kLadderLength * std::cos(kLadderAngle / 2)));
    plant_->SetDefaultFreeBodyPose(L_left, X_L_left);
  }

 protected:
  MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
};

TEST_P(JointLockingSapTest, VerifyEquilibrium) {
  JointLockingSapTestConfig config = GetParam();
  std::unique_ptr<Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());

  plant_->GetJointByName("left_right").Lock(&plant_context);

  systems::Simulator<double> simulator(*diagram_, std::move(diagram_context));

  if (config.with_visualization) {
    simulator.set_target_realtime_rate(1);
  }
  simulator.Initialize();
  simulator.AdvanceTo(10);
}

std::vector<JointLockingSapTestConfig> MakeTestCases() {
  return std::vector<JointLockingSapTestConfig>{
      {.joint_typename = WeldJoint<double>::kTypeName,
       .with_visualization = true},
      {.joint_typename = RevoluteJoint<double>::kTypeName,
       .with_visualization = true}
  };
}

INSTANTIATE_TEST_SUITE_P(JointLockingTests, JointLockingSapTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace
}  // namespace multibody
}  // namespace drake
