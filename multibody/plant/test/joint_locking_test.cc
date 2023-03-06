#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

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

  static const std::vector<int>& EvalUnlockedVelocityIndices(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalJointLockingCache(context)
        .unlocked_velocity_indices;
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
        MultibodyPlantTester::EvalUnlockedVelocityIndices(*plant_, *context_);

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
        MultibodyPlantTester::EvalUnlockedVelocityIndices(*plant_, *context_);

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
        MultibodyPlantTester::EvalUnlockedVelocityIndices(*plant_, *context_);

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
// time step.
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

struct FilteredContactResultsConfig {
  ContactModel contact_model{ContactModel::kPoint};
  DiscreteContactSolver solver{DiscreteContactSolver::kTamsi};
};

std::ostream& operator<<(std::ostream& out,
                         const FilteredContactResultsConfig& c) {
  switch (c.solver) {
    case DiscreteContactSolver::kTamsi:
      out << "TAMSI";
      break;
    case DiscreteContactSolver::kSap:
      out << "SAP";
      break;
  }
  out << "_";
  switch (c.contact_model) {
    case ContactModel::kPoint:
      out << "point";
      break;
    case ContactModel::kHydroelastic:
      out << "hydroelastic";
      break;
    case ContactModel::kHydroelasticWithFallback:
      out << "hydroelastic_with_fallback";
      break;
  }
  return out;
}

// Utility testing class to construct a plant with two stacked spheres in
// contact with the contact model and discrete solver specified in the
// parameter.
class FilteredContactResultsTest
    : public ::testing::TestWithParam<FilteredContactResultsConfig> {
 public:
  void SetUp() {
    FilteredContactResultsConfig config = GetParam();

    systems::DiagramBuilder<double> builder;
    plant_ = &AddMultibodyPlantSceneGraph(&builder, 0.01 /* time_step */).plant;
    plant_->set_discrete_contact_solver(config.solver);
    plant_->set_contact_model(config.contact_model);

    const RigidBody<double>& ball_A = AddBall("ball_A");
    const RigidBody<double>& ball_B = AddBall("ball_B");

    // Position ball B above ball A in z such that they are in contact.
    const math::RigidTransformd X_WA{};
    const math::RigidTransformd X_WB{Vector3d{0, 0, radius_}};

    plant_->SetDefaultFreeBodyPose(ball_A, X_WA);
    plant_->SetDefaultFreeBodyPose(ball_B, X_WB);

    plant_->Finalize();
    diagram_ = builder.Build();
  }

  const RigidBody<double>& AddBall(const std::string& name) {
    SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(mass_, radius_);
    const RigidBody<double>& ball = plant_->AddRigidBody(name, M_BBcm);

    // Add sphere geometry for the ball.
    // Pose of sphere geometry S in body frame B.
    const RigidTransformd X_BS = RigidTransformd::Identity();
    // Set material properties.
    geometry::ProximityProperties ball_props;
    geometry::AddContactMaterial(
        1e-4 /* dissipation */, {} /* point stiffness */,
        CoulombFriction<double>{1.0, 1.0} /* friction */, &ball_props);

    // N.B. these properties go unused if the contact model is kPoint.
    geometry::AddCompliantHydroelasticProperties(
        0.5 * radius_, hydroelastic_modulus_, &ball_props);

    plant_->RegisterCollisionGeometry(ball, X_BS, geometry::Sphere(radius_),
                                      "collision", std::move(ball_props));

    // Add visual for the ball.
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    plant_->RegisterVisualGeometry(ball, X_BS, geometry::Sphere(radius_),
                                   "visual", red);
    return ball;
  }

 protected:
  const double mass_{1.0};
  const double radius_{1.0};
  const double hydroelastic_modulus_{1e5};
  MultibodyPlant<double>* plant_;
  std::unique_ptr<systems::Diagram<double>> diagram_{};
};

// Test that contact results are properly reported. MultibodyPlant will not
// report contact results between bodies that are either anchored or with all of
// their dofs locked.
TEST_P(FilteredContactResultsTest, VerifyLockedResults) {
  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  systems::Context<double>& context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());

  FilteredContactResultsConfig config = GetParam();

  // Both spheres unlocked, expect a single contact result.
  {
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 1);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);
    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_GT(results.num_hydroelastic_contacts(), 0);
    }
  }

  // One body of the pair is locked, expect a single contact result.
  {
    plant_->GetBodyByName("ball_A").Lock(&context);
    // Both spheres unlocked, should expect a single contact result.
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 1);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);
    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_GT(results.num_hydroelastic_contacts(), 0);
    }
  }

  // Both spheres locked, expect no contact result for the pair.
  {
    plant_->GetBodyByName("ball_A").Lock(&context);
    plant_->GetBodyByName("ball_B").Lock(&context);

    // Both spheres unlocked, should expect a single contact result.
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);
    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);
    }
  }
}

// Contact filtering happens in the CompliantContactManager before either SAP or
// TAMSI processes them so testing both is strictly not necessary. However this
// test serves as a smoke test for SAP when joint locking is implemented.
// TODO(joemasterjohn): Consider removing the smoke test when more robust SAP
// joint locking tests land.
std::vector<FilteredContactResultsConfig>
MakeFilteredContactResultsTestCases() {
  return std::vector<FilteredContactResultsConfig>{
      {.contact_model = ContactModel::kPoint,
       .solver = DiscreteContactSolver::kTamsi},
      {.contact_model = ContactModel::kHydroelastic,
       .solver = DiscreteContactSolver::kTamsi},
      {.contact_model = ContactModel::kPoint,
       .solver = DiscreteContactSolver::kSap},
      {.contact_model = ContactModel::kHydroelastic,
       .solver = DiscreteContactSolver::kSap}};
}

INSTANTIATE_TEST_SUITE_P(
    JointLockingTests, FilteredContactResultsTest,
    testing::ValuesIn(MakeFilteredContactResultsTestCases()),
    testing::PrintToStringParamName());

struct JointLockingLadderTestConfig {
  bool with_visualization{false};
  bool locked{false};
  DiscreteContactSolver solver{DiscreteContactSolver::kTamsi};
};

std::ostream& operator<<(std::ostream& out,
                         const JointLockingLadderTestConfig& c) {
  switch (c.solver) {
    case DiscreteContactSolver::kTamsi:
      out << "tamsi";
      break;
    case DiscreteContactSolver::kSap:
      out << "sap";
      break;
  }
  out << "_";
  out << (c.locked ? "locked" : "unlocked");
  return out;
}

const double kLMass = 1;       // [kg]
const double kLLength = 0.5;   // [m]
const double kLRadius = 0.05;  // [m]
const double kLq = 0.4;        // [rad]
// Volume for a capsule of radius kLRadius and length kLLength.
const double kLVolume =
    M_PI * kLRadius * kLRadius * ((4. / 3.) * kLRadius + kLLength);

/*
  Builds a simple acrobot model with capsule collision geometry above a
  frictionless ground plane. Initializes the configuration such that the
  acrobot forms a triangular shape with its furthest body (L_upper) in contact
  with the ground. The generalized coordinate for the revolute joint between
  the lower and upper body is denoted Lq and is set to kLq.

                O
               / \
              /   \
  L_lower -> /_Lq__\ <- L_upper
            /       \
           /         \
          /           \
  -------O-----------------------------

  The simple test verifies with both TAMSI and SAP that the acrobot slides down
  to a flat configuration during unlocked simulation and that when locked, the
  acrobot retains its standing configuration and the nominal angle kLq.
*/
class JointLockingLadderTest
    : public ::testing::TestWithParam<JointLockingLadderTestConfig> {
 public:
  void SetUp() {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, kTimestep);
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);

    AddModel();

    plant_->Finalize();

    if (GetParam().with_visualization) {
      std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
      MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph_, meshcat);
      ContactVisualizer<double>::AddToBuilder(&builder, *plant_, meshcat);
    }

    diagram_ = builder.Build();
  }

  void AddModel() {
    const geometry::HalfSpace G_shape;
    const CoulombFriction<double> G_mu(0.0, 0.0);
    plant_->RegisterCollisionGeometry(plant_->world_body(),
                                      RigidTransformd::Identity(), G_shape,
                                      "ground_collision", G_mu);
    plant_->RegisterVisualGeometry(
        plant_->world_body(), RigidTransformd::Identity(), G_shape,
        "ground_visual", Vector4<double>(0.3, 0.3, 0.3, 1.0));

    const geometry::Capsule L_shape(kLRadius, kLLength);
    const CoulombFriction<double> L_mu(0.0, 0.0);

    const SpatialInertia<double> I_LLcm =
        SpatialInertia<double>::SolidCapsuleWithDensity(
            kLMass / kLVolume, kLRadius, kLLength, Vector3d{0, 0, 1});

    const RigidBody<double>& L_lower = plant_->AddRigidBody("L_lower", I_LLcm);
    const RigidBody<double>& L_upper = plant_->AddRigidBody("L_upper", I_LLcm);

    plant_->RegisterCollisionGeometry(
        L_lower, RigidTransformd(Vector3d(0, 0, 0.5 * kLLength)),
        geometry::Sphere(kLRadius), "L_lower_collision", L_mu);
    plant_->RegisterCollisionGeometry(
        L_upper, RigidTransformd(Vector3d(0, 0, 0.5 * kLLength)),
        geometry::Sphere(kLRadius), "L_upper_collision", L_mu);
    plant_->RegisterVisualGeometry(L_lower, RigidTransformd::Identity(),
                                   L_shape, "L_lower_visual",
                                   Vector4<double>(1, 0, 0, 1));
    plant_->RegisterVisualGeometry(L_upper, RigidTransformd::Identity(),
                                   L_shape, "L_upper_visual",
                                   Vector4<double>(0, 1, 0, 1));
    plant_->RegisterVisualGeometry(
        L_lower, RigidTransformd(Vector3d(0, 0, 0.5 * kLLength)),
        geometry::Sphere(kLRadius), "L_lower_visual_sphere",
        Vector4<double>(1, 0, 0, 1));
    plant_->RegisterVisualGeometry(
        L_upper, RigidTransformd(Vector3d(0, 0, 0.5 * kLLength)),
        geometry::Sphere(kLRadius), "L_upper_visual_sphere",
        Vector4<double>(0, 1, 0, 1));

    // Frame P on L_lower and frame Q on L_upper.
    RigidTransformd X_LlowerP(Vector3d(0, 0, kLLength / 2));
    RigidTransformd X_LupperQ(Vector3d(0, 0, -kLLength / 2));

    // Frame M on L_lower, coincident with frame N in the world for the revolute
    // joint between world and L_lower.
    RigidTransformd X_LlowerM(Vector3d(0, 0, -kLLength / 2));
    RigidTransformd X_WN(Vector3d(0, 0, kLRadius));

    const RevoluteJoint<double>& joint =
        plant_->AddJoint<RevoluteJoint>("left_right", L_lower, X_LlowerP,
                                        L_upper, X_LupperQ, Vector3d{0, 1, 0});
    plant_->GetMutableJointByName<RevoluteJoint>(joint.name())
        .set_default_angle(M_PI - kLq);

    const RevoluteJoint<double>& world_left = plant_->AddJoint<RevoluteJoint>(
        "world_left", plant_->world_body(), X_WN, L_lower, X_LlowerM,
        Vector3d{0, 1, 0});

    plant_->GetMutableJointByName<RevoluteJoint>(world_left.name())
        .set_default_angle(0.5*kLq);
  }

 protected:
  MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
};

TEST_P(JointLockingLadderTest, VerifyStandingWhenLocked) {
  const double kEps = 1e-2;
  JointLockingLadderTestConfig config = GetParam();
  std::unique_ptr<Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());
  if (config.locked) {
    plant_->GetJointByName("left_right").Lock(&plant_context);
  }
  systems::Simulator<double> simulator(*diagram_, std::move(diagram_context));

  if (config.with_visualization) {
    simulator.set_target_realtime_rate(1);
  }
  simulator.Initialize();
  simulator.AdvanceTo(1);

  if (config.locked) {
    EXPECT_EQ(plant_->GetJointByName<RevoluteJoint>("left_right")
                  .get_angle(plant_context),
              M_PI - kLq);
  } else {
    EXPECT_NEAR(plant_->GetJointByName<RevoluteJoint>("left_right")
                    .get_angle(plant_context),
                0, kEps);
  }
}

std::vector<JointLockingLadderTestConfig> MakeJointLockingLadderTestCases() {
  return std::vector<JointLockingLadderTestConfig>{
      {.with_visualization = true,
       .locked = false,
       .solver = DiscreteContactSolver::kTamsi},
      {.with_visualization = true,
       .locked = true,
       .solver = DiscreteContactSolver::kTamsi},
      {.with_visualization = true,
       .locked = false,
       .solver = DiscreteContactSolver::kSap},
      {.with_visualization = true,
       .locked = true,
       .solver = DiscreteContactSolver::kSap}};
}

INSTANTIATE_TEST_SUITE_P(JointLockingTests, JointLockingLadderTest,
                         testing::ValuesIn(MakeJointLockingLadderTestCases()),
                         testing::PrintToStringParamName());

}  // namespace
}  // namespace multibody
}  // namespace drake
