#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
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

namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static const internal::JointLockingCacheData<double>& EvalJointLocking(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalJointLocking(context);
  }

  template <typename T>
  static BodyIndex FindBodyByGeometryId(const MultibodyPlant<T>& plant,
                                        geometry::GeometryId id) {
    return plant.FindBodyByGeometryId(id);
  }
};

namespace {

const double kTimestep = 0.01;
const double kElbowPosition = 0.3;
const double kArmLength = 0.1;

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
constexpr auto kDiscreteContactSolverTamsi = DiscreteContactSolver::kTamsi;
constexpr auto kDiscreteContactApproximationTamsi =
    DiscreteContactApproximation::kTamsi;
#pragma GCC diagnostic push

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

  const int body1_velocity_start = body1.floating_velocities_start_in_v();

  // No joints/bodies are locked, all joint/body velocity indices should exist.
  {
    const internal::JointLockingCacheData<double>& cache =
        MultibodyPlantTester::EvalJointLocking(*plant_, *context_);
    const std::vector<int>& unlocked_velocity_indices =
        cache.unlocked_velocity_indices;
    const std::vector<int>& locked_velocity_indices =
        cache.locked_velocity_indices;
    const std::vector<std::vector<int>>& unlocked_velocity_indices_per_tree =
        cache.unlocked_velocity_indices_per_tree;
    const std::vector<std::vector<int>>& locked_velocity_indices_per_tree =
        cache.locked_velocity_indices_per_tree;

    EXPECT_EQ(unlocked_velocity_indices.size(), 9);
    EXPECT_EQ(locked_velocity_indices.size(), 0);

    EXPECT_EQ(unlocked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(locked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[0].size(), 2);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[1].size(), 7);
    EXPECT_EQ(locked_velocity_indices_per_tree[0].size(), 0);
    EXPECT_EQ(locked_velocity_indices_per_tree[1].size(), 0);

    const std::vector<int> expected_unlocked_velocity_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};
    const std::vector<int> expected_locked_velocity_indices = {};

    EXPECT_THAT(
        unlocked_velocity_indices,
        testing::UnorderedElementsAreArray(expected_unlocked_velocity_indices));
    EXPECT_THAT(locked_velocity_indices, testing::UnorderedElementsAreArray(
                                             expected_locked_velocity_indices));

    const std::vector<std::vector<int>>
        expected_unlocked_velocity_indices_per_tree = {
            {0, 1},
            {0, 1, 2, 3, 4, 5, 6},
        };
    const std::vector<std::vector<int>>
        expected_locked_velocity_indices_per_tree = {{}, {}};

    for (int i = 0; i < 2; ++i) {
      EXPECT_THAT(unlocked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_unlocked_velocity_indices_per_tree[i]));
      EXPECT_THAT(locked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_locked_velocity_indices_per_tree[i]));
    }
  }

  // Lock body3_body4 and re-evaluate joint locking indices
  {
    body3_body4.Lock(context_.get());
    const internal::JointLockingCacheData<double>& cache =
        MultibodyPlantTester::EvalJointLocking(*plant_, *context_);
    const std::vector<int>& unlocked_velocity_indices =
        cache.unlocked_velocity_indices;
    const std::vector<int>& locked_velocity_indices =
        cache.locked_velocity_indices;
    const std::vector<std::vector<int>>& unlocked_velocity_indices_per_tree =
        cache.unlocked_velocity_indices_per_tree;
    const std::vector<std::vector<int>>& locked_velocity_indices_per_tree =
        cache.locked_velocity_indices_per_tree;

    EXPECT_EQ(unlocked_velocity_indices.size(), 8);
    EXPECT_EQ(locked_velocity_indices.size(), 1);

    EXPECT_EQ(unlocked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(locked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[0].size(), 1);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[1].size(), 7);
    EXPECT_EQ(locked_velocity_indices_per_tree[0].size(), 1);
    EXPECT_EQ(locked_velocity_indices_per_tree[1].size(), 0);

    const std::vector<int>& expected_unlocked_velocity_indices = {
        body1_velocity_start,         body1_velocity_start + 1,
        body1_velocity_start + 2,     body1_velocity_start + 3,
        body1_velocity_start + 4,     body1_velocity_start + 5,
        body1_body2.velocity_start(), world_body3.velocity_start()};
    const std::vector<int>& expected_locked_velocity_indices = {
        body3_body4.velocity_start()};

    EXPECT_THAT(
        unlocked_velocity_indices,
        testing::UnorderedElementsAreArray(expected_unlocked_velocity_indices));
    EXPECT_THAT(locked_velocity_indices, testing::UnorderedElementsAreArray(
                                             expected_locked_velocity_indices));

    const std::vector<std::vector<int>>
        expected_unlocked_velocity_indices_per_tree = {{0},
                                                       {0, 1, 2, 3, 4, 5, 6}};
    const std::vector<std::vector<int>>
        expected_locked_velocity_indices_per_tree = {{1}, {}};

    for (int i = 0; i < 2; ++i) {
      EXPECT_THAT(unlocked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_unlocked_velocity_indices_per_tree[i]));
      EXPECT_THAT(locked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_locked_velocity_indices_per_tree[i]));
    }
  }

  // Unlock body3_body4 and lock body1 and re-evaluate joint locking indices.
  {
    body3_body4.Unlock(context_.get());
    body1.Lock(context_.get());

    const internal::JointLockingCacheData<double>& cache =
        MultibodyPlantTester::EvalJointLocking(*plant_, *context_);
    const std::vector<int>& unlocked_velocity_indices =
        cache.unlocked_velocity_indices;
    const std::vector<int>& locked_velocity_indices =
        cache.locked_velocity_indices;
    const std::vector<std::vector<int>>& unlocked_velocity_indices_per_tree =
        cache.unlocked_velocity_indices_per_tree;
    const std::vector<std::vector<int>>& locked_velocity_indices_per_tree =
        cache.locked_velocity_indices_per_tree;

    EXPECT_EQ(unlocked_velocity_indices.size(), 3);
    EXPECT_EQ(locked_velocity_indices.size(), 6);

    EXPECT_EQ(unlocked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(locked_velocity_indices_per_tree.size(), 2);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[0].size(), 2);
    EXPECT_EQ(unlocked_velocity_indices_per_tree[1].size(), 1);
    EXPECT_EQ(locked_velocity_indices_per_tree[0].size(), 0);
    EXPECT_EQ(locked_velocity_indices_per_tree[1].size(), 6);

    const std::vector<int>& expected_unlocked_velocity_indices = {
        body1_body2.velocity_start(), world_body3.velocity_start(),
        body3_body4.velocity_start()};
    const std::vector<int>& expected_locked_velocity_indices = {
        body1_velocity_start,     body1_velocity_start + 1,
        body1_velocity_start + 2, body1_velocity_start + 3,
        body1_velocity_start + 4, body1_velocity_start + 5};

    EXPECT_THAT(
        unlocked_velocity_indices,
        testing::UnorderedElementsAreArray(expected_unlocked_velocity_indices));
    EXPECT_THAT(locked_velocity_indices, testing::UnorderedElementsAreArray(
                                             expected_locked_velocity_indices));

    const std::vector<std::vector<int>>
        expected_unlocked_velocity_indices_per_tree = {{0, 1}, {6}};
    const std::vector<std::vector<int>>
        expected_locked_velocity_indices_per_tree = {{}, {0, 1, 2, 3, 4, 5}};

    for (int i = 0; i < 2; ++i) {
      EXPECT_THAT(unlocked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_unlocked_velocity_indices_per_tree[i]));
      EXPECT_THAT(locked_velocity_indices_per_tree[i],
                  testing::UnorderedElementsAreArray(
                      expected_locked_velocity_indices_per_tree[i]));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(IndexPermutations, JointLockingTest,
                         ::testing::Values(0, 1));

struct TrajectoryTestConfig {
  DiscreteContactSolver solver{kDiscreteContactSolverTamsi};
};

std::ostream& operator<<(std::ostream& out, DiscreteContactSolver solver) {
  switch (solver) {
    case kDiscreteContactSolverTamsi: {
      out << "TAMSI";
      break;
    }
    case DiscreteContactSolver::kSap: {
      out << "SAP";
      break;
    }
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const TrajectoryTestConfig& c) {
  return out << c.solver;
}

// Fixture to construct two plants. Each containing a single double pendulum.
// One of the plants will have a weld joint at the elbow joint to lock its
// position, while the other will have a revolute joint, locked using the joint
// locking APIs.
class TrajectoryTest : public ::testing::TestWithParam<TrajectoryTestConfig> {
 public:
  void SetUp() {
    TrajectoryTestConfig config = GetParam();
    plant_welded_ = MakeDoublePendulumPlant(true, config.solver);
    plant_locked_ = MakeDoublePendulumPlant(false, config.solver);
  }

  // Create a plant with a XZ-planar double pendulum where the masses are
  // concentrated at the lower ends of the two links. The 0-configuration has
  // the upper arm sticking out horizontally at (-kArmLength, 0, 0) and the
  // lower arm placed at (-kArmLength, 0, 0.0) relative to the upper arm's
  // frame. If the `weld_elbow` parameter is true, then the revolute joint
  // between bodies `upper_arm` and `lower_arm` is replaced with a fixed weld
  // corresponding to the configuration (0, kElbowPosition) in the model with
  // two joints.
  std::unique_ptr<MultibodyPlant<double>> MakeDoublePendulumPlant(
      bool weld_elbow, DiscreteContactSolver solver) {
    std::unique_ptr<MultibodyPlant<double>> plant;
    plant = std::make_unique<MultibodyPlant<double>>(kTimestep);
    // N.B. We want to exercise the TAMSI and SAP code paths. Therefore we
    // arbitrarily choose two model approximations to accomplish this.
    switch (solver) {
      case kDiscreteContactSolverTamsi:
        plant->set_discrete_contact_approximation(
            kDiscreteContactApproximationTamsi);
        break;
      case DiscreteContactSolver::kSap:
        plant->set_discrete_contact_approximation(
            DiscreteContactApproximation::kSap);
        break;
    }

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

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_welded_{nullptr};
  std::unique_ptr<MultibodyPlant<double>> plant_locked_{nullptr};
};

// To verify that the physical behavior of a locked joint is identical to a weld
// joint, we construct two plants. Each plant consists of a double pendulum with
// the shoulder welded to the world. The elbow joint of `plant_welded_` is
// replaced with a WeldJoint fixed at configuration (0, kElbowPosition). The
// elbow joint of `plant_locked_` is a revolute joint that has been locked at
// the configuration (0, kElbowPosition). We verify that the generalized
// accelerations, velocities and positions match to a given accuracy at each
// time step.
TEST_P(TrajectoryTest, CompareWeldAndLocked) {
  // Allow 2 digits of precision loss to account for roundoff differences
  // between the two code paths. This value was determined empircally by
  // observing the maximum error between the two trajectories.
  const double kEps = 1e2 * std::numeric_limits<double>::epsilon();
  const int kNumTimesteps = 10;

  std::unique_ptr<systems::Context<double>> context_welded =
      plant_welded_->CreateDefaultContext();
  std::unique_ptr<systems::Context<double>> context_locked =
      plant_locked_->CreateDefaultContext();

  // Lock the elbow in the unwelded plant.
  const RevoluteJoint<double>& elbow =
      plant_locked_->GetJointByName<RevoluteJoint>("elbow");
  elbow.set_angle(context_locked.get(), kElbowPosition);
  elbow.Lock(context_locked.get());

  // Sanity check.
  ASSERT_EQ(plant_welded_->num_velocities(), 1);
  ASSERT_EQ(plant_locked_->num_velocities(), 2);

  auto simulator_welded = std::make_unique<Simulator<double>>(
      *plant_welded_, std::move(context_welded));
  auto simulator_locked = std::make_unique<Simulator<double>>(
      *plant_locked_, std::move(context_locked));
  simulator_welded->Initialize();
  simulator_locked->Initialize();

  // Simulate for kNumTimesteps and verify acceleration, velocity, and positions
  // at each iteration.
  for (int i = 1; i <= kNumTimesteps; ++i) {
    simulator_welded->AdvanceTo(i * kTimestep);
    simulator_locked->AdvanceTo(i * kTimestep);

    const auto& welded_context =
        plant_welded_->GetMyContextFromRoot(simulator_welded->get_context());
    const auto& locked_context =
        plant_locked_->GetMyContextFromRoot(simulator_locked->get_context());

    int shoulder_index_welded =
        plant_welded_->GetJointByName<RevoluteJoint>("shoulder")
            .velocity_start();
    int shoulder_index_locked =
        plant_locked_->GetJointByName<RevoluteJoint>("shoulder")
            .velocity_start();

    // The welded plant has only one dof, shoulder. The acceleration of each
    // plant at the corresponding dof should match.
    const auto welded_accelerations =
        plant_welded_->get_generalized_acceleration_output_port().Eval(
            welded_context);
    const auto locked_accelerations =
        plant_locked_->get_generalized_acceleration_output_port().Eval(
            locked_context);
    EXPECT_NEAR(welded_accelerations[shoulder_index_welded],
                locked_accelerations[shoulder_index_locked], kEps);
    // The locked elbow dof should have 0 acceleration.
    int elbow_index_locked =
        plant_locked_->GetJointByName<RevoluteJoint>("elbow").velocity_start();
    EXPECT_EQ(locked_accelerations[elbow_index_locked], 0);

    // Check that the velocities and positions of corresponding dofs match.
    Eigen::VectorBlock<const VectorX<double>> welded_positions_and_velocities =
        plant_welded_->GetPositionsAndVelocities(welded_context);
    Eigen::VectorBlock<const VectorX<double>> locked_positions_and_velocities =
        plant_locked_->GetPositionsAndVelocities(locked_context);

    EXPECT_NEAR(welded_positions_and_velocities[shoulder_index_welded],
                locked_positions_and_velocities[shoulder_index_locked], kEps);
    EXPECT_NEAR(welded_positions_and_velocities[plant_welded_->num_positions() +
                                                shoulder_index_welded],
                locked_positions_and_velocities[plant_locked_->num_positions() +
                                                shoulder_index_locked],
                kEps);

    // Position of the locked joint should be identically kElbowPosition.
    // Velocity of the locked joint should be identically 0.
    EXPECT_EQ(locked_positions_and_velocities[elbow_index_locked],
              kElbowPosition);
    EXPECT_EQ(locked_positions_and_velocities[plant_locked_->num_positions() +
                                              elbow_index_locked],
              0);
  }
}

// Test joint locking with TAMSI and SAP.
std::vector<TrajectoryTestConfig> MakeTrajectoryTestCases() {
  return std::vector<TrajectoryTestConfig>{
      {.solver = kDiscreteContactSolverTamsi},
      {.solver = DiscreteContactSolver::kSap},
  };
}

INSTANTIATE_TEST_SUITE_P(JointLockingTests, TrajectoryTest,
                         testing::ValuesIn(MakeTrajectoryTestCases()),
                         testing::PrintToStringParamName());

struct FilteredContactResultsConfig {
  ContactModel contact_model{ContactModel::kPoint};
  std::optional<DiscreteContactSolver> solver{kDiscreteContactSolverTamsi};
};

std::ostream& operator<<(std::ostream& out,
                         const FilteredContactResultsConfig& c) {
  out << internal::GetStringFromContactModel(c.contact_model) << "_";
  if (c.solver.has_value()) {
    out << *c.solver;
  } else {
    out << "continuous";
  }
  return out;
}

// Utility testing class to construct a plant with three stacked spheres in
// contact with the contact model specified in the parameter.
class FilteredContactResultsTest
    : public ::testing::TestWithParam<FilteredContactResultsConfig> {
 public:
  void SetUp() {
    FilteredContactResultsConfig config = GetParam();
    const double time_step = config.solver.has_value() ? kTimestep : 0.0;

    systems::DiagramBuilder<double> builder;
    plant_ = &AddMultibodyPlantSceneGraph(&builder, time_step).plant;
    plant_->set_contact_model(config.contact_model);
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.
    if (time_step > 0) {
      // N.B. We want to exercise the TAMSI and SAP code paths. Therefore we
      // arbitrarily choose two model approximations to accomplish this.
      switch (*config.solver) {
        case kDiscreteContactSolverTamsi:
          plant_->set_discrete_contact_approximation(
              kDiscreteContactApproximationTamsi);
          break;
        case DiscreteContactSolver::kSap:
          plant_->set_discrete_contact_approximation(
              DiscreteContactApproximation::kSap);
          break;
      }
    }

    const RigidBody<double>& ball_A = AddBall("ball_A");
    const RigidBody<double>& ball_B = AddBall("ball_B");
    const RigidBody<double>& ball_C = AddBall("ball_C");

    // Position ball B above ball A in z such that they are in contact.
    // Position ball C above ball B in z such that they are in contact.
    const math::RigidTransformd X_WA{};
    const math::RigidTransformd X_WB{Vector3d{0, 0, 1.95 * radius_}};
    const math::RigidTransformd X_WC{Vector3d{0, 0, 3.9 * radius_}};

    plant_->SetDefaultFloatingBaseBodyPose(ball_A, X_WA);
    plant_->SetDefaultFloatingBaseBodyPose(ball_B, X_WB);
    plant_->SetDefaultFloatingBaseBodyPose(ball_C, X_WC);

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
    geometry::AddCompliantHydroelasticProperties(radius_, hydroelastic_modulus_,
                                                 &ball_props);

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
// their dofs locked. However, geometry query data from SceneGraph (e.g
// data from MbP::CalcGeometryContactData())
// are not automatically filtered. Thus contact results must store a reference
// of either the point pair or hydroelastic contact surface that corresponds to
// that result. We verify that contact results for point contact contain correct
// references to its PointPairPenetration by checking that they store the same
// body indices, in the same order.
TEST_P(FilteredContactResultsTest, VerifyLockedResults) {
  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  systems::Context<double>& context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());

  FilteredContactResultsConfig config = GetParam();

  // All spheres unlocked, expect two contact results.
  {
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 2);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);

      // Check that the point contact result stores the correct corresponding
      // `PenetrationAsPointPair` from geometry queries.
      for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
        const PointPairContactInfo<double>& info =
            results.point_pair_contact_info(i);
        const BodyIndex body_A = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_A);
        const BodyIndex body_B = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_B);
        EXPECT_EQ(body_A, info.bodyA_index());
        EXPECT_EQ(body_B, info.bodyB_index());
      }

    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 2);
    }
  }

  // One body locked, still expect two contact results.
  {
    plant_->GetBodyByName("ball_A").Lock(&context);
    // Both spheres unlocked, should expect a single contact result.
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 2);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);

      // Check that the point contact result stores the correct corresponding
      // `PenetrationAsPointPair` from geometry queries.
      for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
        auto info = results.point_pair_contact_info(i);
        const BodyIndex body_A = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_A);
        const BodyIndex body_B = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_B);
        EXPECT_EQ(body_A, info.bodyA_index());
        EXPECT_EQ(body_B, info.bodyB_index());
      }

    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 2);
    }
  }

  // Two spheres locked, expect only one contact result.
  {
    plant_->GetBodyByName("ball_A").Lock(&context);
    plant_->GetBodyByName("ball_B").Lock(&context);

    // Both spheres unlocked, should expect a single contact result.
    const ContactResults<double>& results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            context);

    if (config.contact_model == ContactModel::kPoint) {
      EXPECT_EQ(results.num_point_pair_contacts(), 1);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 0);

      // Check that the point contact result stores the correct corresponding
      // `PenetrationAsPointPair` from geometry queries.
      for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
        auto info = results.point_pair_contact_info(i);
        const BodyIndex body_A = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_A);
        const BodyIndex body_B = MultibodyPlantTester::FindBodyByGeometryId(
            *plant_, info.point_pair().id_B);
        EXPECT_EQ(body_A, info.bodyA_index());
        EXPECT_EQ(body_B, info.bodyB_index());
      }

    } else {
      EXPECT_EQ(results.num_point_pair_contacts(), 0);
      EXPECT_EQ(results.num_hydroelastic_contacts(), 1);
    }
  }
}

// Test filtering with both point and hydroelastic geometries.
std::vector<FilteredContactResultsConfig>
MakeFilteredContactResultsTestCases() {
  return std::vector<FilteredContactResultsConfig>{
      {.contact_model = ContactModel::kPoint, .solver = std::nullopt},
      {.contact_model = ContactModel::kHydroelastic, .solver = std::nullopt},
      {.contact_model = ContactModel::kPoint,
       .solver = kDiscreteContactSolverTamsi},
      {.contact_model = ContactModel::kHydroelastic,
       .solver = kDiscreteContactSolverTamsi},
      {.contact_model = ContactModel::kPoint,
       .solver = DiscreteContactSolver::kSap},
      {.contact_model = ContactModel::kHydroelastic,
       .solver = DiscreteContactSolver::kSap},
  };
}

INSTANTIATE_TEST_SUITE_P(
    JointLockingTests, FilteredContactResultsTest,
    testing::ValuesIn(MakeFilteredContactResultsTestCases()),
    testing::PrintToStringParamName());
}  // namespace
}  // namespace multibody
}  // namespace drake
