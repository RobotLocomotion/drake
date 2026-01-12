#include "drake/multibody/tree/curvilinear_joint.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;

class CurvilinearJointTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a curvilinear joint
  // with the sole purpose of testing the CurvilinearJoint user facing API.
  void SetUp() override {
    std::unique_ptr<internal::MultibodyTree<double>> model = MakeModel();

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model), true /* is_discrete */);
    context_ = system_->CreateDefaultContext();
  }

  std::unique_ptr<internal::MultibodyTree<double>> MakeModel() {
    // Spatial inertia for adding body. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const auto M_B = SpatialInertia<double>::NaN();

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add a body so we can add joint to it.
    body1_ = &model->AddRigidBody("Body", M_B);

    // Add a curvilinear joint between the world and body1.
    joint1_ = &model->AddJoint<CurvilinearJoint>(
        "Joint1", model->world_body(), std::nullopt, *body1_, std::nullopt,
        trajectory_, kDamping);
    Joint<double>& mutable_joint = model->get_mutable_joint(joint1_->index());
    mutable_joint1_ = dynamic_cast<CurvilinearJoint<double>*>(&mutable_joint);
    mutable_joint1_->set_position_limits(
        Vector1<double>::Constant(kPositionLowerLimit),
        Vector1<double>::Constant(kPositionUpperLimit));
    mutable_joint1_->set_velocity_limits(
        Vector1<double>::Constant(kVelocityLowerLimit),
        Vector1<double>::Constant(kVelocityUpperLimit));
    mutable_joint1_->set_acceleration_limits(
        Vector1<double>::Constant(kAccelerationLowerLimit),
        Vector1<double>::Constant(kAccelerationUpperLimit));

    return model;
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* body1_{nullptr};
  const CurvilinearJoint<double>* joint1_{nullptr};
  CurvilinearJoint<double>* mutable_joint1_{nullptr};

 protected:
  // We specify a trajectory that draws a "stadium curve" (a rectangle with two
  // semicircle caps). The Rectangular section has length l_ and the circular
  // sections have radii 1/k_. This trajectory is periodic, of total length 2*l_
  // + 2*pi/k_.
  const double k_ = 1.0;
  const double l_ = 1.0;
  const std::vector<double> breaks_{0, M_PI / k_, l_ + M_PI / k_,
                                    l_ + 2 * M_PI / k_, 2 * l_ + 2 * M_PI / k_};
  const std::vector<double> turning_rates_{k_, 0, k_, 0};
  const Vector3d tangent_axis_{1., -std::sqrt(2.), 1.};
  const Vector3d plane_axis_{1., std::sqrt(2.), 1.};
  const Vector3d initial_position_{1., 2., 3.};
  const trajectories::PiecewiseConstantCurvatureTrajectory<double> trajectory_{
      breaks_, turning_rates_, tangent_axis_, plane_axis_, initial_position_};
};

TEST_F(CurvilinearJointTest, Type) {
  const Joint<double>& base = *joint1_;
  EXPECT_EQ(base.type_name(), CurvilinearJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(CurvilinearJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 1);
  EXPECT_EQ(tree().num_velocities(), 1);
  EXPECT_EQ(joint1_->num_positions(), 1);
  EXPECT_EQ(joint1_->num_velocities(), 1);
  EXPECT_EQ(joint1_->position_start(), 0);
  EXPECT_EQ(joint1_->velocity_start(), 0);
}

TEST_F(CurvilinearJointTest, GetJointLimits) {
  EXPECT_EQ(joint1_->position_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->position_upper_limits().size(), 1);
  EXPECT_EQ(joint1_->velocity_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->velocity_upper_limits().size(), 1);
  EXPECT_EQ(joint1_->acceleration_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->acceleration_upper_limits().size(), 1);

  EXPECT_EQ(joint1_->position_lower_limit(), kPositionLowerLimit);
  EXPECT_EQ(joint1_->position_upper_limit(), kPositionUpperLimit);
  EXPECT_EQ(joint1_->velocity_lower_limit(), kVelocityLowerLimit);
  EXPECT_EQ(joint1_->velocity_upper_limit(), kVelocityUpperLimit);
  EXPECT_EQ(joint1_->acceleration_lower_limit(), kAccelerationLowerLimit);
  EXPECT_EQ(joint1_->acceleration_upper_limit(), kAccelerationUpperLimit);
}

TEST_F(CurvilinearJointTest, Damping) {
  std::unique_ptr<internal::MultibodyTree<double>> model = MakeModel();
  auto& joint = model->GetMutableJointByName<CurvilinearJoint>("Joint1");
  EXPECT_EQ(joint.default_damping(), kDamping);
  EXPECT_EQ(joint.default_damping_vector(), Vector1d(kDamping));
  const double new_damping = 2.0 * kDamping;
  joint.set_default_damping(new_damping);
  EXPECT_EQ(joint.default_damping(), new_damping);
  EXPECT_EQ(joint.default_damping_vector(), Vector1d(new_damping));

  // Expect to throw on invalid damping values.
  EXPECT_THROW(joint.set_default_damping(-1), std::exception);
}

// Context-dependent value access.
TEST_F(CurvilinearJointTest, ContextDependentAccess) {
  const double some_value = 1.5;
  // Translation access.
  joint1_->set_distance(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_distance(*context_), some_value);

  // Translation rate access.
  joint1_->set_tangential_velocity(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_tangential_velocity(*context_), some_value);

  // Joint locking.
  joint1_->Lock(context_.get());
  EXPECT_EQ(joint1_->get_tangential_velocity(*context_), 0.);

  // Damping.
  EXPECT_EQ(joint1_->GetDamping(*context_), kDamping);
  EXPECT_EQ(joint1_->GetDampingVector(*context_), Vector1d(kDamping));

  EXPECT_NO_THROW(
      joint1_->SetDampingVector(context_.get(), Vector1d(some_value)));
  EXPECT_EQ(joint1_->GetDamping(*context_), some_value);
  EXPECT_EQ(joint1_->GetDampingVector(*context_), Vector1d(some_value));

  EXPECT_NO_THROW(joint1_->SetDamping(context_.get(), kDamping));
  EXPECT_EQ(joint1_->GetDamping(*context_), kDamping);
  EXPECT_EQ(joint1_->GetDampingVector(*context_), Vector1d(kDamping));

  // Expect to throw on invalid damping values.
  EXPECT_THROW(joint1_->SetDamping(context_.get(), -1), std::exception);
  EXPECT_THROW(joint1_->SetDampingVector(context_.get(), Vector1d(-1)),
               std::exception);
}

// Tests API to apply torques to a joint.
TEST_F(CurvilinearJointTest, AddInForces) {
  const double some_value = 1.5;
  // Default initialized to zero forces.
  MultibodyForces<double> forces1(tree());

  // Add value twice.
  joint1_->AddInForce(*context_, some_value, &forces1);
  joint1_->AddInForce(*context_, some_value, &forces1);

  MultibodyForces<double> forces2(tree());
  // Add value only once.
  joint1_->AddInForce(*context_, some_value, &forces2);
  // Add forces2 into itself (same as adding torque twice).
  forces2.AddInForces(forces2);

  // forces1 and forces2 should be equal.
  EXPECT_EQ(forces1.generalized_forces(), forces2.generalized_forces());
  auto F2 = forces2.body_forces().cbegin();
  for (auto& F1 : forces1.body_forces())
    EXPECT_TRUE(F1.IsApprox(*F2++, kEpsilon));
}

// Tests API to add in damping forces.
TEST_F(CurvilinearJointTest, AddInDampingForces) {
  const double translational_velocity = 0.1;
  const double damping = 0.2 * kDamping;

  const Vector1d damping_force_expected(-damping * translational_velocity);

  joint1_->set_tangential_velocity(context_.get(), translational_velocity);
  joint1_->SetDamping(context_.get(), damping);

  MultibodyForces<double> forces(tree());
  joint1_->AddInDamping(*context_, &forces);
  EXPECT_EQ(forces.generalized_forces(), damping_force_expected);
}

TEST_F(CurvilinearJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint1_clone1 = dynamic_cast<const CurvilinearJoint<AutoDiffXd>&>(
      model_clone->get_variant(*joint1_));

  const std::unique_ptr<Joint<AutoDiffXd>> shallow =
      joint1_clone1.ShallowClone();
  const auto& joint1_clone2 =
      dynamic_cast<const CurvilinearJoint<AutoDiffXd>&>(*shallow);

  for (const auto* clone : {&joint1_clone1, &joint1_clone2}) {
    EXPECT_EQ(clone->name(), joint1_->name());
    EXPECT_EQ(clone->frame_on_parent().index(),
              joint1_->frame_on_parent().index());
    EXPECT_EQ(clone->frame_on_child().index(),
              joint1_->frame_on_child().index());
    EXPECT_EQ(clone->position_lower_limits(), joint1_->position_lower_limits());
    EXPECT_EQ(clone->position_upper_limits(), joint1_->position_upper_limits());
    EXPECT_EQ(clone->velocity_lower_limits(), joint1_->velocity_lower_limits());
    EXPECT_EQ(clone->velocity_upper_limits(), joint1_->velocity_upper_limits());
    EXPECT_EQ(clone->acceleration_lower_limits(),
              joint1_->acceleration_lower_limits());
    EXPECT_EQ(clone->acceleration_upper_limits(),
              joint1_->acceleration_upper_limits());
    EXPECT_EQ(clone->default_damping(), joint1_->default_damping());
    EXPECT_EQ(clone->get_default_distance(), joint1_->get_default_distance());
  }
}

TEST_F(CurvilinearJointTest, CanRotateAndTranslate) {
  EXPECT_TRUE(joint1_->can_rotate());
  EXPECT_TRUE(joint1_->can_translate());
}

TEST_F(CurvilinearJointTest, NameSuffix) {
  EXPECT_EQ(joint1_->position_suffix(0), "q");
  EXPECT_EQ(joint1_->velocity_suffix(0), "v");
}

TEST_F(CurvilinearJointTest, RandomTranslationTest) {
  // Calling SetRandomState before setting the distribution results in the
  // zero state.
  RandomGenerator generator;
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_EQ(joint1_->get_distance(*context_), 0.);

  // Setup distribution for random initial conditions.
  std::uniform_real_distribution<symbolic::Expression> uniform(
      1.0, kPositionUpperLimit);
  mutable_joint1_->set_random_distance_distribution(uniform());
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_LE(1.0, joint1_->get_distance(*context_));
  EXPECT_GE(kPositionUpperLimit, joint1_->get_distance(*context_));
}

TEST_F(CurvilinearJointTest, DefaultTranslation) {
  const double default_distance = 0.0;

  const double new_default_distance =
      0.5 * kPositionLowerLimit + 0.5 * kPositionUpperLimit;

  const double out_of_bounds_low_translation = kPositionLowerLimit - 1;
  const double out_of_bounds_high_translation = kPositionUpperLimit + 1;

  // Constructor should set the default translation to 0.0.
  EXPECT_EQ(joint1_->get_default_distance(), default_distance);

  // Setting a new default translation should propagate so that
  // get_default_distance() remains correct.
  mutable_joint1_->set_default_distance(new_default_distance);
  EXPECT_EQ(joint1_->get_default_distance(), new_default_distance);

  // Setting the default angle out of the bounds of the position limits
  // should NOT throw an exception.
  EXPECT_NO_THROW(
      mutable_joint1_->set_default_distance(out_of_bounds_low_translation));
  EXPECT_NO_THROW(
      mutable_joint1_->set_default_distance(out_of_bounds_high_translation));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
