#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::multibody_plant::MultibodyPlant;
using systems::Context;
using std::unique_ptr;

namespace multibody {
namespace multibody_plant {
namespace {

// Fixture to perform a number of computational tests on a KUKA Iiwa model.
class KukaIiwaModelTests : public ::testing::Test {
 public:
  // Creates MultibodyTree for a KUKA Iiwa robot arm.
  void SetUp() override {
    const std::string kArmSdfPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf");

    // Create a model of a Kuka arm. Notice we do not weld the robot's base
    // to the world and therefore the model is free floating in space. This
    // makes for a more interesting setup to test the computation of
    // analytical Jacobians.
    plant_ = std::make_unique<MultibodyPlant<double>>();
    parsing::Parser parser(plant_.get());
    parser.AddModelFromFile(kArmSdfPath);
    // Add a frame H with a fixed pose X_GH in the end effector frame G.
    end_effector_link_ = &plant_->GetBodyByName("iiwa_link_7");
    frame_H_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "H", *end_effector_link_, X_GH_.GetAsIsometry3()));
    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();

    // Scalar-convert the model and create a default context for it.
    plant_autodiff_ = std::make_unique<MultibodyPlant<AutoDiffXd>>(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  void SetArbitraryConfiguration() {
    // Get an arbitrary set of angles and velocities for each joint.
    const VectorX<double> x0 = GetArbitraryJointConfiguration();

    EXPECT_EQ(plant_->num_joints(), 7);
    for (JointIndex joint_index(0); joint_index < plant_->num_joints();
         ++joint_index) {
      const RevoluteJoint<double>& joint =
          dynamic_cast<const RevoluteJoint<double>&>(
              plant_->tree().get_joint(joint_index));
      joint.set_angle(context_.get(), x0[joint_index]);
      joint.set_angular_rate(context_.get(), x0[kNumJoints + joint_index]);
    }

    // Set an arbitrary (though non-identity) pose of the floating base link.
    const auto& base_body = plant_->GetBodyByName("iiwa_link_0");
    const RigidTransform<double> X_WB(
        RollPitchYaw<double>(M_PI / 3, -M_PI / 2, M_PI / 8),
        Vector3<double>(0.05, -0.2, 0.05));
    plant_->SetFreeBodyPoseInAnchoredFrame(
        context_.get(), plant_->world_frame(), base_body,
        X_WB.GetAsIsometry3());
    // Set an arbitrary non-zero spatial velocity of the floating base link.
    const Vector3<double> w_WB{-1, 1, -1};
    const Vector3<double> v_WB{1, -1, 1};
    plant_->tree().SetFreeBodySpatialVelocityOrThrow(base_body, {w_WB, v_WB},
                                                     context_.get());
  }

  // Gets an arm state to an arbitrary configuration in which joint angles and
  // rates are non-zero.
  VectorX<double> GetArbitraryJointConfiguration() {
    VectorX<double> x(2 * kNumJoints);

    // A set of values for the joint's angles chosen mainly to avoid in-plane
    // motions.
    const double q30 = M_PI / 6, q60 = M_PI / 3;
    const double qA = q60;
    const double qB = q30;
    const double qC = q60;
    const double qD = q30;
    const double qE = q60;
    const double qF = q30;
    const double qG = q60;
    // Arbitrary non-zero velocities.
    const double v_positive = 0.1;   // rad/s
    const double v_negative = -0.1;  // rad/s
    const double vA = v_positive;
    const double vB = v_negative;
    const double vC = v_positive;
    const double vD = v_negative;
    const double vE = v_positive;
    const double vF = v_negative;
    const double vG = v_positive;
    x << qA, qB, qC, qD, qE, qF, qG, vA, vB, vC, vD, vE, vF, vG;

    return x;
  }

  // Computes the analytical Jacobian Jq_WPi for a set of points Pi moving with
  // the end effector frame E, given their (fixed) position p_EPi in the end
  // effector frame.
  // This templated helper method allows us to use automatic differentiation.
  // See MultibodyTree::CalcPointsAnalyticalJacobianExpressedInWorld() for
  // details.
  // TODO(amcastro-tri): Rename this method as per issue #10155.
  template <typename T>
  void CalcPointsOnEndEffectorAnalyticJacobian(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T,
      const MatrixX<T>& p_EPi,
      MatrixX<T>* p_WPi, MatrixX<T>* Jq_WPi) const {
    const Body<T>& linkG_on_T =
        plant_on_T.tree().get_variant(*end_effector_link_);
    plant_on_T.tree().CalcPointsAnalyticalJacobianExpressedInWorld(
        context_on_T, linkG_on_T.body_frame(), p_EPi, p_WPi, Jq_WPi);
  }

 protected:
  // Problem sizes.
  const int kNumJoints = 7;
  const int kNumPositions = 14;
  const int kNumVelocities = 13;

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidTransform<double> X_GH_{
      RollPitchYaw<double>{M_PI_2, 0, M_PI_2}.ToRotationMatrix(),
      Vector3d{0, 0, 0.081}};
  const Body<double>* end_effector_link_{nullptr};
  const Frame<double>* frame_H_{nullptr};

  // AutoDiffXd model to compute automatic derivatives:
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

TEST_F(KukaIiwaModelTests, FixtureInvariants) {
  // Sanity check basic invariants.
  // Seven dofs for the arm plus floating base.
  EXPECT_EQ(plant_->num_joints(), kNumJoints);
  EXPECT_EQ(plant_->num_positions(), kNumPositions);
  EXPECT_EQ(plant_->num_velocities(), kNumVelocities);
}

// TODO(amcastro-tri): Rename this test as per issue #10155.
TEST_F(KukaIiwaModelTests, CalcPointsAnalyticalJacobianExpressedInWorld) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  SetArbitraryConfiguration();

  // A set of points Pi fixed in the end effector frame E.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_EPi(3, kNumPoints);
  p_EPi.col(0) << 0.1, -0.05, 0.02;
  p_EPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_WPi(3, kNumPoints);
  MatrixX<double> Jq_WPi(3 * kNumPoints, plant_->num_positions());

  CalcPointsOnEndEffectorAnalyticJacobian(
      *plant_, *context_, p_EPi, &p_WPi, &Jq_WPi);

  // Alternatively, compute the analytic Jacobian by taking the gradient of
  // the positions p_WPi(q) with respect to the generalized positions. We do
  // that with the steps below.

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(plant_->num_positions());
  auto q_double = plant_->GetPositions(*context_);
  math::initializeAutoDiff(q_double, q_autodiff);
  plant_autodiff_->GetMutablePositions(context_autodiff_.get()) = q_autodiff;

  const MatrixX<AutoDiffXd> p_EPi_autodiff = p_EPi;
  MatrixX<AutoDiffXd> p_WPi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> Jq_WPi_autodiff(3 * kNumPoints, plant_->num_positions());

  CalcPointsOnEndEffectorAnalyticJacobian(
      *plant_autodiff_, *context_autodiff_,
      p_EPi_autodiff, &p_WPi_autodiff, &Jq_WPi_autodiff);

  // Extract values and derivatives:
  const Matrix3X<double> p_WPi_value =
      math::autoDiffToValueMatrix(p_WPi_autodiff);
  const MatrixX<double> p_WPi_derivs =
      math::autoDiffToGradientMatrix(p_WPi_autodiff);

  // Some sanity checks:
  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(p_WPi_value, p_WPi,
                              kTolerance, MatrixCompareType::relative));
  // Sizes of the derivatives.
  EXPECT_EQ(p_WPi_derivs.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WPi_derivs.cols(), plant_->num_positions());

  // Verify the computed Jacobian Jq_WPi matches the one obtained using
  // automatic differentiation.
  EXPECT_TRUE(CompareMatrices(Jq_WPi, p_WPi_derivs,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocity) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  SetArbitraryConfiguration();

  // A point P fixed in the end effector frame E.
  Vector3<double> p_EP{0.1, -0.05, 0.02};

  MatrixX<double> Jq_WEp(6, plant_->num_positions());

  // Compute Jacobian using the method under test.
  plant_->tree().CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kQDot,
      end_effector_link_->body_frame(), p_EP, plant_->world_frame(),
      plant_->world_frame(), &Jq_WEp);

  // Alternatively, compute the Jacobian by taking the gradient of the spatial
  // velocity V_WEp with respect to q̇, since V_WEp = Jq_WEP * q̇. We do that with
  // the steps below.

  // Initialize q̇ to have zero values and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> qdot_autodiff =
      math::initializeAutoDiff(VectorX<double>::Zero(kNumPositions));
  auto q_double = plant_->GetPositions(*context_);
  VectorX<AutoDiffXd> v_autodiff(kNumVelocities);
  // Update the context with the position values from `context_`.
  plant_autodiff_->SetPositions(context_autodiff_.get(),
                                q_double.cast<AutoDiffXd>());
  // Set the velocity values in the context using q̇.
  plant_autodiff_->tree().MapQDotToVelocity(*context_autodiff_, qdot_autodiff,
                                            &v_autodiff);
  plant_autodiff_->SetVelocities(context_autodiff_.get(), v_autodiff);

  // Compute V_WEp.
  const Body<AutoDiffXd>& end_effector_link_autodiff =
      plant_autodiff_->tree().get_variant(*end_effector_link_);
  const Isometry3<AutoDiffXd> X_WE_autodiff =
      plant_autodiff_->tree().EvalBodyPoseInWorld(*context_autodiff_,
                                                  end_effector_link_autodiff);
  const Vector3<AutoDiffXd> p_EP_W =
      X_WE_autodiff.linear() * p_EP.cast<AutoDiffXd>();
  const SpatialVelocity<AutoDiffXd> V_WEp_autodiff =
      plant_autodiff_->tree()
          .EvalBodySpatialVelocityInWorld(*context_autodiff_,
                                          end_effector_link_autodiff)
          .Shift(p_EP_W);

  // Extract the Jacobian generated by automatic differentiation.
  MatrixX<double> Jq_WEp_autodiff =
      math::autoDiffToGradientMatrix(V_WEp_autodiff.get_coeffs());

  // Verify the Jacobian Jq_WEp compupted by the method under test matches the
  // one obtained using automatic differentiation.
  EXPECT_TRUE(CompareMatrices(Jq_WEp, Jq_WEp_autodiff, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
