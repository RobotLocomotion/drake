#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/test_utilities/expect_error_message.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace multibody {
namespace multibody_model {
namespace {

using Eigen::MatrixXd;
using multibody::benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel;
using systems::Context;
using systems::ContinuousState;

// This test creates a model for a KUKA Iiiwa arm and verifies we can retrieve
// multibody elements by name or get exceptions accordingly.
GTEST_TEST(MultibodyTree, RetrieveNamedElements) {
  const std::string kInvalidName = "InvalidName";
  const std::vector<std::string> kLinkNames = {
      "iiwa_link_1",
      "iiwa_link_2",
      "iiwa_link_3",
      "iiwa_link_4",
      "iiwa_link_5",
      "iiwa_link_6",
      "iiwa_link_7"};
  const std::vector<std::string> kJointNames = {
      "iiwa_joint_1",
      "iiwa_joint_2",
      "iiwa_joint_3",
      "iiwa_joint_4",
      "iiwa_joint_5",
      "iiwa_joint_6",
      "iiwa_joint_7"};

  // Create a non-finalized model of the arm so that we can test adding more
  // elements to it.
  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(false /* non-finalized model. */);

  // Verify the model was not finalized.
  EXPECT_FALSE(model->topology_is_valid());

  // Attempt to add a body having the same name as a body already part of the
  // model. This is not allowed and an exception should be thrown.
  EXPECT_ERROR_MESSAGE(
      model->AddRigidBody("iiwa_link_5", SpatialInertia<double>()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "This model already contains a body named 'iiwa_link_5'. "
      "Body names must be unique within a given model.");

  // Attempt to add a joint having the same name as a joint already part of the
  // model. This is not allowed and an exception should be thrown.
  EXPECT_ERROR_MESSAGE(
      model->AddJoint<RevoluteJoint>(
          "iiwa_joint_4",
          /* Dummy frame definitions. Not relevant for this test. */
          model->get_world_body(), {},
          model->get_world_body(), {},
          Vector3<double>::UnitZ()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "This model already contains a joint named 'iiwa_joint_4'. "
      "Joint names must be unique within a given model.");

  // Now we tested we cannot add body or joints with an existing name, finalize
  // the model.
  EXPECT_NO_THROW(model->Finalize());

  // Another call to Finalize() is not allowed.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(model->get_num_bodies(), 8);  // It includes the "world" body.
  EXPECT_EQ(model->get_num_joints(), 7);

  // State size.
  EXPECT_EQ(model->get_num_positions(), 7);
  EXPECT_EQ(model->get_num_velocities(), 7);
  EXPECT_EQ(model->get_num_states(), 14);

  // Query if elements exist in the model.
  for (const std::string link_name : kLinkNames) {
    EXPECT_TRUE(model->HasBodyNamed(link_name));
  }
  EXPECT_FALSE(model->HasBodyNamed(kInvalidName));

  for (const std::string joint_name : kJointNames) {
    EXPECT_TRUE(model->HasJointNamed(joint_name));
  }
  EXPECT_FALSE(model->HasJointNamed(kInvalidName));

  // Get links by name.
  for (const std::string link_name : kLinkNames) {
    const Body<double>& link = model->GetBodyByName(link_name);
    EXPECT_EQ(link.get_name(), link_name);
  }
  EXPECT_ERROR_MESSAGE(
      model->GetBodyByName(kInvalidName), std::logic_error,
      "There is no body named '.*' in the model.");

  // Get joints by name.
  for (const std::string joint_name : kJointNames) {
    const Joint<double>& joint = model->GetJointByName(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  EXPECT_ERROR_MESSAGE(
      model->GetJointByName(kInvalidName), std::logic_error,
      "There is no joint named '.*' in the model.");

  // Templatized version to obtain retrieve a particular known type of joint.
  for (const std::string joint_name : kJointNames) {
    const RevoluteJoint<double>& joint =
        model->GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  EXPECT_ERROR_MESSAGE(
      model->GetJointByName<RevoluteJoint>(kInvalidName), std::logic_error,
      "There is no joint named '.*' in the model.");
}

// Fixture to perform a number of computational tests on a KUKA Iiwa model.
class KukaIiwaModelTests : public ::testing::Test {
 public:
  // Creates MultibodyTree for an acrobot model.
  void SetUp() override {
    model_ = MakeKukaIiwaModel<double>();

    // Keep pointers to the modeling elements.
    end_effector_link_ = &model_->GetBodyByName("iiwa_link_7");
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_7"));

    context_ = model_->CreateDefaultContext();

    // Scalar-convert the model and create a default context for it.
    model_autodiff_ = model_->ToAutoDiffXd();
    context_autodiff_ = model_autodiff_->CreateDefaultContext();
  }

  template <typename T>
  Vector3<T> CalcEndEffectorVelocity(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T) const {
    std::vector<SpatialVelocity<T>> V_WB_array;
    model_on_T.CalcAllBodySpatialVelocitiesInWorld(context_on_T, &V_WB_array);
    return V_WB_array[end_effector_link_->get_index()].translational();
  }

  template <typename T>
  Vector3<T> CalcEndEffectorPosition(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T) const {
    const Body<T>& linkG_on_T = model_on_T.get_variant(*end_effector_link_);
    Vector3<T> p_NG;
    model_on_T.CalcPointsPositions(
        context_on_T, linkG_on_T.get_body_frame(),
        Vector3<T>::Zero(),  // position in frame G
        model_on_T.get_world_body().get_body_frame(), &p_NG);
    return p_NG;
  }

  template <typename T>
  void CalcPointsOnEndEffectorGeometricJacobian(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T,
      const MatrixX<T>& p_GPi,
      MatrixX<T>* p_NGpi, MatrixX<T>* J_NGpi) const {
    const Body<T>& linkG_on_T = model_on_T.get_variant(*end_effector_link_);
    model_on_T.CalcPointsGeometricJacobianExpressedInWorld(
        context_on_T, linkG_on_T.get_body_frame(), p_GPi, p_NGpi, J_NGpi);
  }

 protected:
  // The model plant:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  // Non-owning pointer to the end effector link:
  const Body<double>* end_effector_link_{nullptr};
  // Non-owning pointers to the joints:
  std::vector<const RevoluteJoint<double>*> joints_;

  // AutoDiffXd model to compute automatic derivatives:
  std::unique_ptr<MultibodyTree<AutoDiffXd>> model_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

// This test is used to verify the correctness of the method
// MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// The test computes the end effector geometric Jacobian Jg_NG (in the
// Newtonian, world, frame N) using two methods:
// 1. Calling MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// 2. Using AutoDiffXd to compute the partial derivative of v_NG(q, v) with
//    respect to v.
// By comparing the two results we verify the correctness of the MultibodyTree
// implementation.
// In addition, we are testing methods:
// - MultibodyTree::CalcPointsPositions()
// - MultibodyTree::CalcAllBodySpatialVelocitiesInWorld()
TEST_F(KukaIiwaModelTests, GeometricJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = model_->get_num_positions();
  const int kNumStates = model_->get_num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(model_autodiff_->get_num_positions(), kNumPositions);
  ASSERT_EQ(model_autodiff_->get_num_states(), kNumStates);

  ASSERT_EQ(context_->get_continuous_state().size(), kNumStates);
  ASSERT_EQ(context_autodiff_->get_continuous_state().size(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

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
  VectorX<double> q(kNumPositions);
  q << qA, qB, qC, qD, qE, qF, qG;

  // A non-zero set of values for the joint's velocities.
  const double vA = 0.1;
  const double vB = 0.2;
  const double vC = 0.3;
  const double vD = 0.4;
  const double vE = 0.5;
  const double vF = 0.6;
  const double vG = 0.7;
  VectorX<double> v(kNumPositions);
  v << vA, vB, vC, vD, vE, vF, vG;

  // Zero generalized positions and velocities.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Compute the value of the end effector's velocity using <double>.
  Vector3<double> v_NG = CalcEndEffectorVelocity(*model_, *context_);

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize v_autodiff to have values v and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v_autodiff(kNumPositions);
  math::initializeAutoDiff(v, v_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v_autodiff);

  const Vector3<AutoDiffXd> v_NG_autodiff =
      CalcEndEffectorVelocity(*model_autodiff_, *context_autodiff_);

  const Vector3<double> v_NG_value = math::autoDiffToValueMatrix(v_NG_autodiff);
  const MatrixX<double> v_NG_derivs =
      math::autoDiffToGradientMatrix(v_NG_autodiff);

  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(v_NG_value, v_NG,
                              kTolerance, MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_NG_derivs.rows(), 3);
  EXPECT_EQ(v_NG_derivs.cols(), kNumPositions);

  Vector3<double> p_NG;
  Matrix3X<double> Jg_NG(3, model_->get_num_velocities());
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  model_->CalcPointsGeometricJacobianExpressedInWorld(
      *context_, end_effector_link_->get_body_frame(),
      Vector3<double>::Zero(), &p_NG, &Jg_NG);

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(Jg_NG, v_NG_derivs,
                              kTolerance, MatrixCompareType::relative));

  // Verify that v_NG = Jg_NG * v:
  const Vector3<double> J_NG_times_v = Jg_NG * v;
  EXPECT_TRUE(CompareMatrices(J_NG_times_v, v_NG,
                              kTolerance, MatrixCompareType::relative));

  // Verify that MultibodyTree::CalcPointsPositions() computes the same value
  // of p_NG. Even both code paths resolve to CalcPointsPositions(), here we
  // call this method explicitly to provide unit testing for this API.
  Vector3<double> p2_NG = CalcEndEffectorPosition(*model_, *context_);
  EXPECT_TRUE(CompareMatrices(p2_NG, p_NG,
                              kTolerance, MatrixCompareType::relative));

  // The derivative with respect to time should equal v_NG.
  const VectorX<AutoDiffXd> q_autodiff =
      // For reasons beyond my understanding, we need to pass MatrixXd to
      // math::initializeAutoDiffGivenGradientMatrix().
      math::initializeAutoDiffGivenGradientMatrix(MatrixXd(q), MatrixXd(v));
  v_autodiff = v.cast<AutoDiffXd>();
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v_autodiff);

  Vector3<AutoDiffXd> p_NG_autodiff = CalcEndEffectorPosition(
      *model_autodiff_, *context_autodiff_);
  Vector3<double> p_NG_derivs(
      p_NG_autodiff[0].derivatives()[0],
      p_NG_autodiff[1].derivatives()[0],
      p_NG_autodiff[2].derivatives()[0]);
  EXPECT_TRUE(CompareMatrices(p_NG_derivs, v_NG,
                              kTolerance, MatrixCompareType::relative));
}

// Given a set of points Pi attached to the end effector frame G, this test
// computes the analytic Jacobian J_NGpi of these points using two methods:
// 1. Since for the Kuka iiwa arm v = q̇, the analytic Jacobian equals the
//    geometric Jacobian and we compute it with MultibodyTree's implementation.
// 2. We compute the analytic Jacobian by direct differentiation with respect to
//    q using AutoDiffXd.
// We then verify MultibodyTree's implementation by comparing the results from
// both methods.
TEST_F(KukaIiwaModelTests, AnalyticJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = 7;

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

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
  VectorX<double> q0(kNumPositions);
  q0 << qA, qB, qC, qD, qE, qF, qG;

  context_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q0);

  // A set of points Pi attached to the end effector, thus we a fixed position
  // in its frame G.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_GPi(3, kNumPoints);
  p_GPi.col(0) << 0.1, -0.05, 0.02;
  p_GPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_NGpi(3, kNumPoints);
  MatrixX<double> J_NGpi(3 * kNumPoints, kNumPositions);

  // Since for the Kuka iiwa arm v = q̇, the analytic Jacobian equals the
  // geometric Jacobian.
  CalcPointsOnEndEffectorGeometricJacobian(
      *model_, *context_, p_GPi, &p_NGpi, &J_NGpi);

  // Alternatively, compute the analytic Jacobian by taking the gradient of
  // the positions p_NGpi(q) with respect to the generalized positions. We do
  // that with the steps below.

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(kNumPositions);
  math::initializeAutoDiff(q0, q_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q_autodiff);

  const MatrixX<AutoDiffXd> p_GPi_autodiff = p_GPi;
  MatrixX<AutoDiffXd> p_NGpi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> J_NGpi_autodiff(3 * kNumPoints, kNumPositions);

  CalcPointsOnEndEffectorGeometricJacobian(
      *model_autodiff_, *context_autodiff_,
      p_GPi_autodiff, &p_NGpi_autodiff, &J_NGpi_autodiff);

  // Extract values and derivatives:
  const Matrix3X<double> p_NGpi_value =
      math::autoDiffToValueMatrix(p_NGpi_autodiff);
  const MatrixX<double> p_NGpi_derivs =
      math::autoDiffToGradientMatrix(p_NGpi_autodiff);

  // Some sanity checks:
  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(p_NGpi_value, p_NGpi,
                              kTolerance, MatrixCompareType::relative));
  // Sizes of the derivatives.
  EXPECT_EQ(p_NGpi_derivs.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_NGpi_derivs.cols(), kNumPositions);

  // Verify the computed Jacobian J_NGpi matches the one obtained using
  // automatic differentiation.
  // In this case analytic and geometric Jacobians are equal since v = q.
  EXPECT_TRUE(CompareMatrices(J_NGpi, p_NGpi_derivs,
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody_model
}  // namespace multibody
}  // namespace drake

