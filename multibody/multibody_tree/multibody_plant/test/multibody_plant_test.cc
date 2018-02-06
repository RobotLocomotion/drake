#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::kuka_iiwa_robot::MakeKukaIiwaPlant;
using systems::Context;
using systems::ContinuousState;

// This test creates a simple model for an acrobot using MultibodyPlant and
// verifies a number of invariants such as that body and joint models were
// properly added and the model sizes.
GTEST_TEST(MultibodyPlant, SimpleModelCreation) {
  const std::string kInvalidName = "InvalidName";

  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant = MakeAcrobotPlant(parameters);

  // MakeAcrobotPlant() has already called Finalize() on the new acrobot plant.
  // Therefore attempting to call this method again will throw an exception.
  // Verify this.
  EXPECT_THROW(plant->Finalize(), std::logic_error);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(plant->num_bodies(), 3);
  EXPECT_EQ(plant->num_joints(), 2);

  // State size.
  EXPECT_EQ(plant->num_positions(), 2);
  EXPECT_EQ(plant->num_velocities(), 2);
  EXPECT_EQ(plant->num_multibody_states(), 4);

  // Query if elements exist in the model.
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link1_name()));
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link2_name()));
  EXPECT_FALSE(plant->HasBodyNamed(kInvalidName));

  EXPECT_TRUE(plant->HasJointNamed(parameters.shoulder_joint_name()));
  EXPECT_TRUE(plant->HasJointNamed(parameters.elbow_joint_name()));
  EXPECT_FALSE(plant->HasJointNamed(kInvalidName));

  // Get links by name.
  const Body<double>& link1 = plant->GetBodyByName(parameters.link1_name());
  EXPECT_EQ(link1.get_name(), parameters.link1_name());
  const Body<double>& link2 = plant->GetBodyByName(parameters.link2_name());
  EXPECT_EQ(link2.get_name(), parameters.link2_name());

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(plant->GetBodyByName(kInvalidName), std::logic_error);

  // Get joints by name.
  const Joint<double>& shoulder_joint =
      plant->GetJointByName(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder_joint.get_name(), parameters.shoulder_joint_name());
  const Joint<double>& elbow_joint =
      plant->GetJointByName(parameters.elbow_joint_name());
  EXPECT_EQ(elbow_joint.get_name(), parameters.elbow_joint_name());
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Templatized version to obtain retrieve a particular known type of joint.
  const RevoluteJoint<double>& shoulder =
      plant->GetJointByName<RevoluteJoint>(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder.get_name(), parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      plant->GetJointByName<RevoluteJoint>(parameters.elbow_joint_name());
  EXPECT_EQ(elbow.get_name(), parameters.elbow_joint_name());
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // MakeAcrobotPlant() has already called Finalize() on the acrobot model.
  // Therefore no more modeling elements can be added. Verify this.
  EXPECT_THROW(plant->AddRigidBody("AnotherBody", SpatialInertia<double>()),
               std::logic_error);
  EXPECT_THROW(plant->AddJoint<RevoluteJoint>(
      "AnotherJoint", link1, {}, link2, {}, Vector3d::UnitZ()),
               std::logic_error);
  // TODO(amcastro-tri): add test to verify that requesting a joint of the wrong
  // type throws an exception. We need another joint type to do so.
}

// Fixture to perform a number of computational tests on an acrobot model.
class AcrobotPlantTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    plant_ = MakeAcrobotPlant(parameters_);
    link1_ = &plant_->GetBodyByName(parameters_.link1_name());
    link2_ = &plant_->GetBodyByName(parameters_.link2_name());
    shoulder_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.shoulder_joint_name());
    elbow_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.elbow_joint_name());

    context_ = plant_->CreateDefaultContext();
    derivatives_ = plant_->AllocateTimeDerivatives();
  }

  // Verifies the computation performed by MultibodyPlant::CalcTimeDerivatives()
  // for the acrobot model. The comparison is carried out against a benchmark
  // with hand written dynamics.
  void VerifyCalcTimeDerivatives(double theta1, double theta2,
                                 double theta1dot, double theta2dot) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    // Set the state:
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    shoulder_->set_angular_rate(context_.get(), theta1dot);
    elbow_->set_angular_rate(context_.get(), theta2dot);

    plant_->CalcTimeDerivatives(*context_, derivatives_.get());
    const VectorXd xdot = derivatives_->CopyToVector();

    // Now compute inverse dynamics using our benchmark:
    Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    Vector2d rhs = tau_g_expected - C_expected;
    Matrix2d M_expected = acrobot_benchmark_.CalcMassMatrix(theta2);
    Vector2d vdot_expected = M_expected.inverse() * rhs;
    VectorXd xdot_expected(4);
    xdot_expected << Vector2d(theta1dot, theta2dot), vdot_expected;

    EXPECT_TRUE(CompareMatrices(
        xdot, xdot_expected, kTolerance, MatrixCompareType::relative));
  }

 protected:
  // The parameters of the model:
  const AcrobotParameters parameters_;
  // The model plant:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  // Non-owning pointers to the model's elements:
  const Body<double>* link1_{nullptr};
  const Body<double>* link2_{nullptr};
  const RevoluteJoint<double>* shoulder_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      parameters_.m1(), parameters_.m2(),
      parameters_.l1(), parameters_.l2(),
      parameters_.lc1(), parameters_.lc2(),
      parameters_.Ic1(), parameters_.Ic2(),
      parameters_.b1(), parameters_.b2(),
      parameters_.g()};
};

// Verifies the correctness of MultibodyPlant::CalcTimeDerivatives() on a model
// of an acrobot.
TEST_F(AcrobotPlantTests, CalcTimeDerivatives) {
  // Some random tests with non-zero state:
  VerifyCalcTimeDerivatives(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0);                /* joint's angular rates */
  VerifyCalcTimeDerivatives(
      M_PI / 3.0, -M_PI / 5.0,  /* joint's angles */
      0.7, -1.0);               /* joint's angular rates */
  VerifyCalcTimeDerivatives(
      M_PI / 4.0, -M_PI / 3.0,  /* joint's angles */
      -0.5, 2.0);               /* joint's angular rates */
  VerifyCalcTimeDerivatives(
      -M_PI, -M_PI / 2.0,       /* joint's angles */
      -1.5, -2.5);              /* joint's angular rates */
}

// Fixture to perform a number of computational tests on a KUKA Iiwa model.
class KukaIiwaPlantTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    plant_ = MakeKukaIiwaPlant();

    // Keep pointers to the modeling elements.
    linkG_ = &plant_->GetBodyByName("iiwa_link_7");
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
    joints_.push_back(&plant_->GetJointByName<RevoluteJoint>("iiwa_joint_7"));

    context_ = plant_->CreateDefaultContext();

    // Scalar-convert the plant and create a default context for it.
    plant_autodiff_ = systems::System<double>::ToAutoDiffXd(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

  template <typename T>
  Vector3<T> CalcEndEffectorVelocity(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T) const {
    std::vector<SpatialVelocity<T>> V_WB_array;
    plant_on_T.CalcAllBodySpatialVelocitiesInWorld(context_on_T, &V_WB_array);
    return V_WB_array[linkG_->get_index()].translational();
  }

  template <typename T>
  Vector3<T> CalcEndEffectorPosition(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T) const {
    const Body<T>& linkG_on_T = plant_on_T.GetBodyByName("iiwa_link_7");
    Vector3<T> p_NG;
    plant_on_T.CalcPointsPositions(
        context_on_T, linkG_on_T.get_body_frame(),
        Vector3<T>::Zero(),  // position in frame G
        plant_on_T.get_world_body().get_body_frame(), &p_NG);
    return p_NG;
  }

 protected:
  // The model plant:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  // Non-owning pointer to the end effector link:
  const Body<double>* linkG_{nullptr};
  // Non-owning pointers to the joints:
  std::vector<const RevoluteJoint<double>*> joints_;

  // AutoDiffXd model to compute automatic derivatives:
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
};

// This test is used to verify the correctness of the method
// MultibodyPlant::CalcPointsGeometricJacobianExpressedInWorld().
// The test computes the end effector geometric Jacobian Jg_NG (in the
// Newtonian, world, frame N) using two methods:
// 1. Calling MultibodyPlant::CalcPointsGeometricJacobianExpressedInWorld().
// 2. Using AutoDiffXd to compute the partial derivative of v_NG(q, v) with
//    respect to v.
// By comparing the two results we verify the correctness of the MultibodyTree
// implementation.
// In addition, we are testing methods:
// - MultibodyPlant::CalcPointsPositions()
// - MultibodyPlant::CalcAllBodySpatialVelocitiesInWorld()
TEST_F(KukaIiwaPlantTests, GeometricJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = plant_->num_positions();
  const int kNumStates = plant_->num_multibody_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(plant_autodiff_->num_positions(), kNumPositions);
  ASSERT_EQ(plant_autodiff_->num_multibody_states(), kNumStates);

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
  Vector3<double> v_NG = CalcEndEffectorVelocity(*plant_, *context_);

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize v_autodiff to have values v and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v_autodiff(kNumPositions);
  math::initializeAutoDiff(v, v_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v_autodiff);

  const Vector3<AutoDiffXd> v_NG_autodiff =
      CalcEndEffectorVelocity(*plant_autodiff_, *context_autodiff_);

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
  Matrix3X<double> Jg_NG(3, plant_->num_velocities());
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  plant_->CalcPointsGeometricJacobianExpressedInWorld(
      *context_, linkG_->get_body_frame(),
      Vector3<double>::Zero(), &p_NG, &Jg_NG);

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(Jg_NG, v_NG_derivs,
                              kTolerance, MatrixCompareType::relative));

  // Verify that v_NG = Jg_NG * v:
  const Vector3<double> J_NG_times_v = Jg_NG * v;
  EXPECT_TRUE(CompareMatrices(J_NG_times_v, v_NG,
                              kTolerance, MatrixCompareType::relative));

  // Verify that CalcPointsPositions() computes the same value of p_NG.
  Vector3<double> p2_NG = CalcEndEffectorPosition(*plant_, *context_);
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
      *plant_autodiff_, *context_autodiff_);
  Vector3<double> p_NG_derivs(
      p_NG_autodiff[0].derivatives()[0],
      p_NG_autodiff[1].derivatives()[0],
      p_NG_autodiff[2].derivatives()[0]);
  EXPECT_TRUE(CompareMatrices(p_NG_derivs, v_NG,
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

