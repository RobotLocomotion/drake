#include "drake/multibody/tree/multibody_tree.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_mobilizer.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {
namespace multibody_model {
namespace {

using benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel;
using benchmarks::kuka_iiwa_robot::MG::MGKukaIIwaRobot;
using Eigen::AngleAxisd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RollPitchYaw;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using test_utilities::SpatialKinematicsPVA;

// Helper method to verify the integrity of a MultibodyTree model of a Kuka iiwa
// arm.
template <typename T>
void VerifyModelBasics(const MultibodyTree<T>& model) {
  const std::string kInvalidName = "InvalidName";
  const std::vector<std::string> kLinkNames = {
      "iiwa_link_1",
      "iiwa_link_2",
      "iiwa_link_3",
      "iiwa_link_4",
      "iiwa_link_5",
      "iiwa_link_6",
      "iiwa_link_7"};

  const std::vector<std::string> kFrameNames = {
      "iiwa_link_1",
      "iiwa_link_2",
      "iiwa_link_3",
      "iiwa_link_4",
      "iiwa_link_5",
      "iiwa_link_6",
      "iiwa_link_7",
      "tool_arbitrary"};

  const std::vector<std::string> kJointNames = {
      "iiwa_joint_1",
      "iiwa_joint_2",
      "iiwa_joint_3",
      "iiwa_joint_4",
      "iiwa_joint_5",
      "iiwa_joint_6",
      "iiwa_joint_7"};

  const std::vector<std::string> kActuatorNames = {
      "iiwa_actuator_1",
      "iiwa_actuator_2",
      "iiwa_actuator_3",
      "iiwa_actuator_4",
      "iiwa_actuator_5",
      "iiwa_actuator_6",
      "iiwa_actuator_7"};

  // Model Size. Counting the world body, there should be eight bodies.
  EXPECT_EQ(model.num_bodies(), 8);  // It includes the "world" body.
  EXPECT_EQ(model.num_joints(), 7);
  EXPECT_EQ(model.num_actuators(), 7);
  EXPECT_EQ(model.num_actuated_dofs(), 7);

  // State size.
  EXPECT_EQ(model.num_positions(), 7);
  EXPECT_EQ(model.num_velocities(), 7);
  EXPECT_EQ(model.num_states(), 14);

  // Query if elements exist in the model.
  for (const std::string link_name : kLinkNames) {
    EXPECT_TRUE(model.HasBodyNamed(link_name));
  }
  EXPECT_FALSE(model.HasBodyNamed(kInvalidName));

  for (const std::string frame_name : kFrameNames) {
    EXPECT_TRUE(model.HasFrameNamed(frame_name));
  }
  EXPECT_FALSE(model.HasFrameNamed(kInvalidName));

  for (const std::string joint_name : kJointNames) {
    EXPECT_TRUE(model.HasJointNamed(joint_name));
  }
  EXPECT_FALSE(model.HasJointNamed(kInvalidName));

  for (const std::string actuator_name : kActuatorNames) {
    EXPECT_TRUE(model.HasJointActuatorNamed(actuator_name));
  }
  EXPECT_FALSE(model.HasJointActuatorNamed(kInvalidName));

  // Get links by name.
  for (const std::string link_name : kLinkNames) {
    const Body<T>& link = model.GetBodyByName(link_name);
    EXPECT_EQ(link.name(), link_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetBodyByName(kInvalidName), std::logic_error,
      "There is no body named '.*' in the model.");

  // Test we can also retrieve links as RigidBody objects.
  for (const std::string link_name : kLinkNames) {
    const RigidBody<T>& link = model.GetRigidBodyByName(link_name);
    EXPECT_EQ(link.name(), link_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetRigidBodyByName(kInvalidName), std::logic_error,
      "There is no body named '.*' in the model.");

  // Get frames by name.
  for (const std::string frame_name : kFrameNames) {
    const Frame<T>& frame = model.GetFrameByName(frame_name);
    EXPECT_EQ(frame.name(), frame_name);
    EXPECT_EQ(
        &frame, &model.GetFrameByName(frame_name, default_model_instance()));
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetFrameByName(kInvalidName), std::logic_error,
      "There is no frame named '.*' in the model.");

  // Get joints by name.
  for (const std::string joint_name : kJointNames) {
    const Joint<T>& joint = model.GetJointByName(joint_name);
    EXPECT_EQ(joint.name(), joint_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetJointByName(kInvalidName), std::logic_error,
      "There is no joint named '.*' in the model.");

  // Templatized version to obtain retrieve a particular known type of joint.
  for (const std::string joint_name : kJointNames) {
    const RevoluteJoint<T>& joint =
        model.template GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_EQ(joint.name(), joint_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.template GetJointByName<RevoluteJoint>(kInvalidName),
      std::logic_error, "There is no joint named '.*' in the model.");

  // Get actuators by name.
  for (const std::string actuator_name : kActuatorNames) {
    const JointActuator<T>& actuator =
        model.GetJointActuatorByName(actuator_name);
    EXPECT_EQ(actuator.name(), actuator_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetJointActuatorByName(kInvalidName), std::logic_error,
      "There is no joint actuator named '.*' in the model.");

  // Test we can retrieve joints from the actuators.
  int names_index = 0;
  for (const std::string actuator_name : kActuatorNames) {
    const JointActuator<T>& actuator =
        model.GetJointActuatorByName(actuator_name);
    // We added actuators and joints in the same order. Assert this before
    // making that assumption in the test that follows.
    const Joint<T>& joint = actuator.joint();
    ASSERT_EQ(static_cast<int>(actuator.index()),
              static_cast<int>(joint.index()));
    const std::string& joint_name = kJointNames[names_index];
    EXPECT_EQ(joint.name(), joint_name);
    ++names_index;
  }
}

// This test creates a model for a KUKA Iiiwa arm and verifies we can retrieve
// multibody elements by name or get exceptions accordingly.
GTEST_TEST(MultibodyTree, VerifyModelBasics) {
  // Create a non-finalized model of the arm so that we can test adding more
  // elements to it.
  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(false /* non-finalized model. */);

  // Verify the model was not finalized.
  EXPECT_FALSE(model->topology_is_valid());

  // Attempt to add a body having the same name as a body already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddRigidBody("iiwa_link_5", default_model_instance(),
                          SpatialInertia<double>()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      ".* already contains a body named 'iiwa_link_5'. "
      "Body names must be unique within a given model.");

  // Attempt to add a joint having the same name as a joint already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddJoint<RevoluteJoint>(
          "iiwa_joint_4",
          model->world_body(), nullopt,
          model->GetBodyByName("iiwa_link_5"), nullopt,
          Vector3<double>::UnitZ()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      ".* already contains a joint named 'iiwa_joint_4'. "
      "Joint names must be unique within a given model.");

  // Attempt to add a joint having the same name as a joint already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddJointActuator(
          "iiwa_actuator_4",
          model->GetJointByName("iiwa_joint_4")),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      ".* already contains a joint actuator named 'iiwa_actuator_4'. "
          "Joint actuator names must be unique within a given model.");

  // Now we tested we cannot add body or joints with an existing name, finalize
  // the model.
  EXPECT_NO_THROW(model->Finalize());

  // Another call to Finalize() is not allowed.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  VerifyModelBasics(*model);
}

// MBPlant provides most of the testing for MBTreeSystem. Here we just want
// to make sure that a badly-behaved derived class is notified.
class BadDerivedMBSystem : public MultibodyTreeSystem<double> {
 public:
  explicit BadDerivedMBSystem(bool double_finalize)
      : MultibodyTreeSystem<double>() {
    mutable_tree().AddBody<RigidBody>(SpatialInertia<double>());
    Finalize();
    if (double_finalize) {
      Finalize();
    }
  }

  // Make this accessible so we can intentionally call it after finalizing
  // and check the error message.
  using MultibodyTreeSystem<double>::mutable_tree;
};

GTEST_TEST(MultibodyTreeSystem, CatchBadBehavior) {
  // Create the internal tree and finalize the MBSystem correctly.
  BadDerivedMBSystem finalized(false);
  EXPECT_NO_THROW(finalized.mutable_tree());

  // Make the MBSystem behave badly.
  DRAKE_EXPECT_THROWS_MESSAGE(BadDerivedMBSystem(true), std::logic_error,
                              ".*Finalize().*repeated.*not allowed.*");

  auto model = std::make_unique<MultibodyTree<double>>();
  EXPECT_NO_THROW(MultibodyTreeSystem<double>(std::move(model)));
  EXPECT_EQ(model, nullptr);  // Should have been moved from.

  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyTreeSystem<double>(std::move(model)), std::logic_error,
      ".*MultibodyTreeSystem().*MultibodyTree was null.*");
}

GTEST_TEST(MultibodyTree, BackwardsCompatibility) {
  auto owned_tree = std::make_unique<MultibodyTree<double>>();
  auto* tree = owned_tree.get();
  DRAKE_EXPECT_THROWS_MESSAGE(
    tree->CreateDefaultContext(), std::runtime_error,
    ".*that is owned by a MultibodyPlant.*");
  MultibodyTreeSystem<double> system(std::move(owned_tree));
  EXPECT_NO_THROW(tree->CreateDefaultContext());
}

// Fixture to perform a number of computational tests on a KUKA Iiwa model.
class KukaIiwaModelTests : public ::testing::Test {
 public:
  // Creates MultibodyTree for a KUKA Iiwa robot arm.
  void SetUp() override {
    {
      // We limit the scope for this MultibodyTree since it only is used for the
      // creation of the model. Ownership is later transferred to system_, which
      // is responsible for context resources.
      auto tree = MakeKukaIiwaModel<double>(
          false /* do not finalize model yet */, gravity_);

      // Keep pointers to the modeling elements.
      end_effector_link_ = &tree->GetBodyByName("iiwa_link_7");
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_7"));

      // Add a frame H with a fixed pose X_GH in the end effector frame G.
      // Note: frame names are documented in MakeKukaIiwaModel().
      frame_H_ = &tree->AddFrame<FixedOffsetFrame>(
          "H", *end_effector_link_, X_GH_);

      // Create a system to manage context resources.
      system_ =
          std::make_unique<MultibodyTreeSystem<double>>(std::move(tree));
    }

    context_ = system_->CreateDefaultContext();

    EXPECT_NO_THROW(context_->Clone());

    // Scalar-convert the model and create a default context for it.
    system_autodiff_ = std::make_unique<MultibodyTreeSystem<AutoDiffXd>>(
        tree().ToAutoDiffXd());
    context_autodiff_ = system_autodiff_->CreateDefaultContext();
  }

  // Gets an arm state to an arbitrary configuration in which joint angles and
  // rates are non-zero.
  void GetArbitraryNonZeroConfiguration(
      VectorX<double>* q, VectorX<double>* v) {
    const int kNumPositions = tree().num_positions();
    q->resize(kNumPositions);
    v->resize(kNumPositions);  // q and v have the same dimension for kuka.

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
    *q << qA, qB, qC, qD, qE, qF, qG;

    // A non-zero set of values for the joint's velocities.
    const double vA = 0.1;
    const double vB = 0.2;
    const double vC = 0.3;
    const double vD = 0.4;
    const double vE = 0.5;
    const double vF = 0.6;
    const double vG = 0.7;
    *v << vA, vB, vC, vD, vE, vF, vG;
  }

  // Computes the translational velocity `v_WE` of the end effector frame E in
  // the world frame W.
  template <typename T>
  Vector3<T> CalcEndEffectorVelocity(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T) const {
    std::vector<SpatialVelocity<T>> V_WB_array;
    model_on_T.CalcAllBodySpatialVelocitiesInWorld(context_on_T, &V_WB_array);
    return V_WB_array[end_effector_link_->index()].translational();
  }

  // Computes spatial velocity `V_WE` of the end effector frame E in the world
  // frame W.
  template <typename T>
  SpatialVelocity<T> CalcEndEffectorSpatialVelocity(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T) const {
    std::vector<SpatialVelocity<T>> V_WB_array;
    model_on_T.CalcAllBodySpatialVelocitiesInWorld(context_on_T, &V_WB_array);
    return V_WB_array[end_effector_link_->index()];
  }

  // Computes p_WEo, the position of the end effector frame's origin Eo.
  template <typename T>
  Vector3<T> CalcEndEffectorPosition(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T) const {
    const Body<T>& linkG_on_T = model_on_T.get_variant(*end_effector_link_);
    Vector3<T> p_WE;
    model_on_T.CalcPointsPositions(
        context_on_T, linkG_on_T.body_frame(),
        Vector3<T>::Zero(),  // position in frame G
        model_on_T.world_body().body_frame(), &p_WE);
    return p_WE;
  }

  // Computes the geometric Jacobian Jv_WPi for a set of points Pi moving with
  // the end effector frame E, given their (fixed) position p_WPi in the end
  // effector frame.
  // See MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld() for
  // details.
  template <typename T>
  void CalcPointsOnEndEffectorGeometricJacobian(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T,
      const MatrixX<T>& p_EPi,
      MatrixX<T>* p_WPi, MatrixX<T>* Jv_WPi) const {
    const Body<T>& linkG_on_T = model_on_T.get_variant(*end_effector_link_);
    model_on_T.CalcPointsGeometricJacobianExpressedInWorld(
        context_on_T, linkG_on_T.body_frame(), p_EPi, p_WPi, Jv_WPi);
  }

  // Computes the geometric Jacobian Jv_WHp for a set of points Pi moving with
  // frame H with fixed pose X_GH in the end effector frame G, given their
  // (fixed) position p_HPi.
  // See MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld() for
  // details.
  template <typename T>
  void CalcPointsOnFrameHGeometricJacobian(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T,
      const MatrixX<T>& p_HPi,
      MatrixX<T>* p_WPi, MatrixX<T>* Jv_WHp) const {
    const Frame<T>& frameH_on_T = model_on_T.get_variant(*frame_H_);
    model_on_T.CalcPointsGeometricJacobianExpressedInWorld(
        context_on_T, frameH_on_T, p_HPi, p_WPi, Jv_WHp);
  }

  // Computes the frame geometric Jacobian Jv_WHp for frame Hp which is the
  // frame H (attached to the end effector, see test fixture docs) shifted to
  // have its origin at Po. Po's position p_HPo is specified in frame H.
  // See MultibodyTree::CalcFrameGeometricJacobianExpressedInWorld() for
  // details.
  template <typename T>
  void CalcFrameHpGeometricJacobian(
      const MultibodyTree<T>& model_on_T,
      const Context<T>& context_on_T,
      const Vector3<T>& p_HPo, MatrixX<T>* Jv_WHp) const {
    const Frame<T>& frameH_on_T = model_on_T.get_variant(*frame_H_);
    model_on_T.CalcFrameGeometricJacobianExpressedInWorld(
        context_on_T, frameH_on_T, p_HPo, Jv_WHp);
  }

  const MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

  const MultibodyTree<AutoDiffXd>& tree_autodiff() const {
    return internal::GetInternalTree(*system_autodiff_);
  }

 protected:
  // Acceleration of gravity:
  const double gravity_{9.81};
  // The model plant:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  // Non-owning pointer to the end effector link:
  const Body<double>* end_effector_link_{nullptr};
  // Non-owning pointer to a fixed pose frame on the end effector link:
  const Frame<double>* frame_H_{nullptr};
  const RigidTransform<double> X_GH_{
      RollPitchYaw<double>{M_PI_2, 0, M_PI_2}.ToRotationMatrix(),
      Vector3d{0, 0, 0.081}};
  // Non-owning pointers to the joints:
  std::vector<const RevoluteJoint<double>*> joints_;

  // AutoDiffXd model to compute automatic derivatives:
  std::unique_ptr<MultibodyTreeSystem<AutoDiffXd>> system_autodiff_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;

  // And independent benchmarking set of solutions.
  const MGKukaIIwaRobot<double> benchmark_{gravity_};
};

// Verifies the integrity of a scalar converted MultibodyTree from <double> to
// <AutoDiffXd>.
TEST_F(KukaIiwaModelTests, VerifyScalarConversionToAutoDiffXd) {
  VerifyModelBasics(tree_autodiff());
}

// Verifies the integrity of a scalar converted MultibodyTree from <double> to
// <symbolic::Expression>.
TEST_F(KukaIiwaModelTests, VerifyScalarConversionToSymbolic) {
  auto dut = tree().CloneToScalar<symbolic::Expression>();
  VerifyModelBasics(*dut);
}

// This test is used to verify the correctness of the method
// MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// The test computes the end effector geometric Jacobian Jv_WE (in the world
// frame W) using two methods:
// 1. Calling MultibodyTree::CalcPointsGeometricJacobianExpressedInWorld().
// 2. Using AutoDiffXd to compute the partial derivative of v_WE(q, v) with
//    respect to v.
// By comparing the two results we verify the correctness of the MultibodyTree
// implementation.
// In addition, we are testing methods:
// - MultibodyTree::CalcPointsPositions()
// - MultibodyTree::CalcAllBodySpatialVelocitiesInWorld()
TEST_F(KukaIiwaModelTests, CalcPointsGeometricJacobianExpressedInWorld) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->get_continuous_state().size(), kNumStates);
  ASSERT_EQ(context_autodiff_->get_continuous_state().size(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Zero generalized positions and velocities.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Compute the value of the end effector's velocity using <double>.
  Vector3<double> v_WE = CalcEndEffectorVelocity(tree(), *context_);

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize v_autodiff to have values v and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v_autodiff(kNumPositions);
  math::initializeAutoDiff(v, v_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v_autodiff);

  const Vector3<AutoDiffXd> v_WE_autodiff =
      CalcEndEffectorVelocity(tree_autodiff(), *context_autodiff_);

  const Vector3<double> v_WE_value = math::autoDiffToValueMatrix(v_WE_autodiff);
  const MatrixX<double> v_WE_derivs =
      math::autoDiffToGradientMatrix(v_WE_autodiff);

  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(v_WE_value, v_WE,
                              kTolerance, MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_WE_derivs.rows(), 3);
  EXPECT_EQ(v_WE_derivs.cols(), kNumPositions);

  Vector3<double> p_WE;
  Matrix3X<double> Jv_WE(3, tree().num_velocities());
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  tree().CalcPointsGeometricJacobianExpressedInWorld(
      *context_, end_effector_link_->body_frame(),
      Vector3<double>::Zero(), &p_WE, &Jv_WE);

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(Jv_WE, v_WE_derivs,
                              kTolerance, MatrixCompareType::relative));

  // Verify that v_WE = Jv_WE * v:
  const Vector3<double> Jv_WE_times_v = Jv_WE * v;
  EXPECT_TRUE(CompareMatrices(Jv_WE_times_v, v_WE,
                              kTolerance, MatrixCompareType::relative));

  // Verify that MultibodyTree::CalcPointsPositions() computes the same value
  // of p_WE. Even both code paths resolve to CalcPointsPositions(), here we
  // call this method explicitly to provide unit testing for this API.
  Vector3<double> p2_WE = CalcEndEffectorPosition(tree(), *context_);
  EXPECT_TRUE(CompareMatrices(p2_WE, p_WE,
                              kTolerance, MatrixCompareType::relative));

  // The derivative with respect to time should equal v_WE.
  const VectorX<AutoDiffXd> q_autodiff =
      // For reasons beyond my understanding, we need to pass MatrixXd to
      // math::initializeAutoDiffGivenGradientMatrix().
      math::initializeAutoDiffGivenGradientMatrix(MatrixXd(q), MatrixXd(v));
  v_autodiff = v.cast<AutoDiffXd>();
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_velocity().SetFromVector(v_autodiff);

  Vector3<AutoDiffXd> p_WE_autodiff = CalcEndEffectorPosition(
      tree_autodiff(), *context_autodiff_);
  Vector3<double> p_WE_derivs(
      p_WE_autodiff[0].derivatives()[0],
      p_WE_autodiff[1].derivatives()[0],
      p_WE_autodiff[2].derivatives()[0]);
  EXPECT_TRUE(CompareMatrices(p_WE_derivs, v_WE,
                              kTolerance, MatrixCompareType::relative));
}

// Unit tests MBT::CalcBiasForPointsGeometricJacobianExpressedInWorld() using
// AutoDiffXd to compute time derivatives of the geometric Jacobian to obtain a
// reference solution.
TEST_F(KukaIiwaModelTests, CalcBiasForPointsGeometricJacobianExpressedInWorld) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Since the bias term is a function of q and v only (i.e. it is not a
  // function of vdot), we choose a set of arbitrary values for the generalized
  // accelerations. We do purposely set this values to be non-zero to stress
  // the fact that the bias term is not a function of vdot.
  VectorX<double> vdot(kNumPositions);
  vdot << 1, 2, 3, 4, 5, 6, 7;

  // Set generalized positions and velocities.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize q_autodiff and v_autodiff so that we differentiate with respect
  // to time.
  // Note: here we pass MatrixXd(v) so that the return gradient uses AutoDiffXd
  // (for which we do have explicit instantiations) instead of
  // AutoDiffScalar<Matrix1d>.
  auto q_autodiff = math::initializeAutoDiffGivenGradientMatrix(q, MatrixXd(v));
  auto v_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(v, MatrixXd(vdot));

  VectorX<AutoDiffXd> x_autodiff(2 * kNumPositions);
  x_autodiff << q_autodiff, v_autodiff;

  // Set the context for AutoDiffXd computations.
  tree_autodiff().GetMutablePositionsAndVelocities(context_autodiff_.get())
      = x_autodiff;

  // A set of points Pi attached to frame H on the end effector.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_HPi(3, kNumPoints);
  p_HPi.col(0) << 0.1, -0.05, 0.02;
  p_HPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_WPi(3, kNumPoints);
  MatrixX<double> Jv_WHp(3 * kNumPoints, kNumPositions);

  const MatrixX<AutoDiffXd> p_HPi_autodiff = p_HPi;
  MatrixX<AutoDiffXd> p_WPi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> Jv_WHp_autodiff(3 * kNumPoints, kNumPositions);

  // Compute J̇_WHp using AutoDiffXd.
  CalcPointsOnFrameHGeometricJacobian(
      tree_autodiff(), *context_autodiff_, p_HPi_autodiff,
      &p_WPi_autodiff, &Jv_WHp_autodiff);

  // Extract time derivatives:
  MatrixX<double> Jv_WHp_derivs =
      math::autoDiffToGradientMatrix(Jv_WHp_autodiff);
  Jv_WHp_derivs.resize(3 * kNumPoints, kNumPositions);

  // Compute the expected value of the bias terms using the time derivatives
  // computed with AutoDiffXd.
  const VectorX<double> Ab_WHp_expected = Jv_WHp_derivs * v;

  // Compute points geometric Jacobian bias.
  const VectorX<double> Ab_WHp =
      tree().CalcBiasForPointsGeometricJacobianExpressedInWorld(
      *context_, *frame_H_, p_HPi);

  // Ab_WHp is of size 3⋅kNumPoints x num_velocities. CompareMatrices() below
  // verifies this, in addition to the numerical values of each element.
  EXPECT_TRUE(CompareMatrices(Ab_WHp, Ab_WHp_expected,
                              kTolerance, MatrixCompareType::relative));

  // Verify CalcBiasForPointsGeometricJacobianExpressedInWorld() throws an
  // exception if the input set of points is not represented as a matrix with
  // three rows.
  MatrixX<double> p_HQi(5, kNumPoints);  // an invalid size input set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcBiasForPointsGeometricJacobianExpressedInWorld(
          *context_, *frame_H_, p_HQi),
      std::exception,
      ".* condition '.*.rows\\(\\) == 3' failed.");
}

// Given a set of points Pi attached to the end effector frame G, this test
// computes the analytic Jacobian Jq_WPi of these points using two methods:
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
  VectorX<double> q0, v0;  // v0 will not be used in this test.
  GetArbitraryNonZeroConfiguration(&q0, &v0);

  context_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q0);

  // A set of points Pi attached to the end effector, thus we a fixed position
  // in its frame G.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_EPi(3, kNumPoints);
  p_EPi.col(0) << 0.1, -0.05, 0.02;
  p_EPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_WPi(3, kNumPoints);
  MatrixX<double> Jq_WPi(3 * kNumPoints, kNumPositions);

  // Since for the Kuka iiwa arm v = q̇, the analytic Jacobian Jq_WPi equals the
  // geometric Jacobian Jv_Wpi.
  CalcPointsOnEndEffectorGeometricJacobian(
      tree(), *context_, p_EPi, &p_WPi, &Jq_WPi);

  // Alternatively, compute the analytic Jacobian by taking the gradient of
  // the positions p_WPi(q) with respect to the generalized positions. We do
  // that with the steps below.

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(kNumPositions);
  math::initializeAutoDiff(q0, q_autodiff);
  context_autodiff_->get_mutable_continuous_state().
      get_mutable_generalized_position().SetFromVector(q_autodiff);

  const MatrixX<AutoDiffXd> p_EPi_autodiff = p_EPi;
  MatrixX<AutoDiffXd> p_WPi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> Jq_WPi_autodiff(3 * kNumPoints, kNumPositions);

  CalcPointsOnEndEffectorGeometricJacobian(
      tree_autodiff(), *context_autodiff_,
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
  EXPECT_EQ(p_WPi_derivs.cols(), kNumPositions);

  // Verify the computed Jacobian Jq_WPi matches the one obtained using
  // automatic differentiation.
  // In this case analytic and geometric Jacobians are equal since v = q.
  EXPECT_TRUE(CompareMatrices(Jq_WPi, p_WPi_derivs,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, EvalPoseAndSpatialVelocity) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joints' angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Set joint angles and rates.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Spatial velocity of the end effector in the world frame.
  const SpatialVelocity<double>& V_WE =
      tree().EvalBodySpatialVelocityInWorld(*context_, *end_effector_link_);

  // Pose of the end effector in the world frame.
  const math::RigidTransform<double> X_WE(
      tree().EvalBodyPoseInWorld(*context_, *end_effector_link_));

  // Independent benchmark solution.
  const SpatialKinematicsPVA<double> MG_kinematics =
      benchmark_.CalcEndEffectorKinematics(
          q, v, VectorX<double>::Zero(7) /* vdot */);
  const SpatialVelocity<double>& V_WE_benchmark =
      MG_kinematics.spatial_velocity();
  const math::RigidTransform<double> X_WE_benchmark(MG_kinematics.transform());

  // Compare against benchmark.
  EXPECT_TRUE(V_WE.IsApprox(V_WE_benchmark, kTolerance));
  EXPECT_TRUE(CompareMatrices(X_WE.GetAsMatrix34(),
                              X_WE_benchmark.GetAsMatrix34(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcFrameGeometricJacobianExpressedInWorld) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->get_continuous_state().size(), kNumStates);
  ASSERT_EQ(context_autodiff_->get_continuous_state().size(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joints' angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Set joint angles and rates.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Spatial velocity of the end effector.
  const SpatialVelocity<double>& V_WE =
      tree().EvalBodySpatialVelocityInWorld(*context_, *end_effector_link_);

  // Pose of the end effector.
  const math::RigidTransformd X_WE(
      tree().EvalBodyPoseInWorld(*context_, *end_effector_link_));

  // Position of a frame F measured and expressed in frame E.
  const Vector3d p_EoFo_E = Vector3d(0.2, -0.1, 0.5);
  // Express this same last vector in the world frame.
  const Vector3d p_EoFo_W = X_WE.rotation() * p_EoFo_E;

  // The spatial velocity of a frame F moving with E can be obtained by
  // "shifting" the spatial velocity of frame E from Eo to Fo.
  // That is, frame F has the spatial velocity of a frame Ef obtained by
  // "shifting" from E by an offset p_EoFo.
  const SpatialVelocity<double> V_WEf = V_WE.Shift(p_EoFo_W);

  MatrixX<double> Jv_WF(6, tree().num_velocities());
  // Compute the Jacobian Jv_WF for that relate the generalized velocities with
  // the spatial velocity of frame F.
  tree().CalcFrameGeometricJacobianExpressedInWorld(
      *context_,
      end_effector_link_->body_frame(), p_EoFo_E, &Jv_WF);

  // Verify that V_WEf = Jv_WF * v:
  const SpatialVelocity<double> Jv_WF_times_v(Jv_WF * v);

  EXPECT_TRUE(Jv_WF_times_v.IsApprox(V_WEf, kTolerance));
}

// Unit tests MBT::CalcBiasForFrameGeometricJacobianExpressedInWorld() using
// AutoDiffXd to compute time derivatives of the geometric Jacobian to obtain a
// reference solution.
TEST_F(KukaIiwaModelTests, CalcBiasForFrameGeometricJacobianExpressedInWorld) {
  // The number of generalized velocities in the Kuka iiwa robot arm model.
  const int kNumVelocities = tree().num_velocities();

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joint's angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Since the bias term is a function of q and v only (i.e. it is not a
  // function of vdot), we choose a set of arbitrary values for the generalized
  // accelerations. We do purposely set this values to be non-zero to stress
  // the fact that the bias term is not a function of vdot.
  const VectorX<double> vdot =
      VectorX<double>::Constant(
          kNumVelocities, std::numeric_limits<double>::quiet_NaN());

  // Set generalized positions and velocities.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize q_autodiff and v_autodiff so that we differentiate with respect
  // to time.
  // Note: here we pass MatrixXd(v) so that the return gradient uses AutoDiffXd
  // (for which we do have explicit instantiations) instead of
  // AutoDiffScalar<Matrix1d>.
  auto q_autodiff = math::initializeAutoDiffGivenGradientMatrix(q, MatrixXd(v));
  auto v_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(v, MatrixXd(vdot));

  VectorX<AutoDiffXd> x_autodiff(2 * kNumVelocities);
  x_autodiff << q_autodiff, v_autodiff;

  // Set the context for AutoDiffXd computations.
  tree_autodiff().GetMutablePositionsAndVelocities(context_autodiff_.get())
      = x_autodiff;

  // Po specifies the position of a new frame Hp which is the result of shifting
  // frame H from Ho to Po.
  Vector3<double> p_HPo(0.1, -0.05, 0.02);

  // Frame geometric Jacobian for frame H shifted to frame Hp.
  MatrixX<double> Jv_WHp(6, kNumVelocities);

  const Vector3<AutoDiffXd> p_HPo_autodiff = p_HPo;
  MatrixX<AutoDiffXd> Jv_WHp_autodiff(6, kNumVelocities);

  // Compute J̇_WHp using AutoDiffXd.
  CalcFrameHpGeometricJacobian(
      tree_autodiff(), *context_autodiff_, p_HPo_autodiff, &Jv_WHp_autodiff);

  // Extract time derivatives:
  MatrixX<double> Jv_WHp_derivs =
      math::autoDiffToGradientMatrix(Jv_WHp_autodiff);
  Jv_WHp_derivs.resize(6, kNumVelocities);

  // Compute the expected value of the bias terms using the time derivatives
  // computed with AutoDiffXd.
  const VectorX<double> Ab_WHp_expected = Jv_WHp_derivs * v;

  // Compute frame Hp geometric Jacobian bias.
  const VectorX<double> Ab_WHp =
      tree().CalcBiasForFrameGeometricJacobianExpressedInWorld(
          *context_, *frame_H_, p_HPo);

  // Ab_WHp is of size 6 x num_velocities. CompareMatrices() below
  // verifies this, in addition to the numerical values of each element.
  EXPECT_TRUE(CompareMatrices(Ab_WHp, Ab_WHp_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Verify that even when the input set of points and/or the Jacobian might
// contain garbage on input, a query for the world body Jacobian will always
// yield the following results:
//  a) p_WP_set = p_BP_set, since in this case B = W and,
//  b) J_WP is exactly zero, since the world does not move.
TEST_F(KukaIiwaModelTests, PointsGeometricJacobianForTheWorldFrame) {
  // We choose an arbitrary set of non-zero points in the world body. The actual
  // value does not matter for this test. What matters is that we **always** get
  // a zero Jacobian for the world body.
  const Matrix3X<double> p_WP_set = Matrix3X<double>::Identity(3, 10);

  const int nv = tree().num_velocities();
  const int npoints = p_WP_set.cols();

  // We set the output arrays to garbage so that upon returning from
  // CalcPointsGeometricJacobianExpressedInWorld() we can verify they were
  // properly set.
  Matrix3X<double> p_WP_out = Matrix3X<double>::Constant(3, npoints, M_PI);
  MatrixX<double> Jv_WP = MatrixX<double>::Constant(3 * npoints, nv, M_E);

  // The state stored in the context should not affect the result of this test.
  // Therefore we do not set it.

  tree().CalcPointsGeometricJacobianExpressedInWorld(
      *context_, tree().world_body().body_frame(), p_WP_set,
      &p_WP_out, &Jv_WP);

  // Since in this case we are querying for the world frame:
  //   a) the output set should match the input set exactly and,
  //   b) the Jacobian should be exactly zero.
  EXPECT_EQ(p_WP_out, p_WP_set);
  EXPECT_EQ(Jv_WP, MatrixX<double>::Zero(3 * npoints, nv));
}

// Verify that even when the input set of points and/or the Jacobian might
// contain garbage on input, a query for the world body Jacobian will always
// return a zero Jacobian since the world does not move.
TEST_F(KukaIiwaModelTests, FrameGeometricJacobianForTheWorldFrame) {
  // We choose an arbitrary non-zero point in the world body. The actual value
  // does not matter for this test. What matters is that we **always** get a
  // zero Jacobian for the world body.
  const Vector3<double> p_WP(1.0, 1.0, 1.0);

  const int nv = tree().num_velocities();

  // We set the output Jacobian to garbage so that upon returning from
  // CalcFrameGeometricJacobianExpressedInWorld() we can verify it was
  // properly set to zero.
  MatrixX<double> Jv_WP = MatrixX<double>::Constant(6, nv, M_E);

  // The state stored in the context should not affect the result of this test.
  // Therefore we do not set it.

  tree().CalcFrameGeometricJacobianExpressedInWorld(
      *context_, tree().world_body().body_frame(), p_WP, &Jv_WP);

  // Since in this case we are querying for the world frame, the Jacobian should
  // be exactly zero.
  EXPECT_EQ(Jv_WP, MatrixX<double>::Zero(6, nv));
}

TEST_F(KukaIiwaModelTests, CalcRelativeFrameGeometricJacobian) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->get_continuous_state().size(), kNumStates);
  ASSERT_EQ(context_autodiff_->get_continuous_state().size(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // A set of values for the joints' angles chosen mainly to avoid in-plane
  // motions.
  VectorX<double> q, v;
  GetArbitraryNonZeroConfiguration(&q, &v);

  // Set joint angles and rates.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Three arbitrary frames on the robot.
  const Body<double>& link3 = tree().GetBodyByName("iiwa_link_3");
  const Body<double>& link5 = tree().GetBodyByName("iiwa_link_5");
  const Body<double>& link7 = tree().GetBodyByName("iiwa_link_7");

  // An arbitrary point Q in the end effector link 7.
  const Vector3d p_L7Q = Vector3d(0.2, -0.1, 0.5);

  // Link 3 kinematics.
  const math::RigidTransform<double>& X_WL3 =
      tree().EvalBodyPoseInWorld(*context_, link3);
  const Matrix3<double>& R_WL3 = X_WL3.linear();

  // link 5 kinematics.
  const math::RigidTransform<double>& X_WL5 =
      tree().EvalBodyPoseInWorld(*context_, link5);
  const Matrix3<double>& R_WL5 = X_WL5.linear();

  // link 7 kinematics.
  const math::RigidTransform<double>& X_WL7 =
      tree().EvalBodyPoseInWorld(*context_, link7);
  const Matrix3<double>& R_WL7 = X_WL7.linear();

  // Position of Q in L3, expressed in world.
  // Spatial velocity of frame L3 shifted to Q.
  const math::RigidTransform<double> X_L3L7 = tree().CalcRelativeTransform(
      *context_, link3.body_frame(), link7.body_frame());
  const Vector3<double> p_L3Q_W = R_WL3 * (X_L3L7 * p_L7Q);

  // Position of Q in L7, expressed in world.
  const Vector3<double> p_L7Q_W = R_WL7 * p_L7Q;

  // Spatial velocity of L3 shifted to Q.
  const SpatialVelocity<double> V_WL3q =
      tree().EvalBodySpatialVelocityInWorld(*context_, link3).Shift(p_L3Q_W);

  // Spatial velocity of L7 shifted to Q.
  const SpatialVelocity<double> V_WL7q =
      tree().EvalBodySpatialVelocityInWorld(*context_, link7).Shift(p_L7Q_W);

  // Relative spatial velocity V_L3L7q_L5 of L7q in L3, expressed in L5.
  const SpatialVelocity<double> V_L3L7q_L5 =
      R_WL5.transpose() * (V_WL7q - V_WL3q);

  MatrixX<double> Jv_L3L7q_L5(6, tree().num_velocities());
  // Compute the Jacobian Jv_WF for that relate the generalized velocities with
  // the spatial velocity of frame F.
  tree().CalcRelativeFrameGeometricJacobian(
      *context_, link7.body_frame(), p_L7Q,
      link3.body_frame(), link5.body_frame(), &Jv_L3L7q_L5);

  // Verify that V_L3L7q = Jv_L3L7q * v:
  const SpatialVelocity<double> Jv_L3L7q_L5_times_v(Jv_L3L7q_L5 * v);

  EXPECT_TRUE(Jv_L3L7q_L5_times_v.IsApprox(V_L3L7q_L5, kTolerance));

  // Unit test that CalcRelativeFrameGeometricJacobian throws an exception when
  // the input Jacobian is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcRelativeFrameGeometricJacobian(
          *context_, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), nullptr),
      std::exception, ".*'Jw_V_ABp_E != nullptr'.*");

  // Unit test that CalcRelativeFrameGeometricJacobian throws an exception when
  // the input Jacobian has the wrong number of rows.
  MatrixX<double> Jv_wrong_size;
  Jv_wrong_size.resize(9, tree().num_velocities());
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcRelativeFrameGeometricJacobian(
          *context_, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), &Jv_wrong_size),
      std::exception, ".*'Jw_V_ABp_E->rows\\(\\) == 6'.*");

  // Unit test that CalcRelativeFrameGeometricJacobian throws an exception when
  // the input Jacobian has the wrong number of columns.
  Jv_wrong_size.resize(6, 23);
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcRelativeFrameGeometricJacobian(
          *context_, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), &Jv_wrong_size),
      std::exception, ".*'Jw_V_ABp_E->cols\\(\\) == num_columns'.*");
}

// Fixture to setup a simple MBT model with weld mobilizers. The model is in
// the x-y plane and is sketched below. See unit test code comments for details.
//
//       ◯  <-- Weld joint between the world W and body 1 frame B1.
//       **
//        **
//         **  Body 1. Slab of length 1, at 45 degrees from the world's origin.
//          **
//           **
//            ◯  <-- Weld joint between frames F (on body 1) and M (on body 2).
//           **
//          **
//         **  Body 2. Slab of length 1, at 90 degrees with body 1.
//        **
//       **
//
// There are no other mobilizers and therefore there are no dofs.
class WeldMobilizerTest : public ::testing::Test {
 public:
  // Setup the MBT model as sketched above.
  void SetUp() override {
    // Spatial inertia for each body. The actual value is not important for
    // these tests since they are all kinematic.
    const SpatialInertia<double> M_B;

    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    body1_ = &model->AddBody<RigidBody>(M_B);
    body2_ = &model->AddBody<RigidBody>(M_B);

    model->AddMobilizer<WeldMobilizer>(model->world_body().body_frame(),
                                       body1_->body_frame(), X_WB1_);

    // Add a weld joint between bodies 1 and 2 by welding together inboard
    // frame F (on body 1) with outboard frame M (on body 2).
    const auto& frame_F =
        model->AddFrame<FixedOffsetFrame>(*body1_, X_B1F_);
    const auto& frame_M =
        model->AddFrame<FixedOffsetFrame>(*body2_, X_B2M_);
    model->AddMobilizer<WeldMobilizer>(frame_F, frame_M, X_FM_);

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    // Create a context to store the state for this tree:
    context_ = system_->CreateDefaultContext();

    // Expected pose of body 2 in the world.
    X_WB2_.set_translation(
        Vector3d(M_SQRT2 / 4, -M_SQRT2 / 4, 0.0) - Vector3d::UnitY() / M_SQRT2);
    X_WB2_.set_rotation(math::RotationMatrixd::MakeZRotation(-3 * M_PI_4));
  }

  const MultibodyTree<double>& tree() const {
      return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* body1_{nullptr};
  const RigidBody<double>* body2_{nullptr};

  const math::RigidTransformd X_WB1{AngleAxisd(-M_PI_4, Vector3d::UnitZ()) *
                                    Eigen::Translation3d(0.5, 0.0, 0.0)};
  math::RigidTransformd X_WB1_{X_WB1};
  math::RigidTransformd X_FM_{math::RotationMatrixd::MakeZRotation(-M_PI_2)};
  math::RigidTransformd X_B1F_{Vector3d(0.5, 0.0, 0.0)};
  math::RigidTransformd X_B2M_{Vector3d(-0.5, 0.0, 0.0)};
  math::RigidTransformd X_WB2_;
};

TEST_F(WeldMobilizerTest, StateHasZeroSize) {
  EXPECT_EQ(tree().num_positions(), 0);
  EXPECT_EQ(tree().num_velocities(), 0);
  EXPECT_EQ(context_->get_continuous_state().size(), 0);
}

TEST_F(WeldMobilizerTest, PositionKinematics) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  std::vector<math::RigidTransformd> body_poses;
  tree().CalcAllBodyPosesInWorld(*context_, &body_poses);

  EXPECT_TRUE(body_poses[body1_->index()].IsNearlyEqualTo(X_WB1_, kTolerance));
  EXPECT_TRUE(body_poses[body2_->index()].IsNearlyEqualTo(X_WB2_, kTolerance));
}

}  // namespace
}  // namespace multibody_model
}  // namespace internal
}  // namespace multibody
}  // namespace drake
