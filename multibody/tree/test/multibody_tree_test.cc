#include "drake/multibody/tree/multibody_tree.h"

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"
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
using math::RotationMatrix;
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
      "iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4",
      "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"};

  const std::vector<std::string> kFrameNames = {
      "iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4",
      "iiwa_link_5", "iiwa_link_6", "iiwa_link_7", "tool_arbitrary"};

  const std::vector<std::string> kJointNames = {
      "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
      "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

  const std::vector<std::string> kActuatorNames = {
      "iiwa_actuator_1", "iiwa_actuator_2", "iiwa_actuator_3",
      "iiwa_actuator_4", "iiwa_actuator_5", "iiwa_actuator_6",
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
  for (const std::string& link_name : kLinkNames) {
    EXPECT_TRUE(model.HasBodyNamed(link_name));
    EXPECT_EQ(model.NumBodiesWithName(link_name), 1);
  }
  EXPECT_FALSE(model.HasBodyNamed(kInvalidName));
  EXPECT_EQ(model.NumBodiesWithName(kInvalidName), 0);

  for (const std::string& frame_name : kFrameNames) {
    EXPECT_TRUE(model.HasFrameNamed(frame_name));
  }
  EXPECT_FALSE(model.HasFrameNamed(kInvalidName));

  for (const std::string& joint_name : kJointNames) {
    EXPECT_TRUE(model.HasJointNamed(joint_name));
  }
  EXPECT_FALSE(model.HasJointNamed(kInvalidName));

  for (const std::string& actuator_name : kActuatorNames) {
    EXPECT_TRUE(model.HasJointActuatorNamed(actuator_name));
  }
  EXPECT_FALSE(model.HasJointActuatorNamed(kInvalidName));

  // Get model instance by name.
  DRAKE_EXPECT_THROWS_MESSAGE(model.GetModelInstanceByName(kInvalidName),
                              ".*There is no model instance named.*The current "
                              "model instances are.*DefaultModelInstance.*");

  // Get links by name.
  for (const std::string& link_name : kLinkNames) {
    drake::test::LimitMalloc guard;
    const RigidBody<T>& link = model.GetRigidBodyByName(link_name);
    EXPECT_EQ(link.name(), link_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetRigidBodyByName(kInvalidName),
      ".*There is no RigidBody named .*valid names in model instance "
      "'WorldModelInstance' are: world; valid names in model instance "
      "'DefaultModelInstance' are: iiwa_link_1, iiwa_link_2.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetRigidBodyByName(kLinkNames[0], world_model_instance()),
      ".*There is no RigidBody.*but one does exist in other model instances.*");

  // Test that calling GetRigidBodyByName() with an invalid ModelInstanceIndex
  // throws.
  const ModelInstanceIndex kInvalidIndex(1 << 30);
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetRigidBodyByName(kLinkNames[0], kInvalidIndex),
      ".*There is no model instance.*in the model.*");

  // Get frames by name.
  for (const std::string& frame_name : kFrameNames) {
    drake::test::LimitMalloc guard;
    const Frame<T>& frame = model.GetFrameByName(frame_name);
    EXPECT_EQ(frame.name(), frame_name);
    EXPECT_EQ(&frame,
              &model.GetFrameByName(frame_name, default_model_instance()));
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetFrameByName(kInvalidName),
      ".*There is no Frame named .*valid names in model instance "
      "'DefaultModelInstance' are.* iiwa_link_1.*");

  // Get joints by name.
  for (const std::string& joint_name : kJointNames) {
    drake::test::LimitMalloc guard;
    const Joint<T>& joint = model.GetJointByName(joint_name);
    EXPECT_EQ(joint.name(), joint_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetJointByName(kInvalidName),
      ".*There is no Joint named .*valid names in model instance "
      "'DefaultModelInstance' are.* iiwa_joint_1.*");

  // Templatized version to obtain a particular known type of joint.
  for (const std::string& joint_name : kJointNames) {
    drake::test::LimitMalloc guard;
    const RevoluteJoint<T>& joint =
        model.template GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_EQ(joint.name(), joint_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.template GetJointByName<RevoluteJoint>(kInvalidName),
      ".*There is no Joint named .*valid names in model instance "
      "'DefaultModelInstance' are.* iiwa_joint_1.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.template GetJointByName<PrismaticJoint>(kJointNames[0]),
      ".*not of type.*PrismaticJoint.*but.*RevoluteJoint.*");

  // Get actuators by name.
  for (const std::string& actuator_name : kActuatorNames) {
    drake::test::LimitMalloc guard;
    const JointActuator<T>& actuator =
        model.GetJointActuatorByName(actuator_name);
    EXPECT_EQ(actuator.name(), actuator_name);
  }
  DRAKE_EXPECT_THROWS_MESSAGE(model.GetJointActuatorByName(kInvalidName),
                              ".*There is no JointActuator named .*valid names "
                              "in model instance 'DefaultModelInstance' are.* "
                              "iiwa_actuator_1.*");

  // Test we can retrieve joints from the actuators.
  int names_index = 0;
  for (const std::string& actuator_name : kActuatorNames) {
    drake::test::LimitMalloc guard;
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

// This test creates a model for a KUKA Iiwa arm and verifies we can retrieve
// multibody elements by name or get exceptions accordingly.
GTEST_TEST(MultibodyTree, VerifyModelBasics) {
  // Create a non-finalized model of the arm so that we can test adding more
  // elements to it.
  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(false /* non-finalized model. */);

  // Verify the model was not finalized.
  EXPECT_FALSE(model->is_finalized());

  // Attempt to add a body having the same name as a body already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddRigidBody("iiwa_link_5", default_model_instance(),
                          SpatialInertia<double>::NaN()),
      ".* already contains a body named 'iiwa_link_5'. "
      "Body names must be unique within a given model.");

  // Attempt to add a frame having the same name as a frame already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddFrame<FixedOffsetFrame>(
          "iiwa_link_5", model->GetRigidBodyByName("iiwa_link_1"),
          RigidTransform<double>()),
      ".* already contains a frame named 'iiwa_link_5'. "
      "Frame names must be unique within a given model.");

  // Attempt to add a joint having the same name as a joint already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddJoint<RevoluteJoint>("iiwa_joint_4", model->world_body(),
                                     std::nullopt,
                                     model->GetRigidBodyByName("iiwa_link_5"),
                                     std::nullopt, Vector3<double>::UnitZ()),
      ".* already contains a joint named 'iiwa_joint_4'. "
      "Joint names must be unique within a given model.");

  // Attempt to add an actuator having the same name as an actuator already part
  // of the model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->AddJointActuator("iiwa_actuator_4",
                              model->GetJointByName("iiwa_joint_4")),
      ".* already contains a joint actuator named 'iiwa_actuator_4'. "
      "Joint actuator names must be unique within a given model.");

  // Now we tested we cannot add body or joints with an existing name, finalize
  // the model.
  DRAKE_EXPECT_NO_THROW(model->Finalize());

  // Another call to Finalize() is not allowed.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  VerifyModelBasics(*model);
}

// Confirms that the error messages produced by GetElementByName are reasonable
// even for an empty model.
GTEST_TEST(MultibodyTree, EmptyGetElementByName) {
  // Create an empty model.
  MultibodyTree<double> model;
  const std::string kInvalidName = "InvalidName";

  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetRigidBodyByName(kInvalidName),
      ".*There is no RigidBody named .*valid names in model instance "
      "'WorldModelInstance' are.* world.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetFrameByName(kInvalidName),
      ".*There is no Frame named .*valid names in model instance "
      "'WorldModelInstance' are.* world.*");
  DRAKE_EXPECT_THROWS_MESSAGE(model.GetJointByName(kInvalidName),
                              ".*There are no Joints defined in the model");
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.template GetJointByName<RevoluteJoint>(kInvalidName),
      ".*There are no Joints defined in the model");
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.GetJointActuatorByName(kInvalidName),
      ".*There are no JointActuators defined in the model");
}

// Exercises the error detection and reporting for retrieving model elements by
// name when the same name exists in multiple model instances.  Since the code
// for Body, Frame, Joint, etc. all use the same templated implementation, it's
// sufficient to just test one element type.
GTEST_TEST(MultibodyTree, RetrievingAmbiguousNames) {
  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(false /* non-finalized model. */);

  // Add a duplicate body, but on a different model instance.
  const ModelInstanceIndex other_model_instance =
      model->AddModelInstance("other");
  const std::string link_name = "iiwa_link_5";
  EXPECT_NO_THROW(model->AddRigidBody(link_name, other_model_instance,
                                      SpatialInertia<double>::NaN()));
  EXPECT_NO_THROW(model->Finalize());

  // Link name is ambiguous, there are more than one bodies that use the name.
  EXPECT_GT(model->NumBodiesWithName(link_name), 1);

  // Checking if the name exists throws (unfortunately), unless we specify the
  // intended model instance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->HasBodyNamed(link_name),
      ".*Body.*appears in multiple model instances.*disambiguate.*");
  EXPECT_TRUE(model->HasBodyNamed(link_name, default_model_instance()));

  // Accessing by name throws, unless we specify the intended model instance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      model->GetRigidBodyByName(link_name),
      ".*RigidBody.*appears in multiple model instances.*disambiguate.*");
  EXPECT_NO_THROW(
      model->GetRigidBodyByName(link_name, default_model_instance()));
}

// MBPlant provides most of the testing for MBTreeSystem. Here we just want
// to make sure that a badly-behaved derived class is notified.
class BadDerivedMBSystem : public MultibodyTreeSystem<double> {
 public:
  explicit BadDerivedMBSystem(bool double_finalize)
      : MultibodyTreeSystem<double>() {
    mutable_tree().AddRigidBody("body", SpatialInertia<double>::NaN());
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
  DRAKE_EXPECT_NO_THROW(finalized.mutable_tree());

  // Make the MBSystem behave badly.
  DRAKE_EXPECT_THROWS_MESSAGE(BadDerivedMBSystem(true),
                              ".*Finalize().*repeated.*not allowed.*");

  auto model = std::make_unique<MultibodyTree<double>>();
  DRAKE_EXPECT_NO_THROW(MultibodyTreeSystem<double>(std::move(model)));
  EXPECT_EQ(model, nullptr);  // Should have been moved from.

  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyTreeSystem<double>(std::move(model)),
      ".*MultibodyTreeSystem().*MultibodyTree was null.*");
}

GTEST_TEST(MultibodyTree, BackwardsCompatibility) {
  auto owned_tree = std::make_unique<MultibodyTree<double>>();
  auto* tree = owned_tree.get();
  DRAKE_EXPECT_THROWS_MESSAGE(tree->CreateDefaultContext(),
                              ".*that is owned by a MultibodyPlant.*");
  MultibodyTreeSystem<double> system(std::move(owned_tree));
  DRAKE_EXPECT_NO_THROW(tree->CreateDefaultContext());
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
      end_effector_link_ = &tree->GetRigidBodyByName("iiwa_link_7");
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_7"));

      // Add a frame H with a fixed pose X_GH in the end effector frame G.
      // Note: frame names are documented in MakeKukaIiwaModel().
      frame_H_ =
          &tree->AddFrame<FixedOffsetFrame>("H", *end_effector_link_, X_GH_);

      // Create a system to manage context resources.
      system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(tree));
    }

    context_ = system_->CreateDefaultContext();

    DRAKE_EXPECT_NO_THROW(context_->Clone());

    // Scalar-convert the model and create a default context for it.
    system_autodiff_ = std::make_unique<MultibodyTreeSystem<AutoDiffXd>>(
        tree().ToAutoDiffXd());
    context_autodiff_ = system_autodiff_->CreateDefaultContext();
  }

  // Get an arm state associated with an arbitrary configuration that avoids
  // in-plane motion and in which joint angles and rates are non-zero.
  void GetArbitraryNonZeroJointAnglesAndRates(VectorX<double>* q,
                                              VectorX<double>* v) {
    const int kNumPositions = tree().num_positions();
    q->resize(kNumPositions);
    v->resize(kNumPositions);  // q and v have the same dimension for kuka.

    // These joint angles avoid in-plane motion, but are otherwise arbitrary.
    const double q30 = M_PI / 6, q60 = M_PI / 3;
    const double qA = q60;
    const double qB = q30;
    const double qC = q60;
    const double qD = q30;
    const double qE = q60;
    const double qF = q30;
    const double qG = q60;
    *q << qA, qB, qC, qD, qE, qF, qG;

    // Arbitrary non-zero angular rates in radians/second.
    const double vA = 0.12345;
    const double vB = -0.1987;
    const double vC = 0.54322;
    const double vD = -0.6732;
    const double vE = 0.31415;
    const double vF = -0.7733;
    const double vG = 0.71828;
    *v << vA, vB, vC, vD, vE, vF, vG;
  }

  // Computes the translational velocity `v_WE` of the end effector frame E in
  // the world frame W.
  template <typename T>
  Vector3<T> CalcEndEffectorVelocity(const MultibodyTree<T>& model_on_T,
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
  Vector3<T> CalcEndEffectorPosition(const MultibodyTree<T>& model_on_T,
                                     const Context<T>& context_on_T) const {
    const RigidBody<T>& linkG_on_T =
        model_on_T.get_variant(*end_effector_link_);
    Vector3<T> p_WE;
    model_on_T.CalcPointsPositions(context_on_T, linkG_on_T.body_frame(),
                                   Vector3<T>::Zero(),  // position in frame G
                                   model_on_T.world_body().body_frame(), &p_WE);
    return p_WE;
  }

  // For each point Ei fixed (welded) to an end effector frame E, calculates
  // Jv_WEi_W, Ei's translational velocity Jacobian in the world frame W.
  // See MultibodyTree::CalcJacobianTranslationalVelocity() for details.
  template <typename T>
  void CalcPointsOnEndEffectorTranslationalVelocityJacobianWrtV(
      const MultibodyTree<T>& model_on_T, const Context<T>& context_on_T,
      const MatrixX<T>& p_EoEi_E, MatrixX<T>* p_WoEi_W,
      MatrixX<T>* Jv_WEi_W) const {
    const RigidBody<T>& linkG_on_T =
        model_on_T.get_variant(*end_effector_link_);
    const Frame<T>& frame_E = linkG_on_T.body_frame();
    const Frame<T>& frame_W = model_on_T.world_frame();
    model_on_T.CalcJacobianTranslationalVelocity(
        context_on_T, JacobianWrtVariable::kV, frame_E, frame_E, p_EoEi_E,
        frame_W, frame_W, Jv_WEi_W);

    // For each point Ei, calculate Ei's position from Wo (World origin),
    // expressed in world W.
    model_on_T.CalcPointsPositions(context_on_T, frame_E, p_EoEi_E,  // From E
                                   frame_W, p_WoEi_W);               // to W.
  }

  // For each point Hi fixed (welded) to frame H, calculates Jv_WHi_W,
  // Hi's translational velocity Jacobian in W, expressed in W.
  // See MultibodyTree::CalcJacobianTranslationalVelocity() for details.
  template <typename T>
  void CalcPointsOnFrameHTranslationalVelocityJacobianWrtV(
      const MultibodyTree<T>& model_on_T, const Context<T>& context_on_T,
      const MatrixX<T>& p_HoHi_H, MatrixX<T>* p_WoHi_W,
      MatrixX<T>* Jv_WHi_W) const {
    const Frame<T>& frameH_on_T = model_on_T.get_variant(*frame_H_);
    const Frame<T>& frame_W = model_on_T.world_frame();
    model_on_T.CalcJacobianTranslationalVelocity(
        context_on_T, JacobianWrtVariable::kV, frameH_on_T, frameH_on_T,
        p_HoHi_H, frame_W, frame_W, Jv_WHi_W);

    // Calculate p_WoHi_W (Hi's position from World origin Wo, expressed in W)
    // from p_HoHi_H (Hi's position from Ho, expressed in H).
    model_on_T.CalcPointsPositions(context_on_T, frameH_on_T, p_HoHi_H, frame_W,
                                   p_WoHi_W);
  }

  // For a point Hp fixed/welded to frame H (attached to the end effector, see
  // test fixture docs), calculate Jv_V_WHp, Hp's spatial velocity Jacobian with
  // with respect to generalized velocities v.  If this class is templated on
  // AutoDiffXd, this method can also calculate its time derivative J̇v_V_WHp.
  template <typename T>
  void CalcFrameHpJacobianSpatialVelocityInWorld(
      const MultibodyTree<T>& model_on_T, const Context<T>& context_on_T,
      const Vector3<T>& p_HoHp_H, MatrixX<T>* Jv_V_WHp) const {
    const Frame<T>& frameH_on_T = model_on_T.get_variant(*frame_H_);
    const Frame<T>& frame_W = model_on_T.world_frame();
    model_on_T.CalcJacobianSpatialVelocity(
        context_on_T, JacobianWrtVariable::kV, frameH_on_T, p_HoHp_H, frame_W,
        frame_W, Jv_V_WHp);
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
  const RigidBody<double>* end_effector_link_{nullptr};
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

TEST_F(KukaIiwaModelTests, StateAccess) {
  ASSERT_EQ(tree().num_positions(), 7);
  ASSERT_EQ(tree().num_velocities(), 7);
  ASSERT_EQ(tree().num_states(), 14);

  const Eigen::VectorXd qv_values =
      (Eigen::VectorXd(14) << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)
          .finished();

  // Check whole-state access.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  EXPECT_EQ(tree().get_positions_and_velocities(*context_), qv_values);

  // Verify that position and velocity access works when whole state was set.
  tree().get_mutable_positions_and_velocities(&context_->get_mutable_state()) =
      2 * qv_values;
  EXPECT_EQ(tree().get_positions_and_velocities(*context_), 2 * qv_values);
  EXPECT_EQ(tree().get_positions(*context_), 2 * qv_values.segment(0, 7));
  EXPECT_EQ(tree().get_velocities(*context_), 2 * qv_values.segment(7, 7));

  // Check that position-only access works and that velocity doesn't change.
  tree().GetMutablePositions(context_.get()) = qv_values.segment(0, 7);
  EXPECT_EQ(tree().get_positions(*context_), qv_values.segment(0, 7));
  EXPECT_EQ(tree().get_velocities(*context_), 2 * qv_values.segment(7, 7));
  tree().get_mutable_positions(&context_->get_mutable_state()) =
      3 * qv_values.segment(0, 7);
  EXPECT_EQ(tree().get_positions(*context_), 3 * qv_values.segment(0, 7));
  EXPECT_EQ(tree().get_velocities(*context_), 2 * qv_values.segment(7, 7));

  // Check that velocity-only access works and that position doesn't change.
  tree().GetMutableVelocities(context_.get()) = qv_values.segment(7, 7);
  EXPECT_EQ(tree().get_velocities(*context_), qv_values.segment(7, 7));
  EXPECT_EQ(tree().get_positions(*context_), 3 * qv_values.segment(0, 7));
  tree().get_mutable_velocities(&context_->get_mutable_state()) =
      5 * qv_values.segment(7, 7);
  EXPECT_EQ(tree().get_velocities(*context_), 5 * qv_values.segment(7, 7));
  EXPECT_EQ(tree().get_positions(*context_), 3 * qv_values.segment(0, 7));

  // Check that setting position and velocity separately sets the whole state.
  tree().GetMutablePositions(context_.get()) = 7 * qv_values.segment(0, 7);
  tree().GetMutableVelocities(context_.get()) = 7 * qv_values.segment(7, 7);
  EXPECT_EQ(tree().get_positions_and_velocities(*context_), 7 * qv_values);

  // Test that the state segment methods work.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  EXPECT_EQ(tree().get_state_segment<3>(*context_, 5), qv_values.segment(5, 3));
  EXPECT_EQ(tree().get_state_segment(*context_, 5, 4), qv_values.segment(5, 4));

  // There are four segment-mutating methods. We'll use each
  // to make the same change, then verify with this lambda.
  const auto check_segments = [&]() {
    EXPECT_EQ(tree().get_positions_and_velocities(*context_).segment(8, 3),
              Vector3d(-1, -2, -3));
    // Nothing else should have changed.
    EXPECT_EQ(tree().get_positions_and_velocities(*context_).segment(0, 8),
              qv_values.segment(0, 8));
    EXPECT_EQ(tree().get_positions_and_velocities(*context_).segment(11, 3),
              qv_values.segment(11, 3));
  };

  // Templatized mutable-Context method.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  tree().GetMutableStateSegment<3>(context_.get(), 8) << -1, -2, -3;
  check_segments();

  // Templatized mutable-State method.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  tree().get_mutable_state_segment<3>(&context_->get_mutable_state(), 8) << -1,
      -2, -3;
  check_segments();

  // Runtime mutable-Context method.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  tree().GetMutableStateSegment(context_.get(), 8, 3) << -1, -2, -3;
  check_segments();

  // Runtime mutable-State method.
  tree().GetMutablePositionsAndVelocities(context_.get()) = qv_values;
  tree().get_mutable_state_segment(&context_->get_mutable_state(), 8, 3) << -1,
      -2, -3;
  check_segments();

  EXPECT_TRUE(CompareMatrices(
      tree().GetEffortLowerLimits(),
      Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(CompareMatrices(
      tree().GetEffortUpperLimits(),
      Eigen::VectorXd::Constant(7, std::numeric_limits<double>::infinity())));
}

// This test helps verify MultibodyTree::CalcJacobianTranslationalVelocity()
// with two methods to calculate Jv_WEo_W, which is Eo's (end effector origin's)
// translational velocity Jacobian with respect to v (generalized velocities)
// in the world frame W, expressed in W.
// 1. Calling MultibodyTree::CalcJacobianTranslationalVelocity().
// 2. Using AutoDiffXd to compute the partial derivative of v_WE(q, v) with
//    respect to v.
// In addition, we are testing methods:
// - MultibodyTree::CalcPointsPositions()
// - MultibodyTree::CalcPointsVelocities()
// - MultibodyTree::CalcAllBodySpatialVelocitiesInWorld()
TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityA) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->num_continuous_states(), kNumStates);
  ASSERT_EQ(context_autodiff_->num_continuous_states(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q, v;
  GetArbitraryNonZeroJointAnglesAndRates(&q, &v);

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
  math::InitializeAutoDiff(v, &v_autodiff);
  context_autodiff_->get_mutable_continuous_state()
      .get_mutable_generalized_velocity()
      .SetFromVector(v_autodiff);

  const Vector3<AutoDiffXd> v_WE_autodiff =
      CalcEndEffectorVelocity(tree_autodiff(), *context_autodiff_);

  const Vector3<double> v_WE_value = math::ExtractValue(v_WE_autodiff);
  const MatrixX<double> v_WE_derivs = math::ExtractGradient(v_WE_autodiff);

  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(v_WE_value, v_WE, kTolerance,
                              MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_WE_derivs.rows(), 3);
  EXPECT_EQ(v_WE_derivs.cols(), kNumPositions);

  Vector3<double> p_WE;
  Matrix3X<double> Jv_WE(3, tree().num_velocities());
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const Frame<double>& frame_W = tree().world_frame();
  const Vector3<double> p_EoGo_E = Vector3<double>::Zero();
  tree().CalcJacobianTranslationalVelocity(*context_, JacobianWrtVariable::kV,
                                           frame_E, frame_E, p_EoGo_E, frame_W,
                                           frame_W, &Jv_WE);

  // Calculate p_WoEo_W (Eo's position from World origin Wo expressed in W)
  // from p_EoGo_E (Go's position from Eo expressed in E -- zero vector).
  tree().CalcPointsPositions(*context_, frame_E, p_EoGo_E, frame_W, &p_WE);

  // Verify the computed Jacobian matches the one obtained using automatic
  // differentiation.
  EXPECT_TRUE(CompareMatrices(Jv_WE, v_WE_derivs, kTolerance,
                              MatrixCompareType::relative));

  // Verify that v_WE = Jv_WE * v:
  const Vector3<double> Jv_WE_times_v = Jv_WE * v;
  EXPECT_TRUE(CompareMatrices(Jv_WE_times_v, v_WE, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcPointsVelocities() matches v_WE = Jv_WE * v.
  Vector3<double> v_WEo_W;  // Quantity to be calculated.
  tree().CalcPointsVelocities(*context_, frame_E, p_EoGo_E, frame_W, frame_W,
                              &v_WEo_W);
  EXPECT_TRUE(
      CompareMatrices(v_WEo_W, v_WE, kTolerance, MatrixCompareType::relative));

  // Create a set of points to further test CalcPointsVelocities().
  constexpr int num_position_vectors = 4;
  MatrixX<double> p_EoEi_E(3, num_position_vectors);
  p_EoEi_E.col(0) = Vector3<double>(0, 0, 0);
  p_EoEi_E.col(1) = Vector3<double>(1, 2, 3);
  p_EoEi_E.col(2) = Vector3<double>(-3, 2, -1.23);
  p_EoEi_E.col(3) = Vector3<double>(-0.2, std::sqrt(2), -3.14);

  // Verify CalcPointsVelocities() expressed-in-frame: v_WEi_E = Jv_WEi_E * v.
  MatrixX<double> v_WEi_E(3, num_position_vectors);  // Quantity to calculate.
  tree().CalcPointsVelocities(*context_, frame_E, p_EoEi_E, frame_W, frame_E,
                              &v_WEi_E);
  Matrix3X<double> Jv_WEi_E(3, tree().num_velocities());
  for (int i = 0; i < num_position_vectors; ++i) {
    tree().CalcJacobianTranslationalVelocity(*context_, JacobianWrtVariable::kV,
                                             frame_E, frame_E, p_EoEi_E.col(i),
                                             frame_W, frame_E, &Jv_WEi_E);
    EXPECT_TRUE(CompareMatrices(v_WEi_E.col(i), Jv_WEi_E * v, kTolerance,
                                MatrixCompareType::relative));
  }

  // Verify CalcPointsVelocities() measured-in-frame: v_WEo_E = Jv_WEo_E * v.
  const RigidBody<double>& link3 = tree().GetRigidBodyByName("iiwa_link_3");
  const Frame<double>& frame_M = link3.body_frame();  // Measured-in frame.
  Matrix3X<double> v_MEi_M(3, num_position_vectors);  // Quantity to calculate.
  tree().CalcPointsVelocities(*context_, frame_E, p_EoEi_E, frame_M, frame_M,
                              &v_MEi_M);
  Matrix3X<double> Jv_MEi_M(3, tree().num_velocities());
  for (int i = 0; i < num_position_vectors; ++i) {
    tree().CalcJacobianTranslationalVelocity(*context_, JacobianWrtVariable::kV,
                                             frame_E, frame_E, p_EoEi_E.col(i),
                                             frame_M, frame_M, &Jv_MEi_M);
    EXPECT_TRUE(CompareMatrices(v_MEi_M.col(i), Jv_MEi_M * v, kTolerance,
                                MatrixCompareType::relative));
  }

  // Verify that MultibodyTree::CalcPointsPositions() computes the same value
  // of p_WE. Even both code paths resolve to CalcPointsPositions(), here we
  // call this method explicitly to provide unit testing for this API.
  Vector3<double> p2_WE = CalcEndEffectorPosition(tree(), *context_);
  EXPECT_TRUE(
      CompareMatrices(p2_WE, p_WE, kTolerance, MatrixCompareType::relative));

  // The derivative with respect to time should equal v_WE.
  const VectorX<AutoDiffXd> q_autodiff =
      // For reasons beyond my understanding, we need to pass a MatrixXd as the
      // gradient for math::InitializeAutoDiff(). Eigen complains otherwise.
      math::InitializeAutoDiff(q, MatrixXd(v));
  v_autodiff = v.cast<AutoDiffXd>();
  context_autodiff_->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q_autodiff);
  context_autodiff_->get_mutable_continuous_state()
      .get_mutable_generalized_velocity()
      .SetFromVector(v_autodiff);

  Vector3<AutoDiffXd> p_WE_autodiff =
      CalcEndEffectorPosition(tree_autodiff(), *context_autodiff_);
  Vector3<double> p_WE_derivs(p_WE_autodiff[0].derivatives()[0],
                              p_WE_autodiff[1].derivatives()[0],
                              p_WE_autodiff[2].derivatives()[0]);
  EXPECT_TRUE(CompareMatrices(p_WE_derivs, v_WE, kTolerance,
                              MatrixCompareType::relative));
}

// This test is used to verify the correctness of the method
// MultibodyTree::CalcJacobianTranslationalVelocity().
// It test the end effector's translational Jacobian Jv_v_WE_W with two methods:
// 1. Calling MultibodyTree::CalcJacobianTranslationalVelocity().
// 2. Using AutoDiffXd to compute the partial derivative of v_WE_W(q, v) with
//    respect to v.
// In addition, this also tests the methods:
// - MultibodyTree::CalcPointsPositions()
// - MultibodyTree::CalcAllBodySpatialVelocitiesInWorld()
TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityB) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumVelocities = tree().num_velocities();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);
  ASSERT_EQ(kNumVelocities, 7);
  ASSERT_EQ(kNumPositions + kNumVelocities, kNumStates);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->num_continuous_states(), kNumStates);
  ASSERT_EQ(context_autodiff_->num_continuous_states(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q, v;
  GetArbitraryNonZeroJointAnglesAndRates(&q, &v);

  // Set the robot's joint angles and rates (generalized positions/velocities).
  tree().GetMutablePositionsAndVelocities(context_.get()) << q, v;

  // Compute the end effector's velocity.
  const Vector3<double> v_WE = CalcEndEffectorVelocity(tree(), *context_);

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);

  // Initialize v_autodiff to have values v and so that v is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> v_autodiff = drake::math::InitializeAutoDiff(v);
  context_autodiff_->get_mutable_continuous_state()
      .get_mutable_generalized_velocity()
      .SetFromVector(v_autodiff);

  const Vector3<AutoDiffXd> v_WE_autodiff =
      CalcEndEffectorVelocity(tree_autodiff(), *context_autodiff_);

  const Vector3<double> v_WE_value = math::ExtractValue(v_WE_autodiff);
  const MatrixX<double> v_WE_derivs = math::ExtractGradient(v_WE_autodiff);

  // Values from <AutoDiffXd> should match those computed with <double>.
  EXPECT_TRUE(CompareMatrices(v_WE_value, v_WE, kTolerance,
                              MatrixCompareType::relative));

  // Some sanity checks on the expected sizes of the derivatives.
  EXPECT_EQ(v_WE_derivs.rows(), 3);
  EXPECT_EQ(v_WE_derivs.cols(), kNumPositions);

  Matrix3X<double> Jv_WE(3, kNumVelocities);
  // The end effector (G) Jacobian is computed by asking the Jacobian for a
  // point P with position p_GP = 0 in the G frame.
  const Frame<double>& end_effector_frame = end_effector_link_->body_frame();
  const Frame<double>& world_frame = tree().world_frame();
  tree().CalcJacobianTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, end_effector_frame,
      end_effector_frame, Vector3<double>::Zero(), world_frame, world_frame,
      &Jv_WE);

  // Verify the computed Jacobian matches the one from auto-differentiation.
  EXPECT_TRUE(CompareMatrices(Jv_WE, v_WE_derivs, kTolerance,
                              MatrixCompareType::relative));

  // Verify that v_WE = Jv_WE * v:
  const Vector3<double> Jv_WE_times_v = Jv_WE * v;
  EXPECT_TRUE(CompareMatrices(Jv_WE_times_v, v_WE, kTolerance,
                              MatrixCompareType::relative));

  // Use different frame arguments to again calculate the Jacobian.
  // Again verify the computed Jacobian matches that from auto-differentiation.
  const Vector3<double> p_WE = CalcEndEffectorPosition(tree(), *context_);
  tree().CalcJacobianTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, end_effector_frame, world_frame, p_WE,
      world_frame, world_frame, &Jv_WE);
  EXPECT_TRUE(CompareMatrices(Jv_WE, v_WE_derivs, kTolerance,
                              MatrixCompareType::relative));

  // The derivative with respect to time should equal v_WE.
  const VectorX<AutoDiffXd> q_autodiff =
      math::InitializeAutoDiff(q, MatrixXd(v));
  v_autodiff = v.cast<AutoDiffXd>();
  tree_autodiff().GetMutablePositionsAndVelocities(context_autodiff_.get())
      << q_autodiff,
      v_autodiff;

  Vector3<AutoDiffXd> p_WE_autodiff =
      CalcEndEffectorPosition(tree_autodiff(), *context_autodiff_);
  Vector3<double> p_WE_derivs(p_WE_autodiff[0].derivatives()[0],
                              p_WE_autodiff[1].derivatives()[0],
                              p_WE_autodiff[2].derivatives()[0]);
  EXPECT_TRUE(CompareMatrices(p_WE_derivs, v_WE, kTolerance,
                              MatrixCompareType::relative));
}

// Given a set of points Pi attached to the end effector frame G, this test
// calculates Jq̇_v_WPi (Pi's translational velocity Jacobian with respect to q̇)
// using two methods:
// 1. The Kuka iiwa arm uses `q̇ = v`, hence `Jq_p_WoPi = Jq̇_v_WPi = Jv_v_WPi`,
//    (i.e., the position Jacobian with respect to q is the same as the
//    translational velocity Jacobian with respect to q̇ which is the same as the
//    translational Jacobian with respect to v).  So, we compute
//    `Jq_p_WoPi = Jq̇_v_WPi = Jv_v_WPi` with a MultibodyTree implementation.
// 2. We compute Jq_p_WoPi using AutoDiffXd to differentiate with respect to q.
// We ensure these two results are nearly equal (to near machine epsilon).
TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityC) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = 7;

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q0, v0;  // v0 will not be used in this test.
  GetArbitraryNonZeroJointAnglesAndRates(&q0, &v0);

  context_->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q0);

  // A set of points Pi attached to the end effector, thus we a fixed position
  // in its frame G.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_EPi(3, kNumPoints);
  p_EPi.col(0) << 0.1, -0.05, 0.02;
  p_EPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_WPi(3, kNumPoints);
  MatrixX<double> Jq_WPi(3 * kNumPoints, kNumPositions);

  // For the Kuka iiwa arm q̇ = v, so `Jq_v_Wpi = Jv_v_Wpi` i.e., Pi's
  // translational Jacobian in world W with respect to q̇ (time-derivative of
  // generalized positions) is equal to Pi's translational Jacobian in W with
  // respect to v (generalized velocities).
  CalcPointsOnEndEffectorTranslationalVelocityJacobianWrtV(
      tree(), *context_, p_EPi, &p_WPi, &Jq_WPi);

  // Alternatively, compute the Jacobian with respect to q̇ by forming the
  // gradient of the positions p_WPi(q) with respect to generalized positions q.
  // We do that with the steps below.

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(kNumPositions);
  math::InitializeAutoDiff(q0, &q_autodiff);
  context_autodiff_->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q_autodiff);

  const MatrixX<AutoDiffXd> p_EPi_autodiff = p_EPi;
  MatrixX<AutoDiffXd> p_WPi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> Jq_WPi_autodiff(3 * kNumPoints, kNumPositions);

  CalcPointsOnEndEffectorTranslationalVelocityJacobianWrtV(
      tree_autodiff(), *context_autodiff_, p_EPi_autodiff, &p_WPi_autodiff,
      &Jq_WPi_autodiff);

  // Extract values and derivatives:
  const Matrix3X<double> p_WPi_value = math::ExtractValue(p_WPi_autodiff);
  const MatrixX<double> p_WPi_derivs = math::ExtractGradient(p_WPi_autodiff);

  // Some sanity checks:
  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(p_WPi_value, p_WPi, kTolerance,
                              MatrixCompareType::relative));
  // Sizes of the derivatives.
  EXPECT_EQ(p_WPi_derivs.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WPi_derivs.cols(), kNumPositions);

  // Verify the computed Jacobian Jq_WPi matches the one obtained using
  // automatic differentiation.  In this Kuka iiwa arm example, q̇ = v, so
  // these two Jacobian calculations should be nearly equal.
  EXPECT_TRUE(CompareMatrices(Jq_WPi, p_WPi_derivs, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, EvalPoseAndSpatialVelocity) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q, v;
  GetArbitraryNonZeroJointAnglesAndRates(&q, &v);

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
  const RigidTransform<double>& X_WE(
      tree().EvalBodyPoseInWorld(*context_, *end_effector_link_));

  // Independent benchmark solution.
  const SpatialKinematicsPVA<double> MG_kinematics =
      benchmark_.CalcEndEffectorKinematics(q, v,
                                           VectorX<double>::Zero(7) /* vdot */);
  const SpatialVelocity<double>& V_WE_benchmark =
      MG_kinematics.spatial_velocity();
  const RigidTransform<double> X_WE_benchmark(MG_kinematics.transform());

  // Compare against benchmark.
  EXPECT_TRUE(V_WE.IsApprox(V_WE_benchmark, kTolerance));
  EXPECT_TRUE(CompareMatrices(X_WE.GetAsMatrix34(),
                              X_WE_benchmark.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocityA) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->num_continuous_states(), kNumStates);
  ASSERT_EQ(context_autodiff_->num_continuous_states(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q, v;
  GetArbitraryNonZeroJointAnglesAndRates(&q, &v);

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
  const math::RigidTransformd& X_WE(
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
  const Frame<double>& frame_W = tree().world_frame();
  tree().CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     end_effector_link_->body_frame(), p_EoFo_E,
                                     frame_W, frame_W, &Jv_WF);

  // Verify that V_WEf = Jv_WF * v:
  const SpatialVelocity<double> Jv_WF_times_v(Jv_WF * v);

  EXPECT_TRUE(Jv_WF_times_v.IsApprox(V_WEf, kTolerance));
}

// Verify that even when the input set of points and/or the Jacobian might
// contain garbage on input, a query for the world body Jacobian will always
// yield the following results:
//  a) p_WP_set = p_BP_set, since in this case B = W and,
//  b) J_WP is exactly zero, since the world does not move.
TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityD) {
  // We choose an arbitrary set of non-zero points in the world body. The actual
  // value does not matter for this test. What matters is that we **always** get
  // a zero Jacobian for the world body.
  const Matrix3X<double> p_WP_set = Matrix3X<double>::Identity(3, 10);

  const int nv = tree().num_velocities();
  const int npoints = p_WP_set.cols();

  // We set the output arrays to garbage so that upon returning from
  // CalcJacobianTranslationalVelocity() we can verify they were properly set.
  Matrix3X<double> p_WP_out = Matrix3X<double>::Constant(3, npoints, M_PI);
  MatrixX<double> Jv_WP = MatrixX<double>::Constant(3 * npoints, nv, M_E);

  // For each point P, calculate Jv_v_WP (P's translational velocity Jacobian
  // in world W, expressed in W).  Note: This test case is somewhat degenerate.
  const Frame<double>& frame_W = tree().world_frame();
  tree().CalcJacobianTranslationalVelocity(*context_, JacobianWrtVariable::kV,
                                           frame_W, frame_W, p_WP_set, frame_W,
                                           frame_W, &Jv_WP);

  // For each point P, calculate p_WoP_W (P's position from World origin Wo,
  // expressed in world W).  Note: This test case is somewhat degenerate.
  tree().CalcPointsPositions(*context_, frame_W, p_WP_set, frame_W, &p_WP_out);

  // Since in this case we are querying for the world frame:
  //   a) the output set should match the input set exactly and,
  //   b) the Jacobian should be exactly zero.
  EXPECT_EQ(p_WP_out, p_WP_set);
  EXPECT_EQ(Jv_WP, MatrixX<double>::Zero(3 * npoints, nv));

  // For each point P, calculate v_WP_W (P's velocity measured in World W,
  // expressed in world W.  Note: This test case calculates velocities measured
  // in world for points that are attached to world.
  Matrix3X<double> v_WP_W(3, npoints);  // Calculated quantities.
  tree().CalcPointsVelocities(*context_, frame_W, p_WP_set, frame_W, frame_W,
                              &v_WP_W);
  EXPECT_EQ(v_WP_W, MatrixX<double>::Zero(3, npoints));

  // Since v_WP_W = 0, v_WP_link3 should also be zero.
  const Frame<double>& link3 = tree().GetFrameByName("iiwa_link_3");
  Matrix3X<double> v_WP_link3(3, npoints);  // Calculated quantities.
  tree().CalcPointsVelocities(*context_, frame_W, p_WP_set, frame_W, link3,
                              &v_WP_link3);
  EXPECT_EQ(v_WP_link3, MatrixX<double>::Zero(3, npoints));
}

// Verify that even when the input set of points and/or the Jacobian might
// contain garbage on input, a query for the world body Jacobian will always
// return a zero Jacobian since the world does not move.
TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocityB) {
  // We choose an arbitrary non-zero point in the world body. The actual value
  // does not matter for this test. What matters is that we **always** get a
  // zero Jacobian for the world body.
  const Vector3<double> p_WoWp_W(1.0, 1.0, 1.0);

  const int nv = tree().num_velocities();

  // We set the output Jacobian to garbage so that upon returning from
  // CalcJacobianSpatialVelocity() we can verify it was properly set to zero.
  MatrixX<double> Jv_WWp = MatrixX<double>::Constant(6, nv, M_E);

  // The state stored in the context should not affect the result of this test.
  // Therefore we do not set it.
  const Frame<double>& frame_W = tree().world_frame();
  tree().CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     tree().world_body().body_frame(), p_WoWp_W,
                                     frame_W, frame_W, &Jv_WWp);

  // Since in this case we are querying for the world frame, the Jacobian should
  // be exactly zero.
  EXPECT_EQ(Jv_WWp, MatrixX<double>::Zero(6, nv));
}

TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocityC) {
  // The number of generalized positions in the Kuka iiwa robot arm model.
  const int kNumPositions = tree().num_positions();
  const int kNumStates = tree().num_states();

  ASSERT_EQ(kNumPositions, 7);

  ASSERT_EQ(tree_autodiff().num_positions(), kNumPositions);
  ASSERT_EQ(tree_autodiff().num_states(), kNumStates);

  ASSERT_EQ(context_->num_continuous_states(), kNumStates);
  ASSERT_EQ(context_autodiff_->num_continuous_states(), kNumStates);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  // Get a set of joint angles and angular rates that avoids in-plane motion.
  VectorX<double> q, v;
  GetArbitraryNonZeroJointAnglesAndRates(&q, &v);

  // Set joint angles and rates.
  int angle_index = 0;
  for (const RevoluteJoint<double>* joint : joints_) {
    joint->set_angle(context_.get(), q[angle_index]);
    joint->set_angular_rate(context_.get(), v[angle_index]);
    angle_index++;
  }

  // Three arbitrary frames on the robot.
  const RigidBody<double>& link3 = tree().GetRigidBodyByName("iiwa_link_3");
  const RigidBody<double>& link5 = tree().GetRigidBodyByName("iiwa_link_5");
  const RigidBody<double>& link7 = tree().GetRigidBodyByName("iiwa_link_7");

  // An arbitrary point Q in the end effector link 7.
  const Vector3d p_L7Q = Vector3d(0.2, -0.1, 0.5);

  // Link 3 kinematics.
  const RigidTransform<double>& X_WL3 =
      tree().EvalBodyPoseInWorld(*context_, link3);
  const RotationMatrix<double>& R_WL3 = X_WL3.rotation();

  // link 5 kinematics.
  const RigidTransform<double>& X_WL5 =
      tree().EvalBodyPoseInWorld(*context_, link5);
  const RotationMatrix<double>& R_WL5 = X_WL5.rotation();

  // link 7 kinematics.
  const RigidTransform<double>& X_WL7 =
      tree().EvalBodyPoseInWorld(*context_, link7);
  const RotationMatrix<double>& R_WL7 = X_WL7.rotation();

  // Position of Q in L3, expressed in world.
  // Spatial velocity of frame L3 shifted to Q.
  const RigidTransform<double> X_L3L7 = tree().CalcRelativeTransform(
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
  tree().CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kV, link7.body_frame(), p_L7Q,
      link3.body_frame(), link5.body_frame(), &Jv_L3L7q_L5);

  // Verify that V_L3L7q = Jv_L3L7q * v:
  const SpatialVelocity<double> Jv_L3L7q_L5_times_v(Jv_L3L7q_L5 * v);

  EXPECT_TRUE(Jv_L3L7q_L5_times_v.IsApprox(V_L3L7q_L5, kTolerance));

  // Unit test that CalcJacobianSpatialVelocity throws an exception when
  // the input Jacobian is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcJacobianSpatialVelocity(
          *context_, JacobianWrtVariable::kV, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), nullptr),
      ".*'Js_V_ABp_E != nullptr'.*");

  // Unit test that CalcJacobianSpatialVelocity throws an exception when
  // the input Jacobian has the wrong number of rows.
  MatrixX<double> Jv_wrong_size;
  Jv_wrong_size.resize(9, tree().num_velocities());
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcJacobianSpatialVelocity(
          *context_, JacobianWrtVariable::kV, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), &Jv_wrong_size),
      ".*'Js_V_ABp_E->rows\\(\\) == 6'.*");

  // Unit test that CalcJacobianSpatialVelocity throws an exception when
  // the input Jacobian has the wrong number of columns.
  Jv_wrong_size.resize(6, 23);
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree().CalcJacobianSpatialVelocity(
          *context_, JacobianWrtVariable::kV, link7.body_frame(), p_L7Q,
          link3.body_frame(), link5.body_frame(), &Jv_wrong_size),
      ".*'Js_V_ABp_E->cols\\(\\) == num_columns'.*");
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
    const auto M_B = SpatialInertia<double>::NaN();

    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    body1_ = &model->AddRigidBody("body1", M_B);
    body2_ = &model->AddRigidBody("body2", M_B);

    model->AddJoint(std::make_unique<WeldJoint<double>>(
        "weld0", model->world_body().body_frame(), body1_->body_frame(),
        X_WB1_));

    // Add a weld joint between bodies 1 and 2 by welding together inboard
    // frame F (on body 1) with outboard frame M (on body 2).
    const auto& frame_F =
        model->AddFrame<FixedOffsetFrame>("F", *body1_, X_B1F_);
    const auto& frame_M =
        model->AddFrame<FixedOffsetFrame>("M", *body2_, X_B2M_);
    model->AddJoint(
        std::make_unique<WeldJoint<double>>("weld1", frame_F, frame_M, X_FM_));

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    // Create a context to store the state for this tree:
    context_ = system_->CreateDefaultContext();

    // Expected pose of body 2 in the world.
    X_WB2_.set_translation(Vector3d(M_SQRT2 / 4, -M_SQRT2 / 4, 0.0) -
                           Vector3d::UnitY() / M_SQRT2);
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
  EXPECT_EQ(context_->num_continuous_states(), 0);
}

TEST_F(WeldMobilizerTest, PositionKinematics) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  std::vector<math::RigidTransformd> body_poses;
  tree().CalcAllBodyPosesInWorld(*context_, &body_poses);

  EXPECT_TRUE(body_poses[body1_->index()].IsNearlyEqualTo(X_WB1_, kTolerance));
  EXPECT_TRUE(body_poses[body2_->index()].IsNearlyEqualTo(X_WB2_, kTolerance));
}

// Verify proper operation of the Joint internal use only method for generating
// unique frame names.
GTEST_TEST(JointTest, UniqueFrameName) {
  // Spatial inertia for each body. The actual value is not important for
  // these tests since they are all kinematic.
  const auto M_B = SpatialInertia<double>::Zero();

  // Create an empty model.
  auto model = std::make_unique<MultibodyTree<double>>();

  const RigidBody<double>& body1 = model->AddRigidBody("body1", M_B);
  const RigidBody<double>& body2 = model->AddRigidBody("body2", M_B);

  const auto& frame1 = model->AddFrame<FixedOffsetFrame>(
      "frame1", body1, RigidTransform<double>());
  const auto& frame2 = model->AddFrame<FixedOffsetFrame>(
      "frame2", body2, RigidTransform<double>());

  const Joint<double>& joint0 =
      model->AddJoint(std::make_unique<WeldJoint<double>>(
          "joint0", frame1, frame2, RigidTransform<double>()));

  EXPECT_EQ(joint0.MakeUniqueOffsetFrameName(frame1, "foo"),
            "joint0_frame1_foo");

  // Verify that disambiguation works in case of a name collision.
  model->AddFrame<FixedOffsetFrame>("joint0_frame2_foo", body2,
                                    RigidTransform<double>());
  EXPECT_EQ(joint0.MakeUniqueOffsetFrameName(frame2, "foo"),
            "_joint0_frame2_foo");
}

}  // namespace
}  // namespace multibody_model
}  // namespace internal
}  // namespace multibody
}  // namespace drake
