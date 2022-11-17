#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/linear_system.h"

using drake::math::RigidTransformd;
using drake::systems::Context;
using drake::test::LimitMalloc;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static VectorX<double> CalcGeneralizedAccelerations(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalForwardDynamics(context).get_vdot();
  }
};

namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

// Fixture to perform forward dynamics tests on a model of a KUKA Iiwa arm. The
// base is free.
class KukaIiwaModelForwardDynamicsTests : public test::KukaIiwaModelTests {
 protected:
  // Given the state of the joints in q and v, this method calculates the
  // forward dynamics for the floating KUKA iiwa robot using the articulated
  // body algorithm. The pose and spatial velocity of the base are arbitrary.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] v robot's joint velocities (generalized velocities).
  // @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaArticulatedBodyAlgorithm(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    DRAKE_DEMAND(vdot != nullptr);
    // Update joint positions and velocities.
    VectorX<double> x(q.size() + v.size());
    x << q, v;
    SetState(x);
    *vdot =
        MultibodyPlantTester::CalcGeneralizedAccelerations(*plant_, *context_);
  }

  // This method calculates the forward dynamics for the 7-DOF KUKA iiwa robot
  // by explicitly solving for the inverse of the mass matrix.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] v robot's joint velocities (generalized velocities).
  // @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaMassMatrixSolve(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    DRAKE_DEMAND(vdot != nullptr);
    // Update joint positions and velocities.
    VectorX<double> x(q.size() + v.size());
    x << q, v;
    SetState(x);

    // Compute force element contributions.
    MultibodyForces<double> forces(*plant_);
    plant_->CalcForceElementsContribution(*context_, &forces);

    // Construct M, the mass matrix.
    const int nv = plant_->num_velocities();
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);

    // Compute tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W via inverse
    // dynamics.
    const VectorX<double> zero_vdot = VectorX<double>::Zero(nv);
    const VectorX<double> tau_id =
        plant_->CalcInverseDynamics(*context_, zero_vdot, forces);

    // Solve for vdot.
    *vdot = M.llt().solve(-tau_id);
  }

  // Verify the solution obtained using the ABA against a reference solution
  // computed by explicitly taking the inverse of the mass matrix.
  void CompareForwardDynamics(const Eigen::Ref<const VectorX<double>>& q,
                              const Eigen::Ref<const VectorX<double>>& v) {
    // Compute forward dynamics using articulated body algorithm.
    VectorX<double> vdot(plant_->num_velocities());
    CalcForwardDynamicsViaArticulatedBodyAlgorithm(q, v, &vdot);

    // Compute forward dynamics using mass matrix.
    VectorX<double> vdot_expected(plant_->num_velocities());
    CalcForwardDynamicsViaMassMatrixSolve(q, v, &vdot_expected);

    // We estimate the difference between vdot and vdot_expected to be in the
    // order of machine epsilon times the condition number "kappa" of the mass
    // matrix.
    const int nv = plant_->num_velocities();
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
    const double kappa = 1.0 / M.llt().rcond();

    // Compare expected results against actual vdot.
    const double kRelativeTolerance = kappa * kEpsilon;
    EXPECT_TRUE(CompareMatrices(vdot, vdot_expected, kRelativeTolerance,
                                MatrixCompareType::relative));
  }
};

// This test is used to verify the correctness of the articulated body algorithm
// for solving forward dynamics. The output from the articulated body algorithm
// is compared against the output from solving using the mass matrix. We verify
// the computation for an arbitrary set of robot states.
TEST_F(KukaIiwaModelForwardDynamicsTests, ForwardDynamicsTest) {
  // Joint angles and velocities.
  VectorX<double> q(kNumJoints), qdot(kNumJoints);
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Test 1: Static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  CompareForwardDynamics(q, qdot);

  // Test 2: Another static configuration.
  q << q30, -q45, q60, -q30, q45, -q60, q30;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  CompareForwardDynamics(q, qdot);

  // Test 3: Non-static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  CompareForwardDynamics(q, qdot);

  // Test 4: Another non-static configuration.
  q << -q45, q60, -q30, q45, -q60, q30, -q45;
  qdot << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  CompareForwardDynamics(q, qdot);

  // Test 5: Another non-static configuration.
  q << q30, q45, q60, -q30, -q45, -q60, 0;
  qdot << 0.3, -0.1, 0.4, -0.1, 0.5, -0.9, 0.2;
  CompareForwardDynamics(q, qdot);
}

// For complex articulated systems such as a humanoid robot, round-off errors
// might accumulate leading to (close to, by machine epsilon) unphysical ABIs in
// the Articulated Body Algorithm. See related issue #12640.
// This test verifies this does not trigger a spurious exception.
GTEST_TEST(MultibodyPlantForwardDynamics, AtlasRobot) {
  MultibodyPlant<double> plant(0.0);
  const std::string model_path =
      FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");
  Parser parser(&plant);
  auto atlas_instance = parser.AddModels(model_path).at(0);
  plant.Finalize();

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  for (JointIndex joint_index(0); joint_index < plant.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant.get_joint(joint_index);
    // This model only has weld and revolute joints. Weld joints have zero DOFs.
    if (joint.num_velocities() != 0) {
      const RevoluteJoint<double>& revolute_joint =
          dynamic_cast<const RevoluteJoint<double>&>(joint);
      // Arbitrary non-zero angle.
      revolute_joint.set_angle(context.get(), 0.5 * joint_index);
    }
  }
  const int num_actuators = plant.num_actuators();
  plant.get_actuation_input_port(atlas_instance)
      .FixValue(context.get(), VectorX<double>::Zero(num_actuators));
  auto derivatives = plant.AllocateTimeDerivatives();
  {
    // CalcTimeDerivatives should not be allocating, but for now we have a few
    // remaining fixes before it's down to zero:
    //  2 temps in MbTS::CalcArticulatedBodyForceCache (F_B_W_, tau_).
    //  1 temp  in MbP::AssembleActuationInput (actuation_input).
    //  2 temps in MbTS::DoCalcTimeDerivatives (xdot, qdot).
    LimitMalloc guard({ .max_num_allocations = 5 });
    EXPECT_NO_THROW(plant.CalcTimeDerivatives(*context, derivatives.get()));
  }

  // Verify that the implicit dynamics match the continuous ones.
  Eigen::VectorXd residual = plant.AllocateImplicitTimeDerivativesResidual();
  plant.CalcImplicitTimeDerivativesResidual(*context, *derivatives, &residual);
  // Note the slightly looser tolerance of 4e-13 which was required for this
  // test.
  EXPECT_TRUE(CompareMatrices(
      residual, Eigen::VectorXd::Zero(plant.num_multibody_states()), 4e-13));
}

// Verifies we can do forward dynamics on a model with a zero-sized state.
GTEST_TEST(WeldedBoxesTest, ForwardDynamicsViaArticulatedBodyAlgorithm) {
  // Problem parameters.
  const double kCubeSize = 1.5;  // Size of the box, in meters.
  const double kBoxMass = 2.0;   // Mass of each box, in Kg.
  // We use discrete_update_period = 0 to set a continuous model that uses the
  // Articulated Body Algorithm (ABA) to evaluate forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  // Set a model with two boxes anchored to the world via weld joints.
  const Vector3d p_BoBcm_B = Vector3d::Zero();
  const UnitInertia<double> G_BBcm =
      UnitInertia<double>::SolidBox(kCubeSize, kCubeSize, kCubeSize);
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(kBoxMass, p_BoBcm_B,
                                                     G_BBcm);
  // Create two rigid bodies.
  const auto& boxA = plant.AddRigidBody("boxA", M_BBo_B);
  const auto& boxB = plant.AddRigidBody("boxB", M_BBo_B);

  // Desired transformation for the boxes in the world.
  const RigidTransformd X_WA(Vector3d::Zero());
  const RigidTransformd X_WB(Vector3d(kCubeSize, 0, 0));
  const RigidTransformd X_AB = X_WA.inverse() * X_WB;

  // Pin boxA to the world and boxB to boxA with weld joints.
  plant.WeldFrames(plant.world_body().body_frame(), boxA.body_frame(), X_WA);
  plant.WeldFrames(boxA.body_frame(), boxB.body_frame(), X_AB);

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  // Evaluate forward dynamics.
  const VectorXd vdot =
      MultibodyPlantTester::CalcGeneralizedAccelerations(plant, *context);
  EXPECT_EQ(vdot.size(), 0);
}

std::unique_ptr<systems::LinearSystem<double>> MakeLinearizedCartPole(
    double time_step) {
  const std::string sdf_file = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");

  MultibodyPlant<double> plant(time_step);
  Parser(&plant).AddModels(sdf_file);
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  plant.get_actuation_input_port().FixValue(context.get(), 0.);
  plant.SetPositionsAndVelocities(context.get(),
                                  Eigen::Vector4d{0, M_PI, 0, 0});

  return systems::Linearize(plant, *context,
                            plant.get_actuation_input_port().get_index(),
                            systems::OutputPortSelection::kNoOutput);
}

// This test revealed a bug (#17037) in MultibodyPlant<AutoDiffXd>.
GTEST_TEST(MultibodyPlantTest, CartPoleLinearization) {
  const double kTimeStep = 0.1;
  auto ct_linearization = MakeLinearizedCartPole(0.0);
  auto dt_linearization = MakeLinearizedCartPole(kTimeStep);

  // v_next = v0 + time_step * (A * x + B * u)
  // q_next = q0 + time_step * v_next
  Eigen::Matrix4d A_expected = Eigen::Matrix4d::Identity();
  A_expected.bottomRows<2>() +=
      kTimeStep * ct_linearization->A().bottomRows<2>();
  A_expected.topRows<2>() += kTimeStep * A_expected.bottomRows<2>();
  Eigen::Vector4d B_expected;
  B_expected.bottomRows<2>() =
      kTimeStep * ct_linearization->B().bottomRows<2>();
  B_expected.topRows<2>() = kTimeStep * B_expected.bottomRows<2>();

  EXPECT_TRUE(CompareMatrices(dt_linearization->A(), A_expected, 1e-16));
  EXPECT_TRUE(CompareMatrices(dt_linearization->B(), B_expected, 1e-16));
}
// TODO(amcastro-tri): Include test with non-zero actuation and external forces.

// Helper function to create a uniform-density cube B and add it to a plant.
// @param[in] plant MultibodyPlant to which body B is added.
// @param[in] body_name name of the body that is being added to the plant.
// @param[in] mass mass of the body that is being added to the plant.
// @param[in] length length, width, and depth of the cube-shaped body.
// @param[in] skip_validity_check setting which is `true` to skip the validity
//  check on the new body B's spatial inertia, which ensures an exception is not
//  thrown when setting body B's spatial inertia (which would otherwise occur if
//  mass or link_length is NaN). Avoiding this early exception allows for a
//  later exception to be thrown in a subsequent function and tested below.
// @note The position vector from Bcm (B's center of mass which is at the cube's
// geometric center) to Bo (B's origin) is p_BcmBo = (-length/2, 0, 0).
const RigidBody<double>& AddCubicalLink(
    MultibodyPlant<double>* plant,
    const std::string& body_name,
    const double mass,
    const double length,
    const bool skip_validity_check = false) {
  DRAKE_DEMAND(plant != nullptr);
  const Vector3<double> p_BoBcm_B(length / 2, 0, 0);
  const UnitInertia<double> G_BBcm_B = UnitInertia<double>::SolidCube(length);
  const UnitInertia<double> G_BBo_B =
      G_BBcm_B.ShiftFromCenterOfMass(-p_BoBcm_B);
  const SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B,
                                       skip_validity_check);
  return plant->AddRigidBody(body_name, M_BBo_B);
}

// Fixture for a MultibodyPlant with connected rigid bodies and a Context.
class ConnectedRigidBodiesTest : public ::testing::Test {
 public:
  // @param[in] mA mass of link A (first link in the multibody plant).
  // @param[in] LA length of uniform-density link A.
  void MakePlant(const std::string& jointA_type_name,
                 const double mA, const double LA,
                 const std::string* jointB_type_name = nullptr,
                 const double mB = 0, const double LB = 0,
                 const std::string* jointC_type_name = nullptr,
                 const double mC = 0, const double LC = 0) {
    // Connect bodyA to world with the designate joint type.
    bodyA_ = &(AddCubicalLink(&plant_, "bodyA", mA, LA));
    const RigidBody<double>& world_body = plant_.world_body();
    const RigidTransform<double> X_WF;
    jointWA_ = AddJointToTestPlant(jointA_type_name,
        world_body, X_WF, *bodyA_, RigidTransform<double>::Identity());

    // Connect bodyB to bodyA with the designate joint type. Create a "fixed"
    // frame F at the x-distal end of link A that connects to link B.
    if (jointB_type_name != nullptr) {
      bodyB_ = &AddCubicalLink(&plant_, "bodyB", mB, LB);
      const RigidTransform<double> X_AF(Vector3<double>(LA, 0, 0));
      jointAB_ = AddJointToTestPlant(*jointB_type_name,
          *bodyA_, X_AF, *bodyB_, RigidTransform<double>::Identity());
    }

    // Connect bodyC to bodyB with the designate joint type. Create a "fixed"
    // frame F at the x-distal end of link B that connects to link C.
    if (jointC_type_name != nullptr) {
      bodyC_ = &AddCubicalLink(&plant_, "bodyC", mC, LC);
      const RigidTransform<double> X_BF(Vector3<double>(LB, 0, 0));
      jointBC_ = AddJointToTestPlant(*jointB_type_name,
          *bodyB_, X_BF, *bodyC_, RigidTransform<double>::Identity());
    }

    // Signal that we are done building the test model.
    plant_.Finalize();

    // Create a default context for subsequent use evaluating forward dynamics.
    context_ = plant_.CreateDefaultContext();
  }

 protected:
  // Make a plant with constructor argument = 0 to signal use of a continuous
  // model (and hence the Articulated Body Algorithm for forward dynamics).
  MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<Context<double>> context_{nullptr};
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RigidBody<double>* bodyC_{nullptr};
  const RevoluteJoint<double>* jointWA_{nullptr};
  const RevoluteJoint<double>* jointAB_{nullptr};
  const RevoluteJoint<double>* jointBC_{nullptr};

  // Helper function to add a joint to this plant.
  // @param[in] joint_type_name name of a Drake joint, e.g., "revolute".
  // @param[in] bodyA inboard body to be connected to joint.
  // @param[in] X_AF rigid transform relating body frame A to "fixed-frame" F
  //   that is attached to body A and connects via the joint to body B.
  // @param[in] bodyB outboard body to be connected to joint.
  // @param[in] X_BM rigid transform relating body frame B to the frame M
  //   that is attached to body B and connects via the joint to body A.
  // @note If joint_type_name is prismatic, the joint has a Fx direction, where
  //   Fx is a unit vector defined by the "fixed_frame" F. If joint_type_name is
  //   revolute, the joint has a Fz direction.
  // @throws std::exception if joint_type_name is not "revolute", "prismatic",
  //   or "FreeJoint".
  // @returns pointer to a revolute joint (if one is created), otherwise null.
  const RevoluteJoint<double>* AddJointToTestPlant(
      const std::string& joint_type_name,
      const RigidBody<double>& bodyA, const RigidTransform<double>& X_AF,
      const RigidBody<double>& bodyB, const RigidTransform<double>& X_BM) {
    const RevoluteJoint<double>* joint{nullptr};
    const std::string joint_name =
        bodyA.name() + "_" + bodyB.name() + "_" + joint_type_name;
    if (joint_type_name == PrismaticJoint<double>::kTypeName) {
      plant_.AddJoint<PrismaticJoint>(joint_name,
          bodyA, X_AF, bodyB, X_BM, Vector3<double>::UnitX());
    } else if (joint_type_name == RevoluteJoint<double>::kTypeName) {
      joint = &plant_.AddJoint<RevoluteJoint>(joint_name,
          bodyA, X_AF, bodyB, X_BM, Vector3<double>::UnitZ());
    } else if (joint_type_name != "FreeJoint") {  // Do nothing for free joint!
      const std::string message = "The test fixture ConnectedRigidBodiesTest "
        "does not support a joint of type " + joint_type_name + ".\n";
      throw std::runtime_error(message);
    }
    return joint;  // Reminder: FreeJoint does not actually create a joint.
  }
};

// ----------------------------------------------------------------------------
// There are 5 tests below whose purpose is to improve MultibodyPlant feedback
// for invalid mass/inertia properties. The tests below uses forward dynamics in
// continuous mode and test whether an exception is thrown for invalid
// mass/inertia properties that are detected in the Articulated Body Algorithm.
// The first two tests use a single body that is connected to world by a one
// degree-of-freedom joint. The 1ˢᵗ test uses a prismatic (translational) joint
// whereas the 2ⁿᵈ test uses a revolute joint. These first two tests ensure an
// exception is thrown for a single body that is either translating or rotating
// and is at the end of a topological chain.
// Tests 3 and 4 use multiple bodies with multiple degree of freedom (albeit
// with each joint being a single degree of freedom). Test 3 uses prismatic
// joints whereas test 4 uses revolute joints. Tests 3 and 4 ensure an exception
// can be regardless of whether or not the problematic joint is the last in a
// topological chain of bodies. Also, since the expected tolerances for rotating
// bodies are several orders of magnitude smaller than translating bodies, both
// translational and rotational joints are tested.
// Test 5 uses a single rigid body that connects to the world with a 6
// degree-of-freedom "free-joint" and hence involves a 6 x 6 articulated body
// hinge inertia matrix (whereas tests 1, 2, 3, 4 only involve a 1 x 1 matrix).
// TODO(Mitiguy) Shorten this comment and give more "why" and less "what".
// ----------------------------------------------------------------------------

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-mass body that is allowed to translate due to a prismatic joint.
TEST_F(ConnectedRigidBodiesTest, ThrowErrorForZeroMassTranslatingBody) {
  const double mA = 0;  // Mass of link A.
  const double LA = 3;  // Length of uniform-density link (arbitrary > 0).
  MakePlant(PrismaticJoint<double>::kTypeName, mA, LA);

  // Verify proper assertion is thrown if mA = 0 since articulated body hinge
  // inertia matrix = [0] which is not positive definite.
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows translation,[^]*");

  // Verify no assertion is thrown if mA = 1E-4 since articulated body hinge
  // inertia matrix = [1E-4] which is positive definite (and far from singular).
  bodyA_->SetMass(context_.get(), 1E-4);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))
}

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-inertia body that is allowed to rotate due to a revolute joint.
TEST_F(ConnectedRigidBodiesTest, ThrowErrorForZeroInertiaRotatingBody) {
  const double mA = 0;  // Mass of link A.
  const double LA = 1;  // Length of uniform-density link (arbitrary > 0).
  MakePlant(RevoluteJoint<double>::kTypeName, mA, LA);

  // Verify proper assertion is thrown if mA = 0 since articulated body hinge
  // inertia matrix = [0] which is not positive definite.
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if mA = 1E-4 since articulated body hinge
  // inertia matrix is positive definite (and far from singular).
  bodyA_->SetMass(context_.get(), 1E-4);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))
}

// Verify an exception may be thrown for a forward dynamic analysis that has
// sequential rigid bodies A and B that translate in the same direction, where
// body A's mass is zero and body B's mass is large and positive.
TEST_F(ConnectedRigidBodiesTest, ThrowErrorForZeroMassTranslating2Bodies) {
  const double mA = 0, mB = 1;  // Mass of links A and B.
  const double LA = 1, LB = 1;  // Length of uniform-density links.
  const std::string prismatic(PrismaticJoint<double>::kTypeName);
  MakePlant(prismatic, mA, LA, &prismatic, mB, LB);

  // Verify assertion is thrown if mA = 0, mB = 1 since articulated body hinge
  // inertia matrix = [0] which is not positive definite.
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows translation,[^]*");

  // Verify assertion is thrown if mA = 0, mB = 1E9 since articulated body hinge
  // inertia matrix ≈ [-2.38419e-07] which is not positive definite.
  // Note: Mathematically, if mA = 0, any real value of mB should still throw.
  // TODO(Mitiguy) It seems surprising that the matrix ≠ [0] since mA = 0.
  //  Explain why matrix is ≈ 10⁸ * machine epsilon distant from [0].
  bodyB_->SetMass(context_.get(), 1E9);
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows translation,[^]*");

  // Verify no assertion is thrown if mA = 1E-3, mB = 1E9 since articulated body
  // hinge inertia matrix is positive definite (and far enough from singular).
  // Note: mA = 1E-3 was chosen from empirical numerical testing with LLT.
  bodyA_->SetMass(context_.get(), 1E-3);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))
}

// Perform a forward dynamic analysis for a planar triple pendulum consisting of
// rigid bodies A, B, C, each which has an inboard revolute-pin axis that is
// parallel to the world's Z-direction. Verify an exception is thrown if body
// C's mass and inertia are zero or bodies A and B's mass and inertia are zero.
TEST_F(ConnectedRigidBodiesTest, ThrowErrorForZeroInertiaRotating3Bodies) {
  const double mA = 0, mB = 1, mC = 0;  // Mass of links A, B, C.
  const double L = 0.2;  // Length of uniform-density links A, B, C.
  const std::string revolute(RevoluteJoint<double>::kTypeName);
  MakePlant(revolute, mA, L, &revolute, mB, L, &revolute, mC, L);

  // World X is vertically downward and world Y is horizontally-right.
  plant_.mutable_gravity_field().set_gravity_vector(Vector3d(9.8, 0, 0));

  // Create a default context and evaluate forward dynamics.
  systems::Context<double>* context_ptr = context_.get();
  jointWA_->set_angle(context_ptr, 0);
  jointAB_->set_angle(context_ptr, 0);
  jointBC_->set_angle(context_ptr, 0);

  // Verify proper assertion is thrown if mA = mB = 1, mc = 0 since articulated
  // body hinge inertia matrix = [0] which is not positive definite.
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body bodyB to body bodyC "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if mA = 1, mB = 0, mC = 1.
  bodyA_->SetMass(context_ptr, 1);
  bodyB_->SetMass(context_ptr, 0);
  bodyC_->SetMass(context_ptr, 1);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))

  // Verify proper assertion is thrown if mA = mB = 0, mC = 1 since articulated
  // body hinge inertia matrix ≈ [-1.90126e-17] which is not positive-definite.
  // TODO(Mitiguy) Improve robustness of test. If mC = 2, no assertion is thrown
  //  since the articulated body hinge inertia matrix ≈ [1.582e-17].
  //  The tests herein were chosen because they worked -- based on computation
  //  in computer hardware available from CI (Continuous Integration) testing.
  bodyA_->SetMass(context_ptr, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows rotation,[^]*");

  // Verify no assertion is thrown if the initial revolute angles for WA and BC
  // are each 0 degrees and AB's initial revolute angle is far-enough from zero.
  jointAB_->set_angle(context_ptr,  0.1 * M_PI/180);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))
}

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-mass, zero-inertia free body (both translates and rotates).
TEST_F(ConnectedRigidBodiesTest, ThrowErrorForZeroMassInertiaFreeBody) {
  const double mA = 0;    // Mass of link A.
  const double LA = 0.3;  // Length of uniform-density link (arbitrary > 0).
  MakePlant("FreeJoint", mA, LA);

  // Verify assertion is thrown if mA = 0 since articulated body hinge inertia
  // matrix is 6 x 6 zero matrix (albeit with NaN in upper-triangular part).
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.EvalForwardDynamics(*context_),
    "An internal mass matrix .+ body world to body bodyA "
    "is not positive-definite. .+ allows rotation.+ translation[^]*");

  // Verify no assertion is thrown if mA = 1E-4 since articulated body hinge
  // inertia matrix is positive definite (and far from singular).
  bodyA_->SetMass(context_.get(), 1E-4);
  DRAKE_EXPECT_NO_THROW(plant_.EvalForwardDynamics(*context_))
}

}  // namespace
}  // namespace multibody
}  // namespace drake
