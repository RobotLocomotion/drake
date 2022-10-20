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
  auto atlas_instance = parser.AddModelFromFile(model_path);
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
  Parser(&plant).AddModelFromFile(sdf_file);
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

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-mass body that is allowed to translate due to a prismatic joint.
GTEST_TEST(TestSingularHingeMatrix, ThrowErrorForZeroMassTranslatingBodyA) {
  // Create a plant with discrete_update_period = 0 to set a continuous model
  // that uses the Articulated Body Algorithm (ABA) for forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  double mA = 0;  // Mass of link A.
  const double length = 3;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);

  // Add bodyA to world with X-prismatic joint (bodyA has zero mass).
  const RigidBody<double>& world_body = plant.world_body();
  plant.AddJoint<multibody::PrismaticJoint>("WA_prismatic_jointX",
      world_body, std::nullopt, body_A, std::nullopt, Vector3<double>::UnitX());

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();
  systems::Context<double>* context_ptr = context.get();

  // Verify proper error message is thrown when HingeMatrix = [0].
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is "
    "not positive definite and equal to \\[0\\]. "
    "Since the joint allows translation, ensure body bodyA has a reasonable "
    "non-zero mass. Note: The mass of body bodyA is 0. ");

  // Verify assertion is thrown if mA = 1E-13.
  body_A.SetMass(context_ptr, mA = 1E-13);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is "
    "nearly singular and equal to \\[1e-13\\]. "
    "Since the joint allows translation, ensure body bodyA has a reasonable "
    "non-zero mass. Note: The mass of body bodyA is 1e-13. ");

  // Verify no assertion is thrown if mA = 1E-4.
  // HingeMatrix ≈ [1E-4] which is regarded as non-singular.
  body_A.SetMass(context_ptr, mA = 1E-4);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))
}

// Verify an exception is thrown for a forward dynamic analysis of a single
// zero-inertia body that is allowed to rotate due to a revolute joint.
GTEST_TEST(TestSingularHingeMatrix, ThrowErrorForZeroInertiaRotatingBody) {
  // Create a plant with discrete_update_period = 0 to set a continuous model
  // that uses the Articulated Body Algorithm (ABA) for forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  double mA = 0;  // Mass of link A.
  const double length = 1;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);

  // Add bodyA to world with Z-revolute joint (bodyA has zero mass/inertia).
  const RigidBody<double>& world_body = plant.world_body();
  plant.AddJoint<multibody::RevoluteJoint>("WA_revolute_jointZ",
      world_body, std::nullopt, body_A, std::nullopt, Vector3<double>::UnitZ());

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();
  systems::Context<double>* context_ptr = context.get();

  // Verify proper error message is thrown when HingeMatrix = [0].
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is "
    "not positive definite and equal to \\[0\\]. "
    "Since the joint allows rotation, ensure body bodyA has reasonable "
    "non-zero moments of inertia about joint rotation axes. "
    "Note: The inertia matrix of body bodyA about its body origin is [^]*");

  // Verify an assertion is thrown if mA = 1E-11.
  // HingeMatrix ≈ [4.16667e-12], so this is regarded as near-singular.
  body_A.SetMass(context_ptr, mA = 1E-11);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is nearly singular "
    "and equal to \\[[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\]. "
    "Since the joint allows rotation, ensure body bodyA has reasonable "
    "non-zero moments of inertia about joint rotation axes. "
    "Note: The inertia matrix of body bodyA about its body origin is [^]*");

  // Verify no assertion is thrown if mA = 1E-4 (far enough from singular).
  body_A.SetMass(context_ptr, mA = 1E-4);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))
}

// Verify an exception may be thrown for a forward dynamic analysis that has
// sequential rigid bodies A and B that translate in the same direction, where
// body A's mass may be disproportionally small (or lage) relative to B's mass.
GTEST_TEST(TestSingularHingeMatrix, DisproportionateMassTranslatingBodiesAB) {
  // Create a plant with discrete_update_period = 0 to set a continuous model
  // that uses the Articulated Body Algorithm (ABA) for forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  double mA = 1E-11, mB = 1E9;  // Mass of links A, B.
  const double length = 1;  // Length of uniform-density link (arbitrary > 0).
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&plant, "bodyB", mB, length);

  // Add bodyA to world with X-prismatic joint.
  const RigidBody<double>& world_body = plant.world_body();
  plant.AddJoint<multibody::PrismaticJoint>("WA_prismatic_jointX",
      world_body, std::nullopt, body_A, std::nullopt, Vector3<double>::UnitX());

  // Add bodyB to bodyA with X-prismatic joint.
  plant.AddJoint<multibody::PrismaticJoint>("AB_prismatic_jointX",
      body_A, std::nullopt, body_B, std::nullopt, Vector3<double>::UnitX());

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();
  systems::Context<double>* context_ptr = context.get();

  // Verify proper assertion is thrown if mA = 1E-11, mB = 1E9.
  // HingeMatrix ≈ [-2.38409e-07], negative due to round-off (so singular).
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is not positive "
    "definite and equal to \\[[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\]. "
    "Since the joint allows translation, ensure body bodyA has a reasonable "
    "non-zero mass. Note: The mass of body bodyA is 1e-11. ");

  // Verify no assertion is thrown if mA = 1E-3, mB = 1E9.
  body_A.SetMass(context_ptr, mA = 1E-3);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))

  // Verify no assertion is thrown if mA = 1E9, mB = 1E-11.
  body_A.SetMass(context_ptr, mA = 1E9);
  body_B.SetMass(context_ptr, mB = 1E-11);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body bodyA to body bodyB is nearly singular "
    "and equal to \\[[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\]. "
    "Since the joint allows translation, ensure body bodyB has a reasonable "
    "non-zero mass. Note: The mass of body bodyB is 1e-11. ");
}

// Perform a forward dynamic analysis for a planar triple pendulum consisting of
// rigid bodies A, B, C rotating in the world's Z-direction.
// Verify an exception is thrown if body C's mass and inertia are zero or if
// bodies A and B's mass and inertia are disproportionally small relative to C.
GTEST_TEST(TestSingularHingeMatrix, DisproportionateInertiaRotatingBodiesBC) {
  // Create a plant with discrete_update_period = 0 to set a continuous model
  // that uses the Articulated Body Algorithm (ABA) for forward dynamics.
  const double discrete_update_period = 0;
  MultibodyPlant<double> plant(discrete_update_period);

  // World X is vertically downward and world Y is horizontally-right.
  plant.mutable_gravity_field().set_gravity_vector(Vector3d(9.8, 0, 0));

  // Create the bodies in the triple pendulum.
  // Reminder: Cubical link B has p_BoBcm_B = [length/2, 0, 0].
  double mA = 1, mB = 1, mC = 0;  // Mass of links A, B, C.
  const double length = 0.2;      // Length of uniform-density links A, B, C.
  const RigidBody<double>& body_A = AddCubicalLink(&plant, "bodyA", mA, length);
  const RigidBody<double>& body_B = AddCubicalLink(&plant, "bodyB", mB, length);
  const RigidBody<double>& body_C = AddCubicalLink(&plant, "bodyC", mC, length);

  // Add body A to world W  with a Z-revolute joint.
  const RigidBody<double>& world_body = plant.world_body();
  const RevoluteJoint<double>& WA_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("WA_revolute_jointZ",
      world_body, std::nullopt, body_A, std::nullopt, Vector3<double>::UnitZ());

  // Add body B to body A with a Z-revolute joint. To do this, create a
  // "fixed" frame Af at the X-distal end of link A that connects to B.
  const Vector3d p_AoAfo_A(length, 0, 0);  // Position vector from Ao to Afo.
  const math::RigidTransformd X_AAf(p_AoAfo_A);  // Rigid transform from A to Af
  const RevoluteJoint<double>& AB_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("AB_revolute_jointZ",
      body_A, X_AAf, body_B, std::nullopt, Vector3<double>::UnitZ());

  // Add body C to body B with a Z-revolute joint. To do this, create a
  // "fixed" frame Bf at the X-distal end of link B that connects to C.
  const Vector3d p_BoBfo_B(length, 0, 0);  // Position vector from Bo to Bfo.
  const math::RigidTransformd X_BBf(p_BoBfo_B);  // Rigid transform from B to Bf
  const RevoluteJoint<double>& BC_revolute_jointZ =
      plant.AddJoint<multibody::RevoluteJoint>("BC_revolute_jointZ",
      body_B, X_BBf, body_C, std::nullopt, Vector3<double>::UnitZ());

  // Signal that we are done building the test model.
  plant.Finalize();

  // Create a default context and evaluate forward dynamics.
  auto context = plant.CreateDefaultContext();
  systems::Context<double>* context_ptr = context.get();
  WA_revolute_jointZ.set_angle(context_ptr, 0);
  AB_revolute_jointZ.set_angle(context_ptr, 0);
  BC_revolute_jointZ.set_angle(context_ptr, 0);

  // Verify proper assertion is thrown if mA = 1, mB = 1, mc = 0.
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body bodyB to body bodyC is "
    "not positive definite and equal to \\[0\\]. "
    "Since the joint allows rotation, ensure body bodyC has reasonable "
    "non-zero moments of inertia about joint rotation axes. "
    "Note: The inertia matrix of body bodyC about its body origin is [^]*");

  // Verify no assertion is thrown if mA = 1, mB = 1, mC = 1E-13.
  // HingeMatrix ≈ [1.67e-15], so this is regarded as near-singular.
  body_C.SetMass(context_ptr, mC = 1E-13);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body bodyB to body bodyC is nearly singular "
    "and equal to \\[[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\]. "
    "Since the joint allows rotation, ensure body bodyC has reasonable "
    "non-zero moments of inertia about joint rotation axes. "
    "Note: The inertia matrix of body bodyC about its body origin is [^]*");

  // Verify no assertion is thrown if mA = 1, mB = 0, mC = 1.
  body_B.SetMass(context_ptr, mB = 0);
  body_C.SetMass(context_ptr, mC = 1);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))

  // Verify assertion is thrown if mA = 0, mB = 0, mC = 1.
  // HingeMatrix ≈ [-1.90126e-17], so this is regarded as singular.
  body_A.SetMass(context_ptr, mA = 0);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
    "The articulated body hinge inertia matrix associated with "
    "the joint that connects body WorldBody to body bodyA is not positive "
    "definite and equal to \\[[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\]. "
    "Since the joint allows rotation, ensure body bodyA has reasonable "
    "non-zero moments of inertia about joint rotation axes. "
    "Note: The inertia matrix of body bodyA about its body origin is [^]*");

  // No assertion is thrown for these non-zero angles, even though angular
  // accelerations are unusually large (hinge matrix is semi-near singular).
  WA_revolute_jointZ.set_angle(context_ptr, 5 * M_PI/180);
  AB_revolute_jointZ.set_angle(context_ptr, 7 * M_PI/180);
  BC_revolute_jointZ.set_angle(context_ptr, 9 * M_PI/180);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))

  // Drake's angular accelerations are large and are nearly identical to those
  // produced by MotionGenesis (MG) for this same problem. The MG simulation
  // for this simulation fails at t = 0.0124 seconds with:
  // qAddot = 2.6E+10   qBddot = -5.2E+10  qCddot = 2.6E+010.
  const VectorX<double>& vdot = plant.EvalForwardDynamics(*context).get_vdot();
  const Vector3d qddot_expected(393.28375042294266,   // qAddot
                               -793.82369625837146,   // qBddot
                                400.53994583542919);  // qCddot
  constexpr double kAngularAccelerationTolerance = 1E-8;
  EXPECT_NEAR(vdot(0), qddot_expected(0), kAngularAccelerationTolerance);
  EXPECT_NEAR(vdot(1), qddot_expected(1), kAngularAccelerationTolerance);
  EXPECT_NEAR(vdot(2), qddot_expected(2), kAngularAccelerationTolerance);

  // Verify that an assertion is thrown when the initial revolute angles for WA,
  // AB, BC are 5 degrees, theta_AB, and 9 degrees, respectively. The hinge
  // matrix is either not positive definite or near singular when the theta_AB
  // revolute angle is very small. Hinge matrices for joint angles (in degrees):
  // theta_AB_deg = 1E-6  HingeMatrix ≈ [3.3E-17].
  // theta_AB_deg = 1E-5  HingeMatrix ≈ [1.2E-15].
  // theta_AB_deg = 1E-4  HingeMatrix ≈ [1.2E-13].
  // theta_AB_deg = 1E-3  HingeMatrix ≈ [1.2E-11].
  double theta_AB_deg = 1E-6 * M_PI/180;
  for (int i = 0; i < 4; ++i) {
     AB_revolute_jointZ.set_angle(context_ptr,  theta_AB_deg);
     DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalForwardDynamics(*context),
       "The articulated body hinge inertia matrix associated with "
       "the joint that connects body WorldBody to body bodyA is [^]*");
     theta_AB_deg *= 10;
  }

  // Verify no assertion is thrown when the initial revolute angles for WA, AB,
  // and BC are 5 degrees, 0.1, and 9 degrees, respectively, i.e., with the
  // initial AB revolute angle sufficiently far from zero.
  AB_revolute_jointZ.set_angle(context_ptr,  1E-1 * M_PI/180);
  DRAKE_EXPECT_NO_THROW(plant.EvalForwardDynamics(*context))
}


}  // namespace
}  // namespace multibody
}  // namespace drake
