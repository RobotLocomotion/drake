#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/systems/framework/context.h"

using drake::systems::Context;

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
  EXPECT_NO_THROW(plant.CalcTimeDerivatives(*context, derivatives.get()));
}

// TODO(amcastro-tri): Include test with non-zero actuation and external forces.

}  // namespace
}  // namespace multibody
}  // namespace drake
