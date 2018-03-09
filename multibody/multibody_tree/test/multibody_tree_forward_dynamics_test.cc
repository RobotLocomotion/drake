#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace multibody_model {
namespace {

using benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using systems::Context;

const double kEpsilon = std::numeric_limits<double>::epsilon();

// Fixture to perform forward dynamics tests on the KUKA Iiwa model.
class KukaIiwaModelForwardDynamicsTests : public ::testing::Test {
 public:
  /// Creates MultibodyTree for a KUKA Iiwa robot arm.
  void SetUp() override {
    model_ = MakeKukaIiwaModel<double>(true /* Finalize model */, gravity_);
    context_ = model_->CreateDefaultContext();

    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
    joints_.push_back(&model_->GetJointByName<RevoluteJoint>("iiwa_joint_7"));
  }

  /// Sets the configuration of this KUKA Iiwa robot arm.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v robot's joint velocities (generalized velocities).
  void SetConfiguration(const Eigen::Ref<const VectorX<double>>& q,
                        const Eigen::Ref<const VectorX<double>>& v) {
    int angle_index = 0;
    for (const RevoluteJoint<double>* joint : joints_) {
      joint->set_angle(context_.get(), q[angle_index]);
      joint->set_angular_rate(context_.get(), v[angle_index]);
      angle_index++;
    }
  }

  /// This method calculates the forward dynamics for a 7-DOF KUKA iiwa robot
  /// using the articulated body algorithm given an initial confguration.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v robot's joint velocities (generalized velocities).
  /// @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaArticulatedBodyAlgorithm(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    // Update joint positions and velocities.
    SetConfiguration(q, v);

    // No external forces.
    MultibodyForces<double> applied_forces(*model_);

    // Compute forward dynamics using articulated body algorithm.
    AccelerationKinematicsCache<double> ac(model_->get_topology());
    model_->CalcForwardDynamics(*context_, applied_forces, &ac);
    *vdot = ac.get_vdot();
  }

  /// This method calculates the forward dynamics for a 7-DOF KUKA iiwa robot
  /// by explicitly solving using the mass matrix given an initial confguration.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v robot's joint velocities (generalized velocities).
  /// @param[out] vdot generalized accelerations (1st derivative of v).
  void CalcForwardDynamicsViaMassMatrixSolve(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v,
      EigenPtr<VectorX<double>> vdot) {
    // Update joint positions and velocities.
    SetConfiguration(q, v);

    // Compute position and velocity cache.
    PositionKinematicsCache<double> pc(model_->get_topology());
    VelocityKinematicsCache<double> vc(model_->get_topology());
    model_->CalcPositionKinematicsCache(*context_, &pc);
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);

    // Get model parameters.
    const int nv = model_->num_velocities();
    const int nq = model_->num_bodies();

    // Compute force element contributions.
    MultibodyForces<double> forces(*model_);
    model_->CalcForceElementsContribution(*context_, pc, vc, &forces);

    // Construct M, the mass matrix.
    MatrixX<double> M(nv, nv);
    model_->CalcMassMatrixViaInverseDynamics(*context_, &M);

    // Construct C, the Coriolis term.
    VectorX<double> C(nv);
    std::vector<SpatialAcceleration<double>> A_WB_array(nq);
    std::vector<SpatialForce<double>> F_BMo_W_array(nq);

    const std::vector<SpatialForce<double>>& Fapplied_Bo_W_array =
        forces.body_forces();
    const VectorX<double>& tau_applied_array = forces.generalized_forces();
    model_->CalcInverseDynamics(
        *context_, pc, vc, VectorX<double>::Zero(nv), Fapplied_Bo_W_array,
        tau_applied_array, &A_WB_array, &F_BMo_W_array, &C);

    // Solve for vdot.
    *vdot = M.llt().solve(-C);
  }


 protected:
  // Compare the articulated body algorithm solution for forward dynamics
  // against that of the mass matrix solve method.
  void CompareForwardDynamics(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v) {
    // Compute forward dynamics using articulated body algorithm.
    Vector7d vdot;
    CalcForwardDynamicsViaArticulatedBodyAlgorithm(q, v, &vdot);

    // Compute forward dynamics using mass matrix.
    Vector7d vdot_expected;
    CalcForwardDynamicsViaMassMatrixSolve(q, v, &vdot_expected);

    // Compare expected results against actual vdot.
    const double kTolerance = 50 * kEpsilon;
    EXPECT_TRUE(CompareMatrices(
        vdot, vdot_expected, kTolerance, MatrixCompareType::relative));
  }

  // Acceleration of gravity:
  const double gravity_{9.81};
  // The model plant:
  std::unique_ptr<MultibodyTree<double>> model_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  // Non-owning pointers to the joints:
  std::vector<const RevoluteJoint<double>*> joints_;
};

// This test is used to verify the correctness of the articulated body algorithm
// for solving forward dynamics. The output from the articulated body algorithm
// is compared against the output from solving using the mass matrix.
TEST_F(KukaIiwaModelForwardDynamicsTests, ForwardDynamicsTest) {
  // State variables and helper angles.
  Vector7d q, qdot;
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

}  // namespace
}  // namespace multibody_model
}  // namespace multibody
}  // namespace drake
