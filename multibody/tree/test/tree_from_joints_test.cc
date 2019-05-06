// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using benchmarks::Acrobot;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

// Set of MultibodyTree tests for a double pendulum model. This simple set of
// tests serves as an example on how to build a model using RigidBody and Joint
// objects.
// This double pendulum is similar to the acrobot model described in Section 3.1
// of the Underactuated Robotics notes available online at
// http://underactuated.csail.mit.edu/underactuated.html?chapter=3.
// The only difference is that this model has no actuation.
// This double pendulum is defined in the x-y plane with gravity acting in the
// negative y-axis direction.
// In this model the two links of the pendulum have the same length with their
// respective centers of mass located at the links' centroids.
//
// The schematic below shows the location and relationship of the frames defined
// by the model. A few comments:
//  - The pendulum moves in the x-y plane, with angles θ₁ and θ₂ defined
//    positive according to the right-hand-rule with the thumb aligned in the
//    z-direction.
//  - The body frame for link 1 is placed at its geometric center.
//  - The body frame for link 2 is placed at the joint's outboard frame.
//  - The origins of the shoulder frames (`Si` and `So`) are coincident at all
//    times. `So` is aligned with `Si` for `θ₁ = 0`.
//  - The origins of the elbow frames (`Ei` and `Eo`) are coincident at all
//    times. `Eo` is aligned with `Ei` for `θ₂ = 0`.
//  - The elbow outboard frame `Eo` IS the link 2 frame `L2`, refer to schematic
//    below.
//
//       y ^
//         | Si ≡ W World body frame.
//         +--> x  Shoulder inboard frame Si coincides with W.
//      X_SiSo(θ₁) Shoulder revolute joint with generalized position θ₁.
//      +--+-----+
//      |  ^     |
//      |  | So  | Shoulder outboard frame So.
//      |  +-->  |
//      |        |
//      | X_L1So | Pose of So in L1.
//      |        |
//      |  ^     |
//      |  | L1  | Upper link body frame L1.
//      |  +-->  |
//      |        |
//      | X_L1Ei | Pose of Ei in L1.
//      |        |
//      |  ^     |
//      |  | Ei  | Elbow inboard frame Ei.
//      |  +-->  |
//      +--------+
//      X_EiEo(θ₂) Elbow revolute joint with generalized position θ₂.
//      +--+-----+
//      |  ^     |
//      |  | Eo  | Elbow outboard frame Eo.
//      |  +-->  | Lower link's frame L2 is coincident with the elbow frame Eo.
//      | Eo ≡ L2|
//      |        |
//      |p_L2oLcm| Position vector of link 2's com measured from the link's
//      |        | frame origin L2o.
//      |  ^     |
//      |  | L2cm| Lower link's frame L2 shifted to its center of mass.
//      |  +-->  |
//      |        |
//      |        |
//      |        |
//      |        |
//      |        |
//      |        |
//      +--------+
template<typename T>
class DoublePendulumModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoublePendulumModel)

  DoublePendulumModel() {
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Position of L1's COM measured in L1, expressed in L1.
    Vector3d p_L1oL1cm = Vector3d::Zero();  // L1 is at the link's COM.
    // Inertia for a thin rod oriented along the y-axis with moment of inertia
    // about an axis perpendicular to y equal to Ic1_:
    UnitInertia<double> G1_L1 =
        UnitInertia<double>::StraightLine(Ic1_, Vector3d::UnitY());
    // Spatial inertia of link 1 about the origin of its frame L1o and
    // expressed in its body frame L1.
    SpatialInertia<double> M1_L1(mass1_, p_L1oL1cm, G1_L1);

    // Inertia for a thin rod oriented along the y-axis with moment of inertia
    // about an axis perpendicular to y equal to Ic2_:
    UnitInertia<double> G2_L2cm =
        UnitInertia<double>::StraightLine(Ic2_, Vector3d::UnitY());
    // Spatial inertia about link 2's center of mass L2cm.
    SpatialInertia<double> M2_L2cm(mass2_, Vector3d::Zero(), G2_L2cm);
    // Since link 2's frame origin L2o is not located at its center of mass
    // L2cm, we must shift M2_L2cm to obtain M2_L2o.
    const Vector3d p_L2oL2cm(0.0, -half_length2_, 0.0);
    SpatialInertia<double> M2_L2 = M2_L2cm.Shift(-p_L2oL2cm);

    // Adds the upper and lower links of the pendulum:
    link1_ = &model->template AddBody<RigidBody>(M1_L1);
    link2_ = &model->template AddBody<RigidBody>(M2_L2);
    world_body_ = &model->world_body();

    // The shoulder joint connects the world with link 1.
    // Its inboard frame, Si, is the world frame.
    // Its outboard frame, So, is at a fixed offset from link 1's frame L1.
    // L1's origin is located at the com of link 1.
    // X_L1So defines the pose of the shoulder outboard frame So in link 1's
    // frame.
    const math::RigidTransformd X_L1So{Vector3d(0.0, half_length1_, 0.0)};

    shoulder_ = &model->template AddJoint<RevoluteJoint>(
        "ShoulderJoint",
        *world_body_,
        {},      /* Default to Identity; frame Si IS the world frame W. */
        *link1_,
        X_L1So,  /* Pose of So in link 1's frame L1. */
        Vector3d::UnitZ()  /* revolute axis */);

    // The elbow is the joint that connects links 1 and 2.
    // An inboard frame Ei is rigidly attached to link 1. It is located
    // at y = -half_link_length_ in the frame of link 1.
    // X_L1Ei specifies the pose of the elbow inboard frame Ei in the body
    // frame L1 of link 1.
    // The elbow's outboard frame Eo is taken to be coincident with link 2's
    // frame L2 (i.e. L2o != L2cm).
    // X_L1Ei defines the pose of the elbow inboard frame Ei link 1's frame L1.
    const math::RigidTransformd X_L1Ei{Vector3d(0.0, -half_length1_, 0.0)};

    elbow_ = &model->template AddJoint<RevoluteJoint>(
        "ElbowJoint",
        *link1_,
        X_L1Ei,  /* Pose of Ei in L1. */
        *link2_,
        {},      /* Default to Identity; frame Eo IS frame L2. */
        Vector3d::UnitZ() /* revolute axis */);

    // Add force element for a constant gravity pointing downwards, that is, in
    // the minus y-axis direction.
    gravity_element_ = &model->mutable_gravity_field();
    gravity_element_->set_gravity_vector(
        Vector3d(0.0, -acceleration_of_gravity_, 0.0));

    // We are done adding modeling elements.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
  }

  double mass1() const { return mass1_; }
  double length1() const { return length1_; }
  double half_length1() const { return half_length1_; }
  double Ic1() const { return Ic1_; }

  double mass2() const { return mass2_; }
  double length2() const { return length2_; }
  double half_length2() const { return half_length2_; }
  double Ic2() const { return Ic2_; }

  double gravity() const { return acceleration_of_gravity_; }

  const RevoluteJoint<T>& shoulder() const { return *shoulder_; }
  const RevoluteJoint<T>& elbow() const { return *elbow_; }

  const UniformGravityFieldElement<T>& gravity_element() const {
    return *gravity_element_;
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

  std::unique_ptr<Context<double>> CreateDefaultContext() const {
    return system_->CreateDefaultContext();
  }

 private:
  std::unique_ptr<internal::MultibodyTreeSystem<T>> system_;

  const Body<T>* world_body_{nullptr};
  // Bodies:
  const RigidBody<T>* link1_{nullptr};
  const RigidBody<T>* link2_{nullptr};
  // Joints:
  const RevoluteJoint<T>* shoulder_{nullptr};
  const RevoluteJoint<T>* elbow_{nullptr};

  // Force elements:
  UniformGravityFieldElement<T>* gravity_element_{nullptr};

  // Pendulum parameters:
  const double length1_ = 1.0;
  const double mass1_ = 1.0;
  // Unit inertia about an axis perpendicular to the rod for link1.
  const double Ic1_ = .083;
  const double length2_ = 2.0;
  const double mass2_ = 1.0;
  // Unit inertia about an axis perpendicular to the rod for link2.
  const double Ic2_ = .33;
  const double half_length1_ = length1_ / 2;
  const double half_length2_ = length2_ / 2;
  // Acceleration of gravity at Earth's surface.
  const double acceleration_of_gravity_ = 9.81;
};

// Creates a DoublePendulumModel class with an underlying MultibodyTree model
// of a double pendulum.
class PendulumTests : public ::testing::Test {
 public:
  void SetUp() override {
      context_ = pendulum_.CreateDefaultContext();
  }

  // For the double pendulum system it turns out that the mass matrix is only a
  // function of the elbow angle, theta2, independent of the shoulder angle,
  // theta1. This fact can be verified by calling this method with
  // arbitrary values of theta1, given that our benchmark does have this
  // property. This is done in the unit test below.
  void VerifyCalcMassMatrixViaInverseDynamics(
      double theta1, double theta2) {
    const double kTolerance = 10 * kEpsilon;
    pendulum_.shoulder().set_angle(context_.get(), theta1);
    pendulum_.elbow().set_angle(context_.get(), theta2);

    // MultibodyTree mass matrix:
    Matrix2d H;
    tree().CalcMassMatrixViaInverseDynamics(*context_, &H);

    // Benchmark mass matrix:
    Matrix2d H_expected = acrobot_benchmark_.CalcMassMatrix(theta2);

    EXPECT_TRUE(CompareMatrices(
        H, H_expected, kTolerance, MatrixCompareType::relative));
  }

  // For the double pendulum model under test, it verifies the result returned
  // by UniformGravityFieldElement::CalcGravityGeneralizedForces() which
  // computes the generalized forces tau_g due to a uniform gravity field force
  // element. The results are verified against our Acrobot benchmark class which
  // implements a handwritten computation of these forces.
  void VerifyGravityGeneralizedForces(double theta1, double theta2) const {
    const double kTolerance = 10 * kEpsilon;
    pendulum_.shoulder().set_angle(context_.get(), theta1);
    pendulum_.elbow().set_angle(context_.get(), theta2);

    const UniformGravityFieldElement<double>& gravity_element =
        pendulum_.gravity_element();

    Vector2d tau_g =
        gravity_element.CalcGravityGeneralizedForces(*context_);

    Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);

    EXPECT_TRUE(CompareMatrices(
        tau_g, tau_g_expected, kTolerance, MatrixCompareType::relative));
  }

  // This test verifies the API to set joint forces and compute inverse dynamics
  // when forces are applied.
  // For this double pendulum model we refer to the shoulder joint as joint 1
  // and to the elbow joint as joint 2. In this case, the state is fully
  // described by the joints' angles `theta1` and `theta2` and the joints' angle
  // rates `theta1dot` and `theta2dot`. Input torques `tau1` and `tau2` are
  // applied to the shoulder and elbow joints respectively.
  void VerifyInverseDynamicsWithAppliedForces(
      double theta1, double theta2,
      double theta1dot, double theta2dot,
      double tau1, double tau2) {
    const double kTolerance = 10 * kEpsilon;

    // Set up workspace:
    const int nv = tree().num_velocities();
    // External forces:
    MultibodyForces<double> forces(pendulum_.tree());
    // Accelerations of the bodies:
    std::vector<SpatialAcceleration<double>> A_WB_array(
        tree().num_bodies());
    // Generalized accelerations:
    VectorX<double> vdot = VectorX<double>::Zero(nv);

    // Useful aliases:
    std::vector<SpatialForce<double>>& F_BBo_W_array =
        forces.mutable_body_forces();
    VectorX<double>& tau_array = forces.mutable_generalized_forces();

    // Set angles:
    pendulum_.shoulder().set_angle(context_.get(), theta1);
    pendulum_.elbow().set_angle(context_.get(), theta2);

    // Set angular rates:
    pendulum_.shoulder().set_angular_rate(context_.get(), theta1dot);
    pendulum_.elbow().set_angular_rate(context_.get(), theta2dot);

    internal::PositionKinematicsCache<double> pc(tree().get_topology());
    internal::VelocityKinematicsCache<double> vc(tree().get_topology());
    tree().CalcPositionKinematicsCache(*context_, &pc);
    tree().CalcVelocityKinematicsCache(*context_, pc, &vc);

    // Compute forces applied through force elements. This initializes the
    // forces to zero and adds in contributions due to force elements:
    tree().CalcForceElementsContribution(*context_, pc, vc, &forces);

    // Apply external torques at the joints:
    pendulum_.shoulder().AddInTorque(*context_, tau1, &forces);
    pendulum_.elbow().AddInTorque(*context_, tau2, &forces);

    // Arrays for output forces:
    std::vector<SpatialForce<double>> F_BMo_W(tree().num_bodies());
    VectorX<double> tau(tree().num_velocities());

    // With vdot = 0, this computes:
    //   rhs = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
    tree().CalcInverseDynamics(
        *context_, vdot,
        F_BBo_W_array, tau_array,
        &A_WB_array,
        &F_BMo_W, &tau  /* Output forces */);

    // Now compute inverse dynamics using our benchmark:
    Vector2d tau_expected(tau1, tau2);
    Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    Vector2d rhs = C_expected - tau_g_expected - tau_expected;

    EXPECT_TRUE(CompareMatrices(
        tau, rhs, kTolerance, MatrixCompareType::relative));

    // Verify alternative APIs for inverse dynamics.
    const VectorX<double> tau_id2 =
        tree().CalcInverseDynamics(*context_, vdot, forces);
    EXPECT_TRUE(CompareMatrices(
        tau_id2, rhs, kTolerance, MatrixCompareType::relative));
  }

  const internal::MultibodyTree<double>& tree() const {
    return pendulum_.tree();
  }

 protected:
  // The MultibodyTree model under test.
  DoublePendulumModel<double> pendulum_;
  std::unique_ptr<Context<double>> context_;

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      pendulum_.mass1(), pendulum_.mass2(),
      pendulum_.length1(), pendulum_.length2(),
      pendulum_.half_length1(), pendulum_.half_length2(),
      pendulum_.Ic1(), pendulum_.Ic2(), 0.0, 0.0,
      pendulum_.gravity()};
};

// Compute the mass matrix using the inverse dynamics method.
TEST_F(PendulumTests, CalcMassMatrixViaInverseDynamics) {
  VerifyCalcMassMatrixViaInverseDynamics(0.0, 0.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 2.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 3.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 4.0);

  // We run the same previous tests with different shoulder angles to verify
  // that the mass matrix is a function of theta2 only.
  // This is verified implicitly since the results are compared against a
  // benchmark whose results depend on the elbow angle only.
  // See the documentation for this test's method
  // VerifyCalcMassMatrixViaInverseDynamics().
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, 0.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 2.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 3.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 4.0);

  VerifyCalcMassMatrixViaInverseDynamics(-M_PI / 7.0, 0.0);
  VerifyCalcMassMatrixViaInverseDynamics(-M_PI / 7.0, M_PI / 2.0);
  VerifyCalcMassMatrixViaInverseDynamics(-M_PI / 7.0, M_PI / 3.0);
  VerifyCalcMassMatrixViaInverseDynamics(-M_PI / 7.0, M_PI / 4.0);
}

// Verify the correct result from
// UniformGravityFieldElement::CalcGravityGeneralizedForces().
TEST_F(PendulumTests, VerifyGravityGeneralizedForces) {
  VerifyGravityGeneralizedForces(0.0, 0.0);
  VerifyGravityGeneralizedForces(0.0, M_PI / 2.0);
  VerifyGravityGeneralizedForces(0.0, M_PI / 3.0);
  VerifyGravityGeneralizedForces(0.0, M_PI / 4.0);

  VerifyGravityGeneralizedForces(M_PI / 3.0, 0.0);
  VerifyGravityGeneralizedForces(M_PI / 3.0, M_PI / 2.0);
  VerifyGravityGeneralizedForces(M_PI / 3.0, M_PI / 3.0);
  VerifyGravityGeneralizedForces(M_PI / 3.0, M_PI / 4.0);

  VerifyGravityGeneralizedForces(-M_PI / 7.0, 0.0);
  VerifyGravityGeneralizedForces(-M_PI / 7.0, M_PI / 2.0);
  VerifyGravityGeneralizedForces(-M_PI / 7.0, M_PI / 3.0);
  VerifyGravityGeneralizedForces(-M_PI / 7.0, M_PI / 4.0);
}

// Compute forward dynamics by explicitly forming the mass matrix.
TEST_F(PendulumTests, CalcForwardDynamicsViaExplicitMassMatrixSolve) {
  // With zero velocity and zero input torques.
  VerifyInverseDynamicsWithAppliedForces(
      0.0, 0.0,   /* joint's angles */
      0.0, 0.0,   /* joint's angular rates */
      0.0, 0.0);  /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      0.0, M_PI / 2.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      0.0, M_PI / 3.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      0.0, M_PI / 4.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */

  VerifyInverseDynamicsWithAppliedForces(
      M_PI / 3.0, 0.0,   /* joint's angles */
      0.0, 0.0,          /* joint's angular rates */
      0.0, 0.0);         /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      M_PI / 3.0, M_PI / 2.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      M_PI / 3.0, M_PI / 3.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      M_PI / 3.0, M_PI / 4.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */

  // With non-zero velocities and zero torques:
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.0, 0.0);                /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 3.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.0, 0.0);                /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      -1.5, 0.5,                /* joint's angular rates */
      0.0, 0.0);                /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 3.0,  /* joint's angles */
      -1.5, 0.5,                /* joint's angular rates */
      0.0, 0.0);                /* joint's torques */

  // With non-zero velocities and non-zero torques:
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.5, 1.0);                /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.5, -1.0);               /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      -0.5, 1.0);               /* joint's torques */
  VerifyInverseDynamicsWithAppliedForces(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      -0.5, -1.0);              /* joint's torques */
}

}  // namespace
}  // namespace multibody
}  // namespace drake
