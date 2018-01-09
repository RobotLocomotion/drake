// clang-format: off
#include "drake/multibody/multibody_tree/multibody_tree.h"
// clang-format: on

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_forcing.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using benchmarks::Acrobot;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using std::vector;
using systems::Context;

// Set of MultibodyTree tests for a double pendulum model. This simple set of
// tests serve as an example on how to build a model using RigidBody and Joint
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
//  - The body frames for each link are placed at their geometric center.
//  - The origin of the shoulder frames (Si and So) are coincident at all times.
//    So is aligned with Si for θ₁ = 0.
//  - The origin of the elbow frames (Ei and Eo) are coincident at all times.
//    Eo is aligned with Ei for θ₂ = 0.
//
//       y ^
//         | Si ≡ W World body frame.
//         +--> x  Shoulder inboard frame Si coincides with W.
//      X_SiSo(θ₁) Shoulder revolute mobilizer with generalized position θ₁.
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
//      X_EiEo(θ₂) Elbow revolute mobilizer with generalized position θ₂.
//      +--+-----+
//      |  ^     |
//      |  |Eo/L2| Elbow outboard frame Eo.
//      |  +-->  | Lower link's frame L2 is coincident with the elbow frame Eo.
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
    tree_ = std::make_unique<MultibodyTree<T>>();

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
    link1_ = &tree_->template AddBody<RigidBody>(M1_L1);
    link2_ = &tree_->template AddBody<RigidBody>(M2_L2);
    world_body_ = &tree_->get_world_body();

    // The shoulder joint connects the world with link 1.
    // Its inboard frame, Si, is the world frame.
    // Its outboard frame, So, is at a fixed offset frame link 2's frame L2.
    // L2's origin is located at the com of link 2.
    // Pose of the shoulder outboard frame So in link 1's frame:
    const Isometry3d X_L2So{Translation3d(0.0, half_length1_, 0.0)};

    shoulder_ = &tree_->template AddJoint<RevoluteJoint>(
        "ShoulderJoint",
        *world_body_, {},  /* No input, frame Si IS the world frame W. */
        *link1_, X_L2So,   /* Pose of So in link 2's frame L2. */
        Vector3d::UnitZ()  /* revolute axis */);

    // The elbow is the mobilizer that connects links 1 and 2.
    // An inboard frame Ei is rigidly attached to link 1. It is located
    // at y = -half_link_length_ in the frame of link 1.
    // X_L1Ei specifies the pose of the elbow inboard frame Ei in the body
    // frame L1 of link 1.
    // The elbow's outboard frame Eo is taking to be coincident with link 2's
    // frame L2 (i.e. L2o != L2cm).
    // Pose of the elbow inboard frame Ei link 1's frame L1:
    const Isometry3d X_L1Ei{Translation3d(0.0, -half_length1_, 0.0)};

    elbow_ = &tree_->template AddJoint<RevoluteJoint>(
        "ElbowJoint",
        *link1_, X_L1Ei,  /* Pose of Ei in L1. */
        *link2_, {},      /* No pose provided, frame Eo IS frame L2. */
        Vector3d::UnitZ() /* revolute axis */);

    // Add force element for a constant gravity pointing downwards, that is, in
    // the minus y-axis direction.
    tree_->template AddForceElement<UniformGravityFieldElement>(
        Vector3d(0.0, -acceleration_of_gravity_, 0.0));

    // We are done adding modeling elements.
    tree_->Finalize();
  }

  const MultibodyTree<T>& get_tree() const {
    return *tree_;
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

 private:
  std::unique_ptr<MultibodyTree<T>> tree_;
  const Body<T>* world_body_;
  // Bodies:
  const RigidBody<T>* link1_;
  const RigidBody<T>* link2_;
  // Joints:
  const RevoluteJoint<T>* shoulder_;
  const RevoluteJoint<T>* elbow_;

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

class PendulumTests : public ::testing::Test {
 public:
  // Creates a DoublePendulumModel class with an underlying MultibodyTree model
  // of a double pendulum.
  void SetUp() override {
    tree_ = &model_.get_tree();
    context_ = tree_->CreateDefaultContext();
  }

  // Helper method to extract a pose from the position kinematics.
  // TODO(amcastro-tri):
  // Replace this by a method Body<T>::get_pose_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  template<typename T>
  const Isometry3<T> &get_body_pose_in_world(
      const PositionKinematicsCache<T> &pc,
      const Body<T> &body) const {
    const MultibodyTreeTopology &topology = tree_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return pc.get_X_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract spatial velocity from the velocity kinematics
  // cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_velocity_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  const SpatialVelocity<double> &get_body_spatial_velocity_in_world(
      const VelocityKinematicsCache<double> &vc,
      const Body<double> &body) const {
    const MultibodyTreeTopology &topology = tree_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return vc.get_V_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract spatial acceleration from the acceleration
  // kinematics cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_acceleration_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  const SpatialAcceleration<double> &get_body_spatial_acceleration_in_world(
      const AccelerationKinematicsCache<double> &ac,
      const Body<double> &body) const {
    const MultibodyTreeTopology &topology = tree_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return ac.get_A_WB(topology.get_body(body.get_index()).body_node);
  }

  void VerifyCalcMassMatrixViaInverseDynamics(
      double theta1, double theta2) {
    const double kTolerance = 10 * kEpsilon;
    model_.shoulder().set_angle(context_.get(), theta1);
    model_.elbow().set_angle(context_.get(), theta2);

    // MultibodyTree mass matrix:
    Matrix2d H;
    tree_->CalcMassMatrixViaInverseDynamics(*context_, &H);
    
    // Benchmark mass matrix:
    Matrix2d H_expected = acrobot_benchmark_.CalcMassMatrix(theta2);
    
    CompareMatrices(H, H_expected, kTolerance, MatrixCompareType::relative);
  }

  void VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      double theta1, double theta2,
      double theta1dot, double theta2dot,
      double tau1, double tau2) {
    const double kTolerance = 10 * kEpsilon;

    // Set angles:
    model_.shoulder().set_angle(context_.get(), theta1);
    model_.elbow().set_angle(context_.get(), theta2);

    // Set angular rates:
    model_.shoulder().set_angular_rate(context_.get(), theta1dot);
    model_.elbow().set_angular_rate(context_.get(), theta2dot);

    // Apply external torques at the joints:
    MultibodyTreeForcing<double> forcing(model_.get_tree());
    model_.shoulder().AddInTorque(*context_, tau1, &forcing);
    model_.shoulder().AddInTorque(*context_, tau2, &forcing);

    // Vector of generalized velocites time derivatives:
    VectorX<double> vdot(tree_->get_num_velocities());

    tree_->CalcForwardDynamicsViaExplicitMassMatrixSolve(
        *context_, forcing, &vdot);

    // Now compute forward dynamics using our benchmark:
    Vector2d tau(tau1, tau2);
    Vector2d C = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    Vector2d tau_g = acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    Matrix2d M = acrobot_benchmark_.CalcMassMatrix(theta2);

    Vector2d rhs = tau + tau_g - C;
    Vector2d vdot_benchmark = M.llt().solve(rhs);

    CompareMatrices(
        vdot, vdot_benchmark, kTolerance, MatrixCompareType::relative);
  }

 protected:
  // The MultibodyTree model under test.
  DoublePendulumModel<double> model_;

  // Pointer to the underlying model. model_ owns it.
  const MultibodyTree<double>* tree_{nullptr};

  std::unique_ptr<Context<double>> context_;

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      model_.mass1(), model_.mass2(),
      model_.length1(), model_.length2(),
      model_.half_length1(), model_.half_length2(),
      model_.Ic1(), model_.Ic2(), 0.0, 0.0,
      model_.gravity()};
};

// Compute the mass matrix using the inverse dynamics method.
TEST_F(PendulumTests, CalcMassMatrixViaInverseDynamics) {
  VerifyCalcMassMatrixViaInverseDynamics(0.0, 0.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 2.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 3.0);
  VerifyCalcMassMatrixViaInverseDynamics(0.0, M_PI / 4.0);

  // For the double pendulum system it turns out that the mass matrix is only a
  // function of the elbow angle, theta2, independent of the shoulder angle,
  // theta1.
  // Therefore M(q) = H(elbow_angle). We therefore run the same previous tests
  // with different shoulder angles to verify this is true.
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, 0.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 2.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 3.0);
  VerifyCalcMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 4.0);
}

// Compute forward dynamics by explicitly forming the mass matrix.
TEST_F(PendulumTests, CalcForwardDynamicsViaExplicitMassMatrixSolve) {
  // With zero velocity and zero input torques.
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      0.0, 0.0,   /* joint's angles */
      0.0, 0.0,   /* joint's angular rates */
      0.0, 0.0);  /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      0.0, M_PI / 2.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      0.0, M_PI / 3.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      0.0, M_PI / 4.0,  /* joint's angles */
      0.0, 0.0,         /* joint's angular rates */
      0.0, 0.0);        /* joint's torques */

  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      M_PI / 3.0, 0.0,   /* joint's angles */
      0.0, 0.0,          /* joint's angular rates */
      0.0, 0.0);         /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      M_PI / 3.0, M_PI / 2.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      M_PI / 3.0, M_PI / 3.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      M_PI / 3.0, M_PI / 4.0,  /* joint's angles */
      0.0, 0.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */

  // With non-zero velocities and zero torques:
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
     -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
     -M_PI / 5.0, M_PI / 3.0,  /* joint's angles */
      0.5, 1.0,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
     -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
     -1.5, 0.5,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
     -M_PI / 5.0, M_PI / 3.0,  /* joint's angles */
     -1.5, 0.5,                /* joint's angular rates */
      0.0, 0.0);               /* joint's torques */

  // With non-zero velocities and non-zero torques:
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.5, 1.0);                /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      0.5, -1.0);               /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
     -0.5, 1.0);                /* joint's torques */
  VerifyCalcForwardDynamicsViaExplicitMassMatrixSolve(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
     -0.5, -1.0);               /* joint's torques */
}

}  // namespace
}  // namespace multibody
}  // namespace drake
