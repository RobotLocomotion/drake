#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/examples/multibody/cart_pole/gen/cart_pole_params.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using drake::multibody::Body;
using drake::multibody::JointActuator;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::Context;

class CartPoleTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Make the cart_pole model.
    const std::string full_name = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf");
    Parser(&cart_pole_).AddModelFromFile(full_name);

    // Add gravity to the model.
    cart_pole_.mutable_gravity_field().set_gravity_vector(
        -default_parameters_.gravity() * Vector3<double>::UnitZ());

    // Now the model is complete.
    cart_pole_.Finalize();

    ASSERT_EQ(cart_pole_.num_bodies(), 3);  // Includes the world body.
    ASSERT_EQ(cart_pole_.num_joints(), 2);

    // Get joints so that we can set the state.
    cart_slider_ = &cart_pole_.GetJointByName<PrismaticJoint>("CartSlider");
    pole_pin_ = &cart_pole_.GetJointByName<RevoluteJoint>("PolePin");

    // Verify there is a single actuator for the slider joint.
    ASSERT_EQ(cart_pole_.num_actuators(), 1);
    const JointActuator<double>& actuator =
        cart_pole_.GetJointActuatorByName("CartSlider");
    ASSERT_EQ(actuator.joint().index(), cart_slider_->index());

    // Create a context to store the state for this model:
    context_ = cart_pole_.CreateDefaultContext();
    cart_pole_.get_actuation_input_port().FixValue(context_.get(), 0.);
  }

  // Makes the mass matrix for the cart-pole system.
  // See http://underactuated.csail.mit.edu/underactuated.html?chapter=acrobot.
  Matrix2<double> CartPoleHandWrittenMassMatrix(double theta) {
    const double mc = default_parameters_.mc();  // mass of the cart in kg.
    const double mp = default_parameters_.mp();  // Pole's point mass in kg.
    const double l = default_parameters_.l();    // length of the pole in m
    const double c = cos(theta);
    Matrix2<double> M;
    M << mc+mp, mp*l*c,
         mp*l*c, mp*l*l;
    return M;
  }

  Vector4<double> CartPoleHandWrittenDynamics(
      const Vector2<double>& q, const Vector2<double>& v) {
    const double mp = default_parameters_.mp();  // Pole's point mass in kg.
    const double l = default_parameters_.l();    // length of the pole in m
    const double g = default_parameters_.gravity();  // Gravity in m/s^2.

    // Mass matrix.
    const double theta = q(1);
    const Matrix2<double> M = CartPoleHandWrittenMassMatrix(theta);

    // Coriolis and gyroscopic terms.
    const double theta_dot = v(1);
    const double s = sin(theta);
    Matrix2<double> C = Matrix2<double>::Zero();
    C(0, 1) = -mp * l * theta_dot * s;

    // Vector of genralized forces due to gravity.
    Vector2<double> tau_g = Vector2<double>(0.0, -mp * g * l * s);

    // Compute the dynamics of the system.
    Vector4<double> state_dot;
    state_dot << v , M.llt().solve(tau_g - C * v);
    return state_dot;
  }

 protected:
  MultibodyPlant<double> cart_pole_{0.0};
  const PrismaticJoint<double>* cart_slider_{nullptr};
  const RevoluteJoint<double>* pole_pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
  // Default parameters generated from cart_pole_params_named_vector.yaml.
  const CartPoleParams<double> default_parameters_;
};

// Tests that the hand-derived mass matrix matches the mass matrix computed with
// a MultibodyPlant built from an SDF file.
TEST_F(CartPoleTest, MassMatrix) {
  // The mass matrix of the system does not depend on the x location of the
  // cart. Therefore we only need the angle of the pole.
  const double theta = M_PI / 3;

  Matrix2<double> M;
  pole_pin_->set_angle(context_.get(), theta);
  cart_pole_.CalcMassMatrixViaInverseDynamics(*context_, &M);
  Matrix2<double> M_expected = CartPoleHandWrittenMassMatrix(theta);

  // Matrix verified to this tolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M, M_expected,
                  kTolerance, MatrixCompareType::relative));

  {  // Repeat the computation to confirm the heap behavior.  We allow the
     // method to heap-allocate 4 temporaries.
    drake::test::LimitMalloc guard({.max_num_allocations = 4});
    cart_pole_.CalcMassMatrixViaInverseDynamics(*context_, &M);
  }
}

// Tests that the hand-derived dynamics matches that computed with a
// MultibodyPlant built from an SDF file.
TEST_F(CartPoleTest, SystemDynamics) {
  // State arbitrarily chosen for this test.
  const Vector2<double> q(2.5, M_PI / 3);
  const Vector2<double> v(-1.5, 0.5);

  Matrix2<double> M;
  cart_slider_->set_translation(context_.get(), q(0));
  cart_slider_->set_translation_rate(context_.get(), v(0));
  pole_pin_->set_angle(context_.get(), q(1));
  pole_pin_->set_angular_rate(context_.get(), v(1));
  std::unique_ptr<systems::ContinuousState<double>> xc_dot =
      cart_pole_.AllocateTimeDerivatives();
  cart_pole_.CalcTimeDerivatives(*context_, xc_dot.get());

  const Vector4<double> xc_dot_expected = CartPoleHandWrittenDynamics(q, v);

  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(xc_dot->CopyToVector(),
                              xc_dot_expected, kTolerance,
                              MatrixCompareType::relative));

    // Verify that the implicit dynamics match the continuous ones.
  Eigen::VectorXd residual =
      cart_pole_.AllocateImplicitTimeDerivativesResidual();
  cart_pole_.CalcImplicitTimeDerivativesResidual(*context_, *xc_dot, &residual);
  EXPECT_TRUE(CompareMatrices(residual, Eigen::VectorXd::Zero(4), 1e-15));
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake
