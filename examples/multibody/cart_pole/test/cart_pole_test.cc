#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/multibody/cart_pole/gen/cart_pole_params.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using drake::multibody::Body;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
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
    AddModelFromSdfFile(full_name, &cart_pole_);

    // Add gravity to the model.
    cart_pole_.AddForceElement<UniformGravityFieldElement>(
        -default_parameters_.gravity() * Vector3<double>::UnitZ());

    // Now the model is complete.
    cart_pole_.Finalize();

    // Get joints so that we can set the state.
    cart_slider_ = &cart_pole_.GetJointByName<PrismaticJoint>("CartSlider");
    pole_pin_ = &cart_pole_.GetJointByName<RevoluteJoint>("PolePin");

    // Create a context to store the state for this model:
    context_ = cart_pole_.CreateDefaultContext();
  }

  // Makes the mass matrix for the cart-pole system.
  // See http://underactuated.csail.mit.edu/underactuated.html?chapter=acrobot.
  Matrix2<double> CartPoleHandWritenMassMatrix(double theta) {
    const double mc = default_parameters_.mc();  // mass of the cart in kg.
    const double mp = default_parameters_.mp();  // Pole's point mass in kg.
    const double l = default_parameters_.l();    // length of the pole in m
    const double c = cos(theta);
    Matrix2<double> M;
    M << mc+mp, mp*l*c,
         mp*l*c, mp*l*l;
    return M;
  }

  Vector4<double> CartPoleHandWritenDynamics(
      const Vector2<double>& q, const Vector2<double>& v) {
    const double mp = default_parameters_.mp();  // Pole's point mass in kg.
    const double l = default_parameters_.l();    // length of the pole in m
    const double g = default_parameters_.gravity();  // Gravity in m/s^2.

    // Mass matrix.
    const double theta = q(1);
    const Matrix2<double> M = CartPoleHandWritenMassMatrix(theta);

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
  MultibodyPlant<double> cart_pole_;
  const PrismaticJoint<double>* cart_slider_{nullptr};
  const RevoluteJoint<double>* pole_pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
  // Default parameters generated from cart_pole_params.named_vector.
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
  cart_pole_.model().CalcMassMatrixViaInverseDynamics(*context_, &M);
  Matrix2<double> M_expected = CartPoleHandWritenMassMatrix(theta);

  // Matrix verified to this tolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M, M_expected,
                  kTolerance, MatrixCompareType::relative));
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

  const Vector4<double> xc_dot_expected = CartPoleHandWritenDynamics(q, v);

  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(xc_dot->CopyToVector(),
                              xc_dot_expected, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake
