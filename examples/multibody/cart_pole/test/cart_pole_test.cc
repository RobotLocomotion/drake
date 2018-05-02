#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {

using multibody::Body;
using multibody::multibody_plant::MultibodyPlant;
using multibody::UniformGravityFieldElement;
using multibody::parsing::AddModelFromSdfFile;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using systems::Context;

namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

Matrix2<double> CartPoleMassMatrix(double theta) {
  // Fixed parameters.
  const double mc = 10;   // mass of the cart in kg.
  const double mp = 1;    // mass of the pole (point mass at the end) in kg.
  const double l = 0.5;   // length of the pole in m
  const double c = cos(theta);
  Matrix2<double> M;
  M << mc+mp, mp*l*c,
       mp*l*c, mp*l*l;
  return M;
}

class CartPoleTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Make the cart_pole model.
    const std::string full_name = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf");
    AddModelFromSdfFile(full_name, &cart_pole_);

    // Add gravity to the model.
    cart_pole_.AddForceElement<UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());

    // Now the model is complete.
    cart_pole_.Finalize();

    // Get joints so that we can set the state.
    cart_slider_ = &cart_pole_.GetJointByName<PrismaticJoint>("CartSlider");
    pole_pin_ = &cart_pole_.GetJointByName<RevoluteJoint>("PolePin");

    // Create a context to store the state for this model:
    context_ = cart_pole_.CreateDefaultContext();
  }

 protected:
  MultibodyPlant<double> cart_pole_;
  //const RigidBody<double>* body1_{nullptr};
  const PrismaticJoint<double>* cart_slider_{nullptr};
  const RevoluteJoint<double>* pole_pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// Verify the expected number of dofs.
TEST_F(CartPoleTest, MassMatrix) {
  const double theta = M_PI / 3;

  Matrix2<double> M;
  pole_pin_->set_angle(context_.get(), theta);
  cart_pole_.model().CalcMassMatrixViaInverseDynamics(*context_, &M);
  Matrix2<double> M_expected = CartPoleMassMatrix(theta);

  // Matrix verified to this tolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M, M_expected,
                  kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake
