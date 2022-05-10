#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake::multibody::multibody_plant {

using drake::multibody::Parser;
using Eigen::VectorXd;
using systems::Context;

namespace {

constexpr auto pi = 3.14159265358979323846;

class InverseDynamicsRTests : public ::testing::Test {
 public:
  MultibodyPlant<double> plant{0.0};

  void load_model(const std::string& file_path) {
    const std::string model_path = FindResourceOrThrow(file_path);
    Parser parser(&plant);
    parser.AddModelFromFile(model_path);
    plant.Finalize();
  }

  void test_inverse_dynamics(const double angle) {
    constexpr auto length = 1.0;
    constexpr auto mass = 2.0;
    constexpr auto gravity = 9.81;

    auto plant_context = plant.CreateDefaultContext();
    auto multibody_forces = MultibodyForces<double>(plant);

    // add the gravity effect as a generalized force
    const auto gravity_forces =
        plant.CalcGravityGeneralizedForces(*plant_context);
    multibody_forces.mutable_generalized_forces() += gravity_forces;

    // set positions
    auto positions = VectorXd(1);
    positions << angle;
    plant.SetPositions(plant_context.get(), positions);

    // set velocities and accelerations to 0
    VectorXd velocities = VectorXd::Zero(1);
    plant.SetVelocities(plant_context.get(), velocities);

    VectorXd known_vdot = VectorXd::Zero(1);

    // calculate the joint torque
    const auto tau =
        plant.CalcInverseDynamics(*plant_context, known_vdot, multibody_forces);

    // compare with the expected torque
    VectorXd expected_tau = VectorXd(1);
    expected_tau << mass * gravity * length / 2 * std::cos(angle);

    EXPECT_TRUE(
        CompareMatrices(expected_tau, tau, 0.01, MatrixCompareType::relative));
  }
};

TEST_F(InverseDynamicsRTests, InverseDynamicsR) {
  load_model("drake/multibody/plant/test/inverse_dynamics_R.urdf");

  test_inverse_dynamics(pi / 4);
  test_inverse_dynamics(0.0);
  test_inverse_dynamics(pi / 2);
  test_inverse_dynamics(pi);
}

}  // namespace
}  // namespace drake::multibody::multibody_plant
