#include "drake/systems/sensors/gyroscope.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace {

using systems::BasicVector;
using systems::sensors::Gyroscope;

class GyroscopeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Plant/System initialization.
    systems::DiagramBuilder<double> builder;
    plant_ = builder.AddSystem<multibody::MultibodyPlant>(0.0);
    const std::string urdf_name =
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
    multibody::Parser parser(plant_);
    parser.AddModelFromFile(urdf_name);
    plant_->Finalize();

    const multibody::Body<double>& arm_body = plant_->GetBodyByName("arm");
    const math::RotationMatrix<double> R_BS(
        drake::math::RollPitchYaw<double>(M_PI / 2, 0, 0));
    const math::RigidTransform<double> X_BS(R_BS, Eigen::Vector3d::Zero());

    gyroscope_ =
        &Gyroscope<double>::AddToDiagram(arm_body, X_BS, *plant_, &builder);

    diagram_ = builder.Build();
  }

  const Gyroscope<double>* gyroscope_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
};

TEST_F(GyroscopeTest, Rotated) {
  double tol = 10 * std::numeric_limits<double>::epsilon();

  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());
  auto& gyro_context =
      diagram_->GetMutableSubsystemContext(*gyroscope_, diagram_context.get());

  double theta = M_PI / 2;
  double omega = .5;

  // Test zero-velocity state.
  plant_->get_actuation_input_port().FixValue(&plant_context, Vector1d(0));
  plant_->SetPositions(&plant_context, Vector1d(theta));
  plant_->SetVelocities(&plant_context, Vector1d(omega));

  const auto& result =
      gyroscope_->get_measurement_output_port().Eval<BasicVector<double>>(
          gyro_context);

  // Compute expected result.
  // Angular velocity in world coordinates is (0, omega, 0).
  // In body coordinates, it is the same, (0, omega, 0).
  // The sensor frame is rotated by pi/2 about the x-axis, so
  // the expected measurement is (0, 0, -omega).
  Eigen::Vector3d expected_result(0, 0, -omega);

  EXPECT_TRUE(CompareMatrices(result.get_value(), expected_result, tol));
}

TEST_F(GyroscopeTest, ScalarConversionTest) {
  EXPECT_TRUE(is_autodiffxd_convertible(*gyroscope_));
  EXPECT_TRUE(is_symbolic_convertible(*gyroscope_));
}

}  // namespace
}  // namespace drake
