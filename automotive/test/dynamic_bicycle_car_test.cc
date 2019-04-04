#include "drake/automotive/dynamic_bicycle_car.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

// Struct to hold the test values for the different tests.
struct TestValues {
  // Initial state values.
  double v_LCp_x = 0;
  double yaw_LC = 0;

  // Inputs initialized to zero.
  double steer_angle = 0;
  double f_Cp_x = 0;  // Longitudinal force

  // The set of TestValues.expected_<parameter> values are calculated using the
  // equations for normal load, tire slip angle, and lateral tire force based on
  // the dynamic bicycle model from Bobier (2012).

  // Expected values initialized to zero.
  double expected_tire_slip_angle_front = 0;
  double expected_tire_slip_angle_rear = 0;
  double expected_normal_load_front = 0;
  double expected_normal_load_rear = 0;
  double expected_lateral_force_front = 0;
  double expected_lateral_force_rear = 0;

  // Expected derivative values initialized to zero.
  double expected_v_LCp_x = 0;
  double expected_v_LCp_y = 0;
  double expected_yawDt_LC = 0;
  double expected_vDt_LCp_x = 0;
  double expected_vDt_LCp_y = 0;
  double expected_yawDDt_LC = 0;

  // Tolerance used for testing the expected values.
  const double tolerance = 1e-5;
};

// Test values for going in a straight line with constant velocity.
TestValues GetTestValuesStraightConstV() {
  TestValues test_values;
  DynamicBicycleCarParams<double> car_params;

  // Set initial values for the state.
  test_values.v_LCp_x = 10;
  test_values.yaw_LC = 0;

  // Test inputs for constant velocity with zero steer input.
  test_values.steer_angle = 0;
  test_values.f_Cp_x = 0;

  // Expected values.
  test_values.expected_tire_slip_angle_front = 0;
  test_values.expected_tire_slip_angle_rear = 0;
  test_values.expected_normal_load_front = 7868.7972;
  test_values.expected_normal_load_rear = 10014.8328;
  test_values.expected_lateral_force_front = 0;
  test_values.expected_lateral_force_rear = 0;

  // Expected derivative values.
  test_values.expected_v_LCp_x = 10;
  test_values.expected_v_LCp_y = 0;
  test_values.expected_yawDt_LC = 0;
  test_values.expected_vDt_LCp_x = test_values.f_Cp_x / car_params.mass();
  test_values.expected_vDt_LCp_y = 0;
  test_values.expected_yawDDt_LC = 0;

  return test_values;
}

// Test values for going around a curve with a positive steer angle.
TestValues GetParamsCurve() {
  TestValues test_values;
  DynamicBicycleCarParams<double> car_params;

  // Set initial values for the state.
  test_values.v_LCp_x = 10;
  test_values.yaw_LC = M_PI / 12;

  // Test inputs
  test_values.steer_angle = M_PI / 6;
  test_values.f_Cp_x = 0;

  // Expected values.
  test_values.expected_tire_slip_angle_front = -0.5236;
  test_values.expected_tire_slip_angle_rear = 0;
  test_values.expected_normal_load_front = 7868.7972;
  test_values.expected_normal_load_rear = 10014.8328;
  test_values.expected_lateral_force_front = 4327.83846;
  test_values.expected_lateral_force_rear = 0;

  // Expected derivative values.
  test_values.expected_v_LCp_x = 10;
  test_values.expected_v_LCp_y = 0;
  test_values.expected_yawDt_LC = 0;
  test_values.expected_vDt_LCp_x = test_values.f_Cp_x / car_params.mass();
  test_values.expected_vDt_LCp_y = 2.05596;
  test_values.expected_yawDDt_LC = 2.88597;

  return test_values;
}

// Test values for going around a curve with a negative steer angle.
TestValues GetParamsCurveNegative() {
  TestValues test_values;
  DynamicBicycleCarParams<double> car_params;

  // Set initial values for the state.
  test_values.v_LCp_x = 10;
  test_values.yaw_LC = -M_PI / 12;

  // Test inputs
  test_values.steer_angle = -M_PI / 6;
  test_values.f_Cp_x = 0;

  // Expected values.
  test_values.expected_tire_slip_angle_front = 0.5236;
  test_values.expected_tire_slip_angle_rear = 0;
  test_values.expected_normal_load_front = 7868.7972;
  test_values.expected_normal_load_rear = 10014.8328;
  test_values.expected_lateral_force_front = -4327.83846;
  test_values.expected_lateral_force_rear = 0;

  // Expected derivative values initialized to zero.
  test_values.expected_v_LCp_x = 10;
  test_values.expected_v_LCp_y = 0;
  test_values.expected_yawDt_LC = 0;
  test_values.expected_vDt_LCp_x = test_values.f_Cp_x / car_params.mass();
  test_values.expected_vDt_LCp_y = -2.05596;
  test_values.expected_yawDDt_LC = -2.88597;

  return test_values;
}

// Test fixture to help test the DynamicBicycle model.
class DynamicBicycleCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_car_ = std::make_unique<DynamicBicycleCar<double>>();
    test_car_params_ = std::make_unique<DynamicBicycleCarParams<double>>();
    context_ = test_car_->CreateDefaultContext();
    output_ = test_car_->AllocateOutput();
    derivatives_ = test_car_->AllocateTimeDerivatives();
    system_output_ = test_car_->AllocateOutput();
  }

  // Returns a pointer to the state vector.
  DynamicBicycleCarState<double>* continuous_state() {
    DynamicBicycleCarState<double>& state =
        test_car_->get_mutable_state(context_.get());
    return &state;
  }

  // Returns a pointer to the state derivatives vector.
  const DynamicBicycleCarState<double>* state_derivatives() const {
    const auto state_derivatives =
        dynamic_cast<const DynamicBicycleCarState<double>*>(
            &derivatives_->get_mutable_vector());
    return state_derivatives;
  }

  // Tests the slip angle of the front and rear tires based on the expected
  // values contained in the input struct.
  void TestTireSlipAngle(TestValues test_values) {
    const double tire_slip_angle_front = test_car_->CalcTireSlip(
        *continuous_state(), *test_car_params_, test_values.steer_angle,
        DynamicBicycleCar<double>::Tire::kFrontTire);
    const double tire_slip_angle_rear = test_car_->CalcTireSlip(
        *continuous_state(), *test_car_params_, test_values.steer_angle,
        DynamicBicycleCar<double>::Tire::kRearTire);

    EXPECT_NEAR(tire_slip_angle_front,
                test_values.expected_tire_slip_angle_front,
                test_values.tolerance);
    EXPECT_NEAR(tire_slip_angle_rear, test_values.expected_tire_slip_angle_rear,
                test_values.tolerance);
  }

  // Tests the normal forces on the front and rear tires based on the expected
  // values contained in the input struct.
  void TestNormalLoad(TestValues test_values) {
    const double normal_load_front = test_car_->CalcNormalTireForce(
        *test_car_params_, test_values.f_Cp_x,
        DynamicBicycleCar<double>::Tire::kFrontTire);
    const double normal_load_rear = test_car_->CalcNormalTireForce(
        *test_car_params_, test_values.f_Cp_x,
        DynamicBicycleCar<double>::Tire::kRearTire);

    EXPECT_NEAR(normal_load_front, test_values.expected_normal_load_front,
                test_values.tolerance);
    EXPECT_NEAR(normal_load_rear, test_values.expected_normal_load_rear,
                test_values.tolerance);
  }

  // Tests the lateral tire forces on the front and rear tires based on the
  // expected values contained in the input struct.
  void TestLateralTireForce(TestValues test_values) {
    // Compute the slip angles of the tires to be used in the lateral force
    // calculation.
    const double tire_slip_angle_front = test_car_->CalcTireSlip(
        *continuous_state(), *test_car_params_, test_values.steer_angle,
        DynamicBicycleCar<double>::Tire::kFrontTire);
    const double tire_slip_angle_rear = test_car_->CalcTireSlip(
        *continuous_state(), *test_car_params_, test_values.steer_angle,
        DynamicBicycleCar<double>::Tire::kRearTire);

    // Compute the normal forces on the tires to be used in the lateral force
    // calculation.
    const double normal_load_front = test_car_->CalcNormalTireForce(
        *test_car_params_, test_values.f_Cp_x,
        DynamicBicycleCar<double>::Tire::kFrontTire);
    const double normal_load_rear = test_car_->CalcNormalTireForce(
        *test_car_params_, test_values.f_Cp_x,
        DynamicBicycleCar<double>::Tire::kRearTire);

    // Compute the lateral forces on the tires and compare against expected
    // values.
    const double lateral_force_front = test_car_->CalcLateralTireForce(
        tire_slip_angle_front, test_car_params_->c_alpha_f(), normal_load_front,
        test_car_params_->mu());
    const double lateral_force_rear = test_car_->CalcLateralTireForce(
        tire_slip_angle_rear, test_car_params_->c_alpha_r(), normal_load_rear,
        test_car_params_->mu());

    EXPECT_NEAR(lateral_force_front, test_values.expected_lateral_force_front,
                test_values.tolerance);
    EXPECT_NEAR(lateral_force_rear, test_values.expected_lateral_force_rear,
                test_values.tolerance);
  }

  // Test the computation of the state derivatives based on the expected values
  // contained in the input struct.
  void TestCalcDerivatives(TestValues test_values) {
    test_car_->CalcTimeDerivatives(*context_, derivatives_.get());

    EXPECT_NEAR(state_derivatives()->p_LoCp_x(), test_values.expected_v_LCp_x,
                test_values.tolerance);
    EXPECT_NEAR(state_derivatives()->p_LoCp_y(), test_values.expected_v_LCp_y,
                test_values.tolerance);
    EXPECT_NEAR(state_derivatives()->yaw_LC(), test_values.expected_yawDt_LC,
                test_values.tolerance);
    EXPECT_NEAR(state_derivatives()->v_LCp_x(), test_values.expected_vDt_LCp_x,
                test_values.tolerance);
    EXPECT_NEAR(state_derivatives()->v_LCp_y(), test_values.expected_vDt_LCp_y,
                test_values.tolerance);
    EXPECT_NEAR(state_derivatives()->yawDt_LC(), test_values.expected_yawDDt_LC,
                test_values.tolerance);
  }

  std::unique_ptr<DynamicBicycleCar<double>>
      test_car_;  // This is the model we are testing.
  std::unique_ptr<DynamicBicycleCarParams<double>> test_car_params_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::SystemOutput<double>> system_output_;
};

TEST_F(DynamicBicycleCarTest, Construction) {
  // Test that the system has one input port and one output port.
  EXPECT_EQ(1, test_car_->num_input_ports());
  EXPECT_EQ(1, test_car_->num_output_ports());

  // Test if the input and output ports are vectors of Eigen scalars.
  EXPECT_EQ(systems::kVectorValued,
            test_car_->get_input_port().get_data_type());
  EXPECT_EQ(systems::kVectorValued,
            test_car_->get_output_port().get_data_type());
}

// Tests whether DynamicBicycleCar of type DynamicBicycleCar<double> can be
// converted to use AutoDiffXd as its scalar type.
TEST_F(DynamicBicycleCarTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*test_car_));
}

// Tests whether DynamicBicycleCar of type DynamicBicycleCar<double> can be
// converted to use symbolic::Expression as its scalar type.
TEST_F(DynamicBicycleCarTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*test_car_));
}

// Tests to make sure the inputs don't directly pass to the outputs.
TEST_F(DynamicBicycleCarTest, DirectFeedthrough) {
  EXPECT_FALSE(test_car_->HasAnyDirectFeedthrough());
}

// Test that the state is passed through to the output.
TEST_F(DynamicBicycleCarTest, Output) {
  // Set the system to have an arbitrary state.
  Vector6<double> test_state;
  test_state << 1.0, 3.0, 5.0, 2.0, 4.0, 6.0;
  continuous_state()->SetFromVector(test_state);

  test_car_->CalcOutput(*context_, system_output_.get());

  const DynamicBicycleCarState<double>* bike_out =
      dynamic_cast<const DynamicBicycleCarState<double>*>(
          system_output_->get_vector_data(0));

  EXPECT_NE(nullptr, bike_out);

  EXPECT_EQ(1.0, bike_out->p_LoCp_x());
  EXPECT_EQ(3.0, bike_out->p_LoCp_y());
  EXPECT_EQ(5.0, bike_out->yaw_LC());
  EXPECT_EQ(2.0, bike_out->v_LCp_x());
  EXPECT_EQ(4.0, bike_out->v_LCp_y());
  EXPECT_EQ(6.0, bike_out->yawDt_LC());
}

// Test with constant velocity in a straight line.
TEST_F(DynamicBicycleCarTest, StraightLineTest) {
  TestValues test_values = GetTestValuesStraightConstV();

  // Set the system to have an initial state with longitudinal velocity of
  // 10 m/s.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_v_LCp_x(test_values.v_LCp_x);
  continuous_state()->set_yaw_LC(test_values.yaw_LC);

  // Sets the input port to be pi/6 radians (~30 degrees) steering angle and
  // zero force input.
  context_->FixInputPort(
      0, Vector2<double>{test_values.steer_angle, test_values.f_Cp_x});

  TestTireSlipAngle(test_values);
  TestNormalLoad(test_values);
  TestLateralTireForce(test_values);
  TestCalcDerivatives(test_values);
}

// Test on a curve with positive steering angle.
TEST_F(DynamicBicycleCarTest, CurveTest) {
  TestValues test_values = GetParamsCurve();

  // Set the system to have an initial state with longitudinal velocity of
  // 10 m/s and a heading of pi/12 radians (~15 degrees).
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_v_LCp_x(test_values.v_LCp_x);
  continuous_state()->set_yaw_LC(test_values.yaw_LC);

  // Sets the input port to be pi/6 radians (~30 degrees) steering angle and
  // zero force input.
  context_->FixInputPort(
      0, Vector2<double>{test_values.steer_angle, test_values.f_Cp_x});

  TestTireSlipAngle(test_values);
  TestNormalLoad(test_values);
  TestLateralTireForce(test_values);
  TestCalcDerivatives(test_values);
}

// Test on a curve with negative steering angle.
TEST_F(DynamicBicycleCarTest, NegativeCurveTest) {
  TestValues test_values = GetParamsCurveNegative();

  // Set the system to have an initial state with longitudinal velocity of
  // 10 m/s and a heading of pi/12 radians (~15 degrees).
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_v_LCp_x(test_values.v_LCp_x);
  continuous_state()->set_yaw_LC(test_values.yaw_LC);

  // Sets the input port to be -pi/6 radians (~30 degrees) steering angle and
  // zero force input.
  context_->FixInputPort(
      0, Vector2<double>{test_values.steer_angle, test_values.f_Cp_x});

  TestTireSlipAngle(test_values);
  TestNormalLoad(test_values);
  TestLateralTireForce(test_values);
  TestCalcDerivatives(test_values);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
