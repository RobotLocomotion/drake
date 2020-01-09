#include "drake/systems/sensors/rotary_encoders.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace {

constexpr double k2Pi = 2.0 * M_PI;

// Test with the simple quantization-only constructor.
GTEST_TEST(TestEncoders, QuantizeOnly) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / k2Pi, tick_counts[1] / k2Pi;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput();
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector2d::Random();
    using std::floor;
    for (int j = 0; j < 2; j++) {
      desired_measurement(j) =
          floor(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
    }

    encoders.get_input_port().FixValue(context.get(), angle);
    encoders.CalcOutput(*context, output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement,
                                measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test with the simple selector-only constructor.
GTEST_TEST(TestEncoders, SelectorOnly) {
  // Construct a system with no quantization, and only inputs 1 and 2 are
  // passed.
  const std::vector<int> indices = {1, 2};
  systems::sensors::RotaryEncoders<double> encoders(4, indices);

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput();
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector4d angle;
  Eigen::Vector2d desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector4d::Random();
    desired_measurement = angle.segment(1, 2);

    encoders.get_input_port().FixValue(context.get(), angle);
    encoders.CalcOutput(*context, output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement,
                                measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test with the simple quantization and selector constructor.
GTEST_TEST(TestEncoders, QuantizationAndSelector) {
  // Construct a system with quantization, and only inputs 1 and 2 are
  // passed.
  const std::vector<int> indices = {1, 2};
  const std::vector<int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(4, indices, tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / k2Pi, tick_counts[1] / k2Pi;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput();
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector4d angle;
  Eigen::Vector2d desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector4d::Random();
    encoders.get_input_port().FixValue(context.get(), angle);

    using std::floor;
    using std::ceil;
    for (int j = 0; j < 2; j++) {
      desired_measurement(j) =
          floor(angle(indices[j]) * ticks_per_radian(j)) / ticks_per_radian(j);
    }

    encoders.CalcOutput(*context, output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement,
                                measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test the calibration offsets (via the parameters).
GTEST_TEST(TestEncoders, CalibrationOffsets) {
  // Construct a system where all inputs are encoders, with quantization.

  const std::vector<int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / k2Pi, tick_counts[1] / k2Pi;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput();
  auto measurement = output->get_vector_data(0);

  // Set some offsets.
  Eigen::Vector2d offsets;
  offsets << 0.1, M_PI_4;
  encoders.set_calibration_offsets(context.get(), offsets);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector2d::Random();
    encoders.get_input_port().FixValue(context.get(), angle);

    angle -= offsets;
    using std::floor;
    using std::ceil;
    for (int j = 0; j < 2; j++) {
      desired_measurement(j) =
          floor(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
    }

    encoders.CalcOutput(*context, output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement,
                                measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test ToAutoDiff and ToSymbolic.
GTEST_TEST(TestEncoders, ScalarConversion) {
  using Expression = symbolic::Expression;

  const std::vector<int> indices = {1, 2};
  const std::vector<int> tick_counts = {2, 4};
  systems::sensors::RotaryEncoders<double> encoders(4, indices, tick_counts);

  // Sanity check AutoDiff form.  We rely on symbolic form to test correctness.
  EXPECT_TRUE(is_autodiffxd_convertible(encoders));

  // Check that both the indices and tick_counts made it into symbolic form.
  EXPECT_TRUE(is_symbolic_convertible(encoders, [&](
      const systems::sensors::RotaryEncoders<Expression>& dut) {
    auto context = dut.CreateDefaultContext();

    // Set input to be symbolic variables.
    const Vector4<Expression> input{
      symbolic::Variable("u0"),
      symbolic::Variable("u1"),
      symbolic::Variable("u2"),
      symbolic::Variable("u3")
    };
    dut.get_input_port().FixValue(context.get(), input);

    // Obtain the symbolic outputs.
    auto outputs = dut.AllocateOutput();
    dut.CalcOutput(*context, outputs.get());
    const systems::BasicVector<Expression>& output =
        *(outputs->get_vector_data(0));
    ASSERT_EQ(output.size(), 2);

    // Symbolic form should be as expected.
    using symbolic::test::ExprEqual;
    EXPECT_PRED2(ExprEqual, output[0], floor((M_1_PI * input[1])) / M_1_PI);
    EXPECT_PRED2(ExprEqual, output[1], floor((M_2_PI * input[2])) / M_2_PI);
  }));
}

}  // namespace
}  // namespace drake
