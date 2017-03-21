#include "drake/systems/sensors/rotary_encoders.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace {

// Test with the simple quantization-only constructor.
GTEST_TEST(TestEncoders, QuantizeOnly) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / M_2_PI, tick_counts[1] / M_2_PI;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
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

    context->FixInputPort(0, angle);
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
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector4d angle;
  Eigen::Vector2d desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector4d::Random();
    desired_measurement = angle.segment(1, 2);

    context->FixInputPort(0, angle);
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
  ticks_per_radian << tick_counts[0] / M_2_PI, tick_counts[1] / M_2_PI;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector4d angle;
  Eigen::Vector2d desired_measurement;

  // TODO(russt): Use c++11 random generators instead (w/ some wrappers).
  srand(42);
  for (int i = 0; i < 10; i++) {
    angle = Eigen::Vector4d::Random();
    context->FixInputPort(0, angle);

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
  ticks_per_radian << tick_counts[0] / M_2_PI, tick_counts[1] / M_2_PI;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
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
    context->FixInputPort(0, angle);

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

}  // namespace
}  // namespace drake
