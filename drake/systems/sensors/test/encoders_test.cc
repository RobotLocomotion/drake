#include "drake/systems/sensors/encoders.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {

// Test with the simple quantization-only constructor.
GTEST_TEST(TestEncoders, QuantizeOnly) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<unsigned int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / M_2_PI, tick_counts[1] / M_2_PI;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  srand(42);
  for (unsigned int i=0; i<10; i++) {
    angle = Eigen::Vector2d::Random();
    using std::floor;  using std::ceil;
    for (unsigned int j=0; j<2; j++) {
      if (angle(j)<0.0)
        desired_measurement(j) = ceil(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
      else
        desired_measurement(j) = floor(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
    }

    context->FixInputPort(0,angle);
    encoders.EvalOutput(*context,output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test with the simple selector-only constructor.
GTEST_TEST(TestEncoders, SelectorOnly) {
  // Construct a system with no quantization, and only inputs 2 and 3 are passed.
  const std::vector<unsigned int> indices = {1, 2};
  systems::sensors::RotaryEncoders<double> encoders(4,indices);

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector4d angle;
  Eigen::Vector2d desired_measurement;

  srand(42);
  for (unsigned int i=0; i<10; i++) {
    angle = Eigen::Vector4d::Random();
    desired_measurement = angle.segment(1,2);

    context->FixInputPort(0,angle);
    encoders.EvalOutput(*context,output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test the calibration offsets (via the parameters)
GTEST_TEST(TestEncoders, CalibrationOffsets) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<unsigned int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  Eigen::Vector2d ticks_per_radian;
  ticks_per_radian << tick_counts[0] / M_2_PI, tick_counts[1] / M_2_PI;

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  // set some offsets
  Eigen::Vector2d offsets;
  offsets << 0.1, M_PI_4;
  encoders.set_calibration_offsets(context.get(),offsets);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  srand(42);
  for (unsigned int i=0; i<10; i++) {
    angle = Eigen::Vector2d::Random();
    context->FixInputPort(0,angle);

    angle -= offsets;
    using std::floor;  using std::ceil;
    for (unsigned int j=0; j<2; j++) {
      if (angle(j)<0.0)
        desired_measurement(j) = ceil(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
      else
        desired_measurement(j) = floor(angle(j) * ticks_per_radian(j)) / ticks_per_radian(j);
    }

    encoders.EvalOutput(*context,output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

}  // namespace drake
