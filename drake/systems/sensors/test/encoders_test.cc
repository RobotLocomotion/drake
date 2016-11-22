#include "drake/systems/sensors/encoders.h"

#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {

// Test with the simple quantization-only constructor.
GTEST_TEST(TestEncoders, QuantizeOnly) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<unsigned int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  srand(42);
  for (unsigned int i=0; i<10; i++) {
    angle = Eigen::Vector2d::Random();
    for (unsigned int j=0; j<2; j++)
      desired_measurement(j) = 2.0*M_PI/tick_counts[j]*floor(angle(j)*tick_counts[j]/(2.0*M_PI));

    context->FixInputPort(0,angle);
    encoders.EvalOutput(*context,output.get());

    std::cout << "angle = " << angle.transpose() << std::endl;
    std::cout << "measurement = " << measurement->CopyToVector().transpose() << std::endl;

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

// Test the calibration offsets (via the parameters)
GTEST_TEST(TestEncoders, CalibrationOffsets) {
  // Construct a system where all inputs are encoders, with quantization

  const std::vector<unsigned int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

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
    for (unsigned int j=0; j<2; j++)
      desired_measurement(j) = 2.0*M_PI/tick_counts[j]*floor(angle(j)*tick_counts[j]/(2.0*M_PI));

    encoders.EvalOutput(*context,output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}
}  // namespace drake
