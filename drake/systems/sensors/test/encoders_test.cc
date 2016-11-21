#include "drake/systems/sensors/encoders.h"

#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {

GTEST_TEST(TestEncoders, QuantizeOnly) {
  // Construct a system where all inputs are encoders, with quantization

  std::vector<unsigned int> tick_counts = {100, 50};
  systems::sensors::RotaryEncoders<double> encoders(tick_counts);

  auto context = encoders.CreateDefaultContext();
  auto output = encoders.AllocateOutput(*context);
  auto measurement = output->get_vector_data(0);

  double tol = 1e-10;
  Eigen::Vector2d angle, desired_measurement;

  for (unsigned int i=0; i<10; i++) {
    angle = Eigen::Vector2d::Random();
    for (unsigned int j=0; j<2; j++)
      desired_measurement(j) = 2.0*M_PI/tick_counts[j]*floor(angle(j)*tick_counts[j]/(2.0*M_PI));

    encoders.EvalOutput(*context,output.get());

    EXPECT_TRUE(CompareMatrices(desired_measurement, measurement->CopyToVector(), tol,
                                MatrixCompareType::absolute));
  }
}

}  // namespace drake
