#include "drake/systems/estimators/extended_kalman_filter.h"

#include <gtest/gtest.h>

#include "drake/systems/estimators/test/nonlinear_kalman_filter_test.h"

namespace drake {
namespace systems {
namespace estimators {
namespace {

class DiscreteTimeEKFTest : public DiscreteTimeNonlinearKalmanFilterTest<
                                ExtendedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
    return ExtendedKalmanFilter(observed_system,
                                *observed_system.CreateDefaultContext(), W, V,
                                options);
  }
};

TEST_F(DiscreteTimeEKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(DiscreteTimeEKFTest, ContructionSqrt) {
  this->TestConstruction(true);
}
TEST_F(DiscreteTimeEKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, VectorInputDynamicsSqrt) {
  this->TestVectorInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, NoInputDynamicsSqrt) {
  this->TestNoInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, AbstractInputDynamicsSqrt) {
  this->TestAbstractInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, ProcessNoiseInputDynamicsSqrt) {
  this->TestProcessNoiseInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, MeasurementNoiseInputDynamicsSqrt) {
  this->TestMeasurementNoiseInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeEKFTest, ProcessAndMeasurementNoiseInputDynamicsSqrt) {
  this->TestProcessAndMeasurementNoiseInputDynamics(true);
}
TEST_F(DiscreteTimeEKFTest, SteadyState) {
  this->TestSteadyState(false);
}
TEST_F(DiscreteTimeEKFTest, SteadyStateSqrt) {
  this->TestSteadyState(true);
}

class ContinuousDiscreteEKFTest
    : public ContinuousDiscreteNonlinearKalmanFilterTest<
          ExtendedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(options.discrete_measurement_time_period.has_value());
    return ExtendedKalmanFilter(observed_system,
                                *observed_system.CreateDefaultContext(), W, V,
                                options);
  }
};

TEST_F(ContinuousDiscreteEKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(ContinuousDiscreteEKFTest, ContructionSqrt) {
  this->TestConstruction(true);
}
TEST_F(ContinuousDiscreteEKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, VectorInputDynamicsSqrt) {
  this->TestVectorInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, NoInputDynamicsSqrt) {
  this->TestNoInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, AbstractInputDynamicsSqrt) {
  this->TestAbstractInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, ProcessNoiseInputDynamicsSqrt) {
  this->TestProcessNoiseInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, MeasurementNoiseInputDynamicsSqrt) {
  this->TestMeasurementNoiseInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteEKFTest, ProcessAndMeasurementNoiseInputDynamicsSqrt) {
  this->TestProcessAndMeasurementNoiseInputDynamics(true);
}
TEST_F(ContinuousDiscreteEKFTest, Simulation) {
  this->TestSimulation(false);
}
TEST_F(ContinuousDiscreteEKFTest, SimulationSqrt) {
  this->TestSimulation(true);
}

class ContinuousTimeEKFTest : public ContinuousTimeNonlinearKalmanFilterTest<
                                  ExtendedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
    return ExtendedKalmanFilter(observed_system,
                                *observed_system.CreateDefaultContext(), W, V,
                                options);
  }
};

TEST_F(ContinuousTimeEKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(ContinuousTimeEKFTest, ContructionSqrt) {
  this->TestConstruction(true);
}
TEST_F(ContinuousTimeEKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, VectorInputDynamicsSqrt) {
  this->TestVectorInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, NoInputDynamicsSqrt) {
  this->TestNoInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, AbstractInputDynamicsSqrt) {
  this->TestAbstractInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, ProcessNoiseInputDynamicsSqrt) {
  this->TestProcessNoiseInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, MeasurementNoiseInputDynamicsSqrt) {
  this->TestMeasurementNoiseInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeEKFTest, ProcessAndMeasurementNoiseInputDynamicsSqrt) {
  this->TestProcessAndMeasurementNoiseInputDynamics(true);
}
TEST_F(ContinuousTimeEKFTest, SteadyState) {
  this->TestSteadyState(false);
}
TEST_F(ContinuousTimeEKFTest, SteadyStateSqrt) {
  this->TestSteadyState(true);
}

}  // namespace

namespace internal {
namespace {

template <typename T>
class ConcatenateVectorAndMatrixTest : public ::testing::Test {};

using DefaultScalarsTypes =
    ::testing::Types<double, AutoDiffXd, symbolic::Expression>;
TYPED_TEST_SUITE(ConcatenateVectorAndMatrixTest, DefaultScalarsTypes);

TYPED_TEST(ConcatenateVectorAndMatrixTest, SquareMatrix) {
  using T = TypeParam;
  Eigen::VectorX<T> vec(2);
  Eigen::MatrixX<T> mat(2, 2);
  vec << 1, 2;
  mat << 3, 4, 5, 6;

  Eigen::VectorX<T> concat = ConcatenateVectorAndSquareMatrix<T>(vec, mat);
  Eigen::VectorX<T> expected(6);
  expected << 1, 2, 3, 5, 4, 6;
  EXPECT_TRUE(CompareMatrices(concat, expected));

  Eigen::MatrixX<T> mat_out(2, 2);
  ExtractSquareMatrix<T>(concat, mat_out);
  EXPECT_TRUE(CompareMatrices(mat, mat_out));
}

TYPED_TEST(ConcatenateVectorAndMatrixTest, LowerTriMatrix) {
  using T = TypeParam;
  Eigen::VectorX<T> vec(2);
  Eigen::MatrixX<T> mat(2, 2);
  vec << 1, 2;
  mat << 3, 0, 4, 5;

  // Overload taking matrix.
  Eigen::VectorX<T> concat = ConcatenateVectorAndLowerTriMatrix<T>(vec, mat);
  Eigen::VectorX<T> expected(5);
  expected << 1, 2, 3, 4, 5;
  EXPECT_TRUE(CompareMatrices(concat, expected));

  // Overload taking triangular view.
  const Eigen::MatrixX<T> mat2 = mat;
  Eigen::VectorX<T> concat2 = ConcatenateVectorAndLowerTriMatrix(
      vec, mat2.template triangularView<Eigen::Lower>());
  EXPECT_TRUE(CompareMatrices(concat2, expected));

  Eigen::MatrixX<T> mat_out(2, 2);
  ExtractLowerTriMatrix<T>(concat, mat_out);
  EXPECT_TRUE(CompareMatrices(mat, mat_out));
}

}  // namespace
}  // namespace internal

}  // namespace estimators
}  // namespace systems
}  // namespace drake
