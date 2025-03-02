#include "drake/systems/estimators/unscented_kalman_filter.h"

#include <gtest/gtest.h>

#include "drake/systems/estimators/test/nonlinear_kalman_filter_test.h"

namespace drake {
namespace systems {
namespace estimators {
namespace {
class DiscreteTimeUKFTest : public DiscreteTimeNonlinearKalmanFilterTest<
                                UnscentedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const UnscentedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
    return UnscentedKalmanFilter(observed_system,
                                 *observed_system.CreateDefaultContext(), W, V,
                                 options);
  }
};

TEST_F(DiscreteTimeUKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(DiscreteTimeUKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(DiscreteTimeUKFTest, SteadyState) {
  this->TestSteadyState(false);
}

class ContinuousDiscreteUKFTest
    : public ContinuousDiscreteNonlinearKalmanFilterTest<
          UnscentedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const UnscentedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(options.discrete_measurement_time_period.has_value());
    return UnscentedKalmanFilter(observed_system,
                                 *observed_system.CreateDefaultContext(), W, V,
                                 options);
  }
};

TEST_F(ContinuousDiscreteUKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(ContinuousDiscreteUKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousDiscreteUKFTest, Simulation) {
  this->TestSimulation(false);
}

class ContinuousTimeUKFTest : public ContinuousTimeNonlinearKalmanFilterTest<
                                  UnscentedKalmanFilterOptions> {
 private:
  std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const UnscentedKalmanFilterOptions& options) const override {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
    return UnscentedKalmanFilter(observed_system,
                                 *observed_system.CreateDefaultContext(), W, V,
                                 options);
  }
};

TEST_F(ContinuousTimeUKFTest, Contruction) {
  this->TestConstruction(false);
}
TEST_F(ContinuousTimeUKFTest, VectorInputDynamics) {
  this->TestVectorInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, NoInputDynamics) {
  this->TestNoInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, AbstractInputDynamics) {
  this->TestAbstractInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, ProcessNoiseInputDynamics) {
  this->TestProcessNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, MeasurementNoiseInputDynamics) {
  this->TestMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, ProcessAndMeasurementNoiseInputDynamics) {
  this->TestProcessAndMeasurementNoiseInputDynamics(false);
}
TEST_F(ContinuousTimeUKFTest, SteadyState) {
  this->TestSteadyState(false);
}

}  // namespace
}  // namespace estimators
}  // namespace systems
}  // namespace drake
