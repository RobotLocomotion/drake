#pragma once

#include "drake/systems/framework/leaf_system.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {

// Base class for the tests on Affine and Linear Systems.
class AffineLinearSystemTest : public ::testing::Test {
 public:
  // Setup an arbitrary AffineSystem.
  AffineLinearSystemTest(double xDot0_0, double xDot0_1, double y0_0,
                         double y0_1)
      : A_(make_2x2_matrix(1.5, 2.7, 3.5, -4.9)),
        B_(make_2x2_matrix(4.9, -5.1, 6.8, 7.2)),
        xDot0_(make_2x1_vector(xDot0_0, xDot0_1)),
        C_(make_2x2_matrix(1.1, 2.5, -3.8, 4.6)),
        D_(make_2x2_matrix(4.1, 5.6, -6.3, 7.7)),
        y0_(make_2x1_vector(y0_0, y0_1)) {}

  void SetUp() override { Initialize(); }

  virtual void Initialize() = 0;

  void SetInput(const Eigen::Ref<const VectorX<double>>& u) {
    input_vector_->get_mutable_value() << u;
    context_->SetInputPort(
        0, std::make_unique<FreestandingInputPort>(std::move(input_vector_)));
  }

  // Helper method to create free standing input ports, i.e., those that are
  // not connected to any other output port in the system.
  // Used to test standalone systems not part of a Diagram.
  static std::unique_ptr<FreestandingInputPort> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return std::make_unique<FreestandingInputPort>(std::move(data));
  }

 protected:
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> system_output_;

  ContinuousState<double>* state_{};
  std::unique_ptr<BasicVector<double>> state_vector_;
  std::unique_ptr<BasicVector<double>> input_vector_;
  std::unique_ptr<ContinuousState<double>> derivatives_;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::VectorXd xDot0_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::VectorXd y0_;

  Eigen::MatrixXd make_2x2_matrix(double a, double b, double c, double d) {
    Eigen::MatrixXd m(2, 2);
    m << a, b, c, d;
    return m;
  }

  Eigen::VectorXd make_2x1_vector(double a, double b) {
    Eigen::VectorXd v(2);
    v << a, b;
    return v;
  }
};

}  // namespace systems
}  // namespace drake
