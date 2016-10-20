# pragma once

#include "drake/systems/framework/primitives/affine_system_plant.h"

#include "gtest/gtest.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {

// Base class for the tests on Affine and Linear Systems.
class AffineLinearSystemTest : public ::testing::Test {
 public:
  // Setup an arbitrary AffineSystem.
  AffineLinearSystemTest(double xDot0_0, double xDot0_1,
                         double Y0_0, double Y0_1)
      : A_(make_2x2_matrix(1.5, 2.7, 3.5, -4.9)),
        B_(make_2x2_matrix(4.9, -5.1, 6.8, 7.2)),
        XDot0_(make_2x1_vector(xDot0_0, xDot0_1)),
        C_(make_2x2_matrix(1.1, 2.5, -3.8, 4.6)),
        D_(make_2x2_matrix(4.1, 5.6, -6.3, 7.7)),
        Y0_(make_2x1_vector(Y0_0, Y0_1)) { }

  void SetUp() override { Initialize(); }

  virtual void Initialize() = 0;

  void SetState(const Eigen::Ref<const VectorX<double>> &x) {
    state_vector_ = make_unique < BasicVector < double >> (x);
    state_ = make_unique < ContinuousState < double >> (move(state_vector_));
    context_->set_continuous_state(std::move(state_));
  }

  void SetInput(const Eigen::Ref<const VectorX<double>> &u) {
    input_vector_->get_mutable_value() << u;
    context_->SetInputPort(
        0, make_unique<FreestandingInputPort>(std::move(input_vector_)));
  }

  // Helper method to create free standing input ports, i.e. those are
  // not connected to any other output port in the system.
  // Used to test standalone systems not part of a Diagram.
  static std::unique_ptr <FreestandingInputPort> MakeInput(
      std::unique_ptr <BasicVector<double>> data) {
    return make_unique<FreestandingInputPort>(std::move(data));
  }

 protected:
  unique_ptr <Context<double>> context_;
  LeafSystemOutput<double> system_output_;
  unique_ptr <ContinuousState<double>> system_derivatives_;

  unique_ptr <ContinuousState<double>> state_;
  unique_ptr <BasicVector<double>> state_vector_;
  unique_ptr <BasicVector<double>> input_vector_;
  unique_ptr <ContinuousState<double>> derivatives_;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::VectorXd XDot0_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::VectorXd Y0_;

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
