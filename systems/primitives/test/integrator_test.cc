#include "drake/systems/primitives/integrator.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 3;

class IntegratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator_.reset(new Integrator<double>(kLength));
    context_ = integrator_->CreateDefaultContext();
    derivatives_ = integrator_->AllocateTimeDerivatives();

    // Set the state to zero initially.
    ContinuousState<double>& xc = continuous_state();
    EXPECT_EQ(3, xc.size());
    EXPECT_EQ(3, xc.get_misc_continuous_state().size());
    xc.SetFromVector(Eigen::VectorXd::Zero(kLength));
  }

  ContinuousState<double>& continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  std::unique_ptr<System<double>> integrator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Tests that the system exports the correct topology.
TEST_F(IntegratorTest, Topology) {
  ASSERT_EQ(1, integrator_->num_input_ports());
  const auto& input_port = integrator_->get_input_port(0);
  EXPECT_EQ(kVectorValued, input_port.get_data_type());
  EXPECT_EQ(kLength, input_port.size());

  ASSERT_EQ(1, integrator_->num_output_ports());
  const auto& output_port = integrator_->get_output_port(0);
  EXPECT_EQ(kVectorValued, output_port.get_data_type());
  EXPECT_EQ(kLength, output_port.size());
}

// Tests that the output of an integrator is its state.
TEST_F(IntegratorTest, Output) {
  ASSERT_EQ(1, context_->num_input_ports());
  integrator_->get_input_port(0).FixValue(context_.get(),
      Eigen::Vector3d{1.0, 2.0, 3.0});

  Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  EXPECT_EQ(expected, integrator_->get_output_port(0).Eval(*context_));

  continuous_state().get_mutable_vector().SetAtIndex(1, 42.0);
  expected << 0.0, 42.0, 0.0;
  EXPECT_EQ(expected, integrator_->get_output_port(0).Eval(*context_));
}

// Tests that the derivatives of an integrator's state are its input.
TEST_F(IntegratorTest, Derivatives) {
  ASSERT_EQ(1, context_->num_input_ports());
  integrator_->get_input_port(0).FixValue(context_.get(),
                                          Eigen::Vector3d{1.0, 2.0, 3.0});

  integrator_->CalcTimeDerivatives(*context_, derivatives_.get());
  Eigen::Vector3d expected(1.0, 2.0, 3.0);
  EXPECT_EQ(expected, derivatives_->CopyToVector());
}

// Asserts that integrators do not have any direct feedthrough inputs.
TEST_F(IntegratorTest, IntegratorIsNotDirectFeedthrough) {
  EXPECT_FALSE(integrator_->HasAnyDirectFeedthrough());
}

class SymbolicIntegratorTest : public IntegratorTest {
 protected:
  void SetUp() override {
    IntegratorTest::SetUp();
    symbolic_integrator_ = integrator_->ToSymbolic();
    symbolic_context_ = symbolic_integrator_->CreateDefaultContext();
    symbolic_derivatives_ = symbolic_integrator_->AllocateTimeDerivatives();

    ASSERT_EQ(1, symbolic_context_->num_input_ports());
    symbolic_integrator_->get_input_port(0).FixValue(symbolic_context_.get(),
        Vector3<symbolic::Expression>{symbolic::Variable("u0"),
                                      symbolic::Variable("u1"),
                                      symbolic::Variable("u2")});

    auto& xc = symbolic_context_->get_mutable_continuous_state_vector();
    xc[0] = symbolic::Variable("x0");
    xc[1] = symbolic::Variable("x1");
    xc[2] = symbolic::Variable("x2");
  }

  std::unique_ptr<System<symbolic::Expression>> symbolic_integrator_;
  std::unique_ptr<Context<symbolic::Expression>> symbolic_context_;
  std::unique_ptr<ContinuousState<symbolic::Expression>> symbolic_derivatives_;
};

TEST_F(SymbolicIntegratorTest, Output) {
  const auto& out =
      symbolic_integrator_->get_output_port(0).Eval(*symbolic_context_);
  EXPECT_EQ("x0", out[0].to_string());
  EXPECT_EQ("x1", out[1].to_string());
  EXPECT_EQ("x2", out[2].to_string());
}

TEST_F(SymbolicIntegratorTest, Derivatives) {
  symbolic_integrator_->CalcTimeDerivatives(*symbolic_context_,
                                            symbolic_derivatives_.get());
  const auto& xcdot = *symbolic_derivatives_;
  EXPECT_EQ("u0", xcdot[0].to_string());
  EXPECT_EQ("u1", xcdot[1].to_string());
  EXPECT_EQ("u2", xcdot[2].to_string());
}

TEST_F(IntegratorTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*integrator_));
}

TEST_F(IntegratorTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*integrator_));
}

}  // namespace
}  // namespace systems
}  // namespace drake
