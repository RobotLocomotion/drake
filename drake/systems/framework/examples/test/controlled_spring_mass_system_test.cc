#include "drake/systems/framework/examples/controlled_spring_mass_system.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/system_port_descriptor.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

const double kSpring = 300.0;  // N/m
const double kMass = 2.0;      // kg
const double Kp = 1.0;
const double Kd = 1.0;
const double Ki = 1.0;

class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ =
        make_unique<PidControlledSpringMassSystem<double>>(
            kSpring, kMass, Kp, Ki, Kd);

    context_ = model_->CreateDefaultContext();
    //output_ = model_->AllocateOutput(*context_);

    // Initialize to default conditions.
    //model_->SetDefaultState(context_.get());
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>* GetMutableContinuousState(
      const System<double>* system) {
    return model_->GetMutableSubsystemState(context_.get(), system)
        ->continuous_state.get();
  }

#if 0
  // Asserts that output_ is what it should be for the default values
  // of input0_, input1_, and input2_.
  void ExpectDefaultOutputs() {
    Eigen::Vector3d expected_output0;
    expected_output0 << 1 + 8 + 64, 2 + 16 + 128, 4 + 32 + 256;  // B

    Eigen::Vector3d expected_output1;
    expected_output1 << 1 + 8, 2 + 16, 4 + 32;  // A
    expected_output1 += expected_output0;       // A + B

    Eigen::Vector3d expected_output2;
    expected_output2 << 81, 243, 729;  // state of integrator1_

    const BasicVector<double>* output0 =
        dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
    ASSERT_NE(nullptr, output0);
    EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
    EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
    EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

    const BasicVector<double>* output1 =
        dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(1));
    ASSERT_NE(nullptr, output1);
    EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
    EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
    EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

    const BasicVector<double>* output2 =
        dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(2));
    ASSERT_NE(nullptr, output2);
    EXPECT_EQ(expected_output2[0], output2->get_value()[0]);
    EXPECT_EQ(expected_output2[1], output2->get_value()[1]);
    EXPECT_EQ(expected_output2[2], output2->get_value()[2]);
  }
#endif

  std::unique_ptr<PidControlledSpringMassSystem<double>> model_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {

  //model_->EvalOutput(*context_, output_.get());

  //ASSERT_EQ(3, output_->get_num_ports());
  //ExpectDefaultOutputs();
}

#if 0
TEST_F(DiagramTest, EvalTimeDerivatives) {
  AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();

  diagram_->EvalTimeDerivatives(*context_, derivatives.get());

  ASSERT_EQ(6, derivatives->get_state().size());
  ASSERT_EQ(0, derivatives->get_generalized_position().size());
  ASSERT_EQ(0, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(6, derivatives->get_misc_continuous_state().size());

  // The derivative of the first integrator is A.
  const ContinuousState<double>& integrator0_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator0_.get());
  EXPECT_EQ(1 + 8, integrator0_xcdot.get_state().GetAtIndex(0));
  EXPECT_EQ(2 + 16, integrator0_xcdot.get_state().GetAtIndex(1));
  EXPECT_EQ(4 + 32, integrator0_xcdot.get_state().GetAtIndex(2));

  // The derivative of the second integrator is the state of the first.
  const ContinuousState<double>& integrator1_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator1_.get());
  EXPECT_EQ(3, integrator1_xcdot.get_state().GetAtIndex(0));
  EXPECT_EQ(9, integrator1_xcdot.get_state().GetAtIndex(1));
  EXPECT_EQ(27, integrator1_xcdot.get_state().GetAtIndex(2));
}
#endif

}  // namespace
}  // namespace systems
}  // namespace drake
