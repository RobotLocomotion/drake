#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace {

std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<double>> data) {
  return std::make_unique<FreestandingInputPort>(std::move(data));
}

/// Sets up the following diagram:
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    DiagramBuilder<double> builder;

    adder0_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder0_->set_name("adder0");
    adder1_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder1_->set_name("adder1");
    adder2_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder2_->set_name("adder2");

    builder.Connect(adder0_->get_output_port(0), adder1_->get_input_port(0));
    builder.Connect(adder0_->get_output_port(0), adder2_->get_input_port(0));
    builder.Connect(adder1_->get_output_port(0), adder2_->get_input_port(1));

    builder.ExportInput(adder0_->get_input_port(0));
    builder.ExportInput(adder0_->get_input_port(1));
    builder.ExportInput(adder1_->get_input_port(1));
    builder.ExportOutput(adder1_->get_output_port(0));
    builder.ExportOutput(adder2_->get_output_port(0));

    diagram_ = builder.Build();
    diagram_->set_name("Unicode Snowman's Favorite Diagram!!1!☃!");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_.reset(new BasicVector<double>(3 /* length */));
    input0_->get_mutable_value() << 1, 2, 4;
    input1_.reset(new BasicVector<double>(3 /* length */));
    input1_->get_mutable_value() << 8, 16, 32;
    input2_.reset(new BasicVector<double>(3 /* length */));
    input2_->get_mutable_value() << 64, 128, 256;
  }

  // Asserts that output_ is what it should be for the default values
  // of input0_, input1_, and input2_.
  void ExpectDefaultOutputs() {
    Eigen::Vector3d expected_output0;
    expected_output0 << 1 + 8 + 64, 2 + 16 + 128, 4 + 32 + 256;  // B

    Eigen::Vector3d expected_output1;
    expected_output1 << 1 + 8, 2 + 16, 4 + 32;  // A
    expected_output1 += expected_output0;       // A + B

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
  }

  std::unique_ptr<Diagram<double>> diagram_;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Adder<double>> adder2_;

  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram exports the correct topology.
TEST_F(DiagramTest, Topology) {
  ASSERT_EQ(3u, diagram_->get_input_ports().size());
  for (const auto& descriptor : diagram_->get_input_ports()) {
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kInputPort, descriptor.get_face());
    EXPECT_EQ(3, descriptor.get_size());
    EXPECT_EQ(kInheritedSampling, descriptor.get_sampling());
  }

  ASSERT_EQ(2u, diagram_->get_output_ports().size());
  for (const auto& descriptor : diagram_->get_output_ports()) {
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kOutputPort, descriptor.get_face());
    EXPECT_EQ(3, descriptor.get_size());
    EXPECT_EQ(kInheritedSampling, descriptor.get_sampling());
  }
}

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {
  // Attach inputs and eval the output.
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(input1_)));
  context_->SetInputPort(2, MakeInput(std::move(input2_)));

  diagram_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(2, output_->get_num_ports());
  ExpectDefaultOutputs();
}

// Tests that the same diagram can be evaluated into the same output with
// different contexts interchangeably.
TEST_F(DiagramTest, Clone) {
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(input1_)));
  context_->SetInputPort(2, MakeInput(std::move(input2_)));

  // Compute the output with the default inputs and sanity-check it.
  diagram_->EvalOutput(*context_, output_.get());
  ExpectDefaultOutputs();

  // Create a clone of the context and change an input.
  auto clone = context_->Clone();

  auto next_input_0 = std::make_unique<BasicVector<double>>(3 /* length */);
  next_input_0->get_mutable_value() << 3, 6, 9;
  clone->SetInputPort(0, MakeInput(std::move(next_input_0)));

  // Recompute the output and check the values.
  diagram_->EvalOutput(*clone, output_.get());

  Eigen::Vector3d expected_output0;
  expected_output0 << 3 + 8 + 64, 6 + 16 + 128, 9 + 32 + 256;  // B
  const BasicVector<double>* output0 =
      dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output0);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Eigen::Vector3d expected_output1;
  expected_output1 << 3 + 8, 6 + 16, 9 + 32;  // A
  expected_output1 += expected_output0;       // A + B
  const BasicVector<double>* output1 =
      dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(1));
  ASSERT_NE(nullptr, output1);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
  EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

  // Check that the context that was cloned is unaffected.
  diagram_->EvalOutput(*context_, output_.get());
  ExpectDefaultOutputs();
}

// A Diagram that adds a constant to an input, and outputs the sum.
class AddConstantDiagram : public Diagram<double> {
 public:
  explicit AddConstantDiagram(double constant) : Diagram<double>() {
    constant_.reset(new ConstantVectorSource<double>(Vector1d{constant}));
    adder_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));

    DiagramBuilder<double> builder;
    builder.Connect(constant_->get_output_port(0), adder_->get_input_port(1));
    builder.ExportInput(adder_->get_input_port(0));
    builder.ExportOutput(adder_->get_output_port(0));
    Initialize(builder.Compile());
  }

 private:
  std::unique_ptr<Adder<double>> adder_;
  std::unique_ptr<ConstantVectorSource<double>> constant_;
};

GTEST_TEST(DiagramSubclassTest, TwelvePlusSevenIsNineteen) {
  AddConstantDiagram plus_seven(7.0);
  auto context = plus_seven.CreateDefaultContext();
  auto output = plus_seven.AllocateOutput(*context);
  ASSERT_NE(nullptr, context);
  ASSERT_NE(nullptr, output);

  auto vec = std::make_unique<BasicVector<double>>(1 /* length */);
  vec->get_mutable_value() << 12.0;
  context->SetInputPort(0, MakeInput(std::move(vec)));

  plus_seven.EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const VectorBase<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(1, output_vector->get_value().rows());
  EXPECT_EQ(19.0, output_vector->get_value().x());
}

}  // namespace
}  // namespace systems
}  // namespace drake
