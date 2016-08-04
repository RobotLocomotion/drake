#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/primitives/adder.h"

namespace drake {
namespace systems {
namespace {

/// Sets up the following diagram:
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    diagram_.reset(new Diagram<double>());
    diagram_->set_name("Unicode Snowman's Favorite Diagram!!1!☃!");

    adder0_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder0_->set_name("adder0");
    adder1_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder1_->set_name("adder1");
    adder2_.reset(new Adder<double>(2 /* inputs */, 3 /* length */));
    adder2_->set_name("adder2");

    diagram_->Connect(adder0_.get(), 0 /* src_port_index */, adder1_.get(),
                      0 /* dest_port_index */);
    diagram_->Connect(adder0_.get(), 0 /* src_port_index */, adder2_.get(),
                      0 /* dest_port_index */);
    diagram_->Connect(adder1_.get(), 0 /* src_port_index */, adder2_.get(),
                      1 /* dest_port_index */);

    diagram_->ExportInput(adder0_.get(), 0);
    diagram_->ExportInput(adder0_.get(), 1);
    diagram_->ExportInput(adder1_.get(), 1);
    diagram_->ExportOutput(adder1_.get(), 0);
    diagram_->ExportOutput(adder2_.get(), 0);

    diagram_->Finalize();

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
        dynamic_cast<const BasicVector<double>*>(
            output_->get_port(0).get_vector_data());
    ASSERT_NE(nullptr, output0);
    EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
    EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
    EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

    const BasicVector<double>* output1 =
        dynamic_cast<const BasicVector<double>*>(
            output_->get_port(1).get_vector_data());
    ASSERT_NE(nullptr, output1);
    EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
    EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
    EXPECT_EQ(expected_output1[2], output1->get_value()[2]);
  }

  static std::unique_ptr<FreestandingInputPort<double>> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return std::unique_ptr<FreestandingInputPort<double>>(
        new FreestandingInputPort<double>(std::move(data)));
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

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, SystemOfAdders) {
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(input1_)));
  context_->SetInputPort(2, MakeInput(std::move(input2_)));

  diagram_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(2, output_->get_num_ports());
  ExpectDefaultOutputs();
}

// Tests that an exception is thrown if the diagram contains a cycle.
TEST_F(DiagramTest, Cycle) {
  diagram_.reset(new Diagram<double>());
  diagram_->set_name("Diagram with a cycle.  Oh no!");
  auto adder = std::make_unique<Adder<double>>(1 /* inputs */, 1 /* length */);
  // Connect the output port to the input port.
  diagram_->Connect(adder.get(), 0, adder.get(), 0);
  EXPECT_THROW(diagram_->Finalize(), std::logic_error);
}

// Tests that an exception is thrown when finalizing a diagram that has already
// been finalized.
TEST_F(DiagramTest, Refinalize) {
  EXPECT_THROW(diagram_->Finalize(), std::logic_error);
}

// Tests that an exception is thrown when finalizing an empty diagram.
TEST_F(DiagramTest, FinalizeWhenEmpty) {
  diagram_.reset(new Diagram<double>());
  diagram_->set_name("Empty diagram!");
  EXPECT_THROW(diagram_->Finalize(), std::logic_error);
}

// Tests that an exception is thrown when getting the context of a diagram that
// has not been finalized.
TEST_F(DiagramTest, Unfinalized) {
  diagram_.reset(new Diagram<double>());
  diagram_->set_name("Unfinalized diagram!");
  auto adder = std::make_unique<Adder<double>>(1 /* inputs */, 1 /* length */);
  diagram_->ExportInput(adder.get(), 0);

  // Before the diagram is finalized, CreateDefaultContext should throw.
  EXPECT_THROW(diagram_->CreateDefaultContext(), std::logic_error);

  // After the diagram is finalized, CreateDefaultContext should not throw.
  diagram_->Finalize();
  EXPECT_NO_THROW(diagram_->CreateDefaultContext());
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
  const BasicVector<double>* output0 = dynamic_cast<const BasicVector<double>*>(
      output_->get_port(0).get_vector_data());
  ASSERT_NE(nullptr, output0);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Eigen::Vector3d expected_output1;
  expected_output1 << 3 + 8, 6 + 16, 9 + 32;  // A
  expected_output1 += expected_output0;       // A + B
  const BasicVector<double>* output1 = dynamic_cast<const BasicVector<double>*>(
      output_->get_port(1).get_vector_data());
  ASSERT_NE(nullptr, output1);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
  EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

  // Check that the context that was cloned is unaffected.
  diagram_->EvalOutput(*context_, output_.get());
  ExpectDefaultOutputs();
}

}  // namespace
}  // namespace systems
}  // namespace drake
