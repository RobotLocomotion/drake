#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace {

std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<double>> data) {
  return std::make_unique<FreestandingInputPort>(std::move(data));
}

/// ExampleDiagram has the following structure:
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
/// integrator1_: A              -> C
/// integrator2_: C              -> output 2
class ExampleDiagram : public Diagram<double> {
 public:
  explicit ExampleDiagram(int length) {
    DiagramBuilder<double> builder;

    adder0_.reset(new Adder<double>(2 /* inputs */, length));
    adder0_->set_name("adder0");
    adder1_.reset(new Adder<double>(2 /* inputs */, length));
    adder1_->set_name("adder1");
    adder2_.reset(new Adder<double>(2 /* inputs */, length));
    adder2_->set_name("adder2");

    integrator0_.reset(new Integrator<double>(length));
    integrator1_.reset(new Integrator<double>(length));

    builder.Connect(adder0_->get_output_port(0), adder1_->get_input_port(0));
    builder.Connect(adder0_->get_output_port(0), adder2_->get_input_port(0));
    builder.Connect(adder1_->get_output_port(0), adder2_->get_input_port(1));

    builder.Connect(adder0_->get_output_port(0),
                    integrator0_->get_input_port(0));
    builder.Connect(integrator0_->get_output_port(0),
                    integrator1_->get_input_port(0));

    builder.ExportInput(adder0_->get_input_port(0));
    builder.ExportInput(adder0_->get_input_port(1));
    builder.ExportInput(adder1_->get_input_port(1));
    builder.ExportOutput(adder1_->get_output_port(0));
    builder.ExportOutput(adder2_->get_output_port(0));
    builder.ExportOutput(integrator1_->get_output_port(0));

    builder.BuildInto(this);
  }

  Integrator<double>* integrator0() { return integrator0_.get(); }
  Integrator<double>* integrator1() { return integrator1_.get(); }

 private:
  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Adder<double>> adder2_;

  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;
};

class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    diagram_ = std::make_unique<ExampleDiagram>(kLength);
    diagram_->set_name("Unicode Snowman's Favorite Diagram!!1!☃!");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_.reset(new BasicVector<double>({1, 2, 4}));
    input1_.reset(new BasicVector<double>({8, 16, 32}));
    input2_.reset(new BasicVector<double>({64, 128, 256}));

    // Initialize the integrator states.
    auto integrator0_xc = GetMutableContinuousState(integrator0());
    ASSERT_NE(nullptr, integrator0_xc);
    integrator0_xc->get_mutable_state()->SetAtIndex(0, 3);
    integrator0_xc->get_mutable_state()->SetAtIndex(1, 9);
    integrator0_xc->get_mutable_state()->SetAtIndex(2, 27);

    auto integrator1_xc = GetMutableContinuousState(integrator1());
    ASSERT_NE(nullptr, integrator1_xc);
    integrator1_xc->get_mutable_state()->SetAtIndex(0, 81);
    integrator1_xc->get_mutable_state()->SetAtIndex(1, 243);
    integrator1_xc->get_mutable_state()->SetAtIndex(2, 729);
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>* GetMutableContinuousState(
      const System<double>* system) {
    return diagram_->GetMutableSubsystemState(context_.get(), system)
        ->continuous_state.get();
  }

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

  void AttachInputs() {
    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));
  }

  Integrator<double>* integrator0() { return diagram_->integrator0(); }
  Integrator<double>* integrator1() { return diagram_->integrator1(); }

  const int kLength = 3;

  std::unique_ptr<ExampleDiagram> diagram_;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram exports the correct topology.
TEST_F(DiagramTest, Topology) {
  ASSERT_EQ(kLength, diagram_->get_num_input_ports());
  for (const auto& descriptor : diagram_->get_input_ports()) {
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kInputPort, descriptor.get_face());
    EXPECT_EQ(kLength, descriptor.get_size());
    EXPECT_EQ(kInheritedSampling, descriptor.get_sampling());
  }

  ASSERT_EQ(kLength, diagram_->get_num_output_ports());
  for (const auto& descriptor : diagram_->get_output_ports()) {
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kOutputPort, descriptor.get_face());
    EXPECT_EQ(kLength, descriptor.get_size());
  }

  // The adder output ports have inherited sampling.
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(0).get_sampling());
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(1).get_sampling());
  // The integrator output port has continuous sampling.
  EXPECT_EQ(kContinuousSampling, diagram_->get_output_port(2).get_sampling());
}

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {
  AttachInputs();
  diagram_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(kLength, output_->get_num_ports());
  ExpectDefaultOutputs();
}

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
      diagram_->GetSubsystemDerivatives(*derivatives, integrator0());
  EXPECT_EQ(1 + 8, integrator0_xcdot.get_state().GetAtIndex(0));
  EXPECT_EQ(2 + 16, integrator0_xcdot.get_state().GetAtIndex(1));
  EXPECT_EQ(4 + 32, integrator0_xcdot.get_state().GetAtIndex(2));

  // The derivative of the second integrator is the state of the first.
  const ContinuousState<double>& integrator1_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator1());
  EXPECT_EQ(3, integrator1_xcdot.get_state().GetAtIndex(0));
  EXPECT_EQ(9, integrator1_xcdot.get_state().GetAtIndex(1));
  EXPECT_EQ(27, integrator1_xcdot.get_state().GetAtIndex(2));
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

  auto next_input_0 = std::make_unique<BasicVector<double>>(kLength);
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

class DiagramOfDiagramsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    subdiagram0_ = std::make_unique<ExampleDiagram>(kLength);
    subdiagram1_ = std::make_unique<ExampleDiagram>(kLength);

    DiagramBuilder<double> builder;
    // Hook up the two diagrams in portwise series.
    for (int i = 0; i < 3; i++) {
      builder.ExportInput(subdiagram0_->get_input_port(i));
      builder.Connect(subdiagram0_->get_output_port(i),
                      subdiagram1_->get_input_port(i));
      builder.ExportOutput(subdiagram1_->get_output_port(i));
    }

    diagram_ = builder.Build();

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_.reset(new BasicVector<double>(std::vector<double>{8}));
    input1_.reset(new BasicVector<double>(std::vector<double>{64}));
    input2_.reset(new BasicVector<double>(std::vector<double>{512}));

    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));

    // Initialize the integrator states.
    ContextBase<double>* d0_context = diagram_->GetMutableSubsystemContext(
        context_.get(), subdiagram0_.get());
    ContextBase<double>* d1_context = diagram_->GetMutableSubsystemContext(
        context_.get(), subdiagram1_.get());

    State<double>* integrator0_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator0());
    integrator0_x->continuous_state->get_mutable_state()->SetAtIndex(0, 3);

    State<double>* integrator1_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator1());
    integrator1_x->continuous_state->get_mutable_state()->SetAtIndex(0, 9);

    State<double>* integrator2_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator0());
    integrator2_x->continuous_state->get_mutable_state()->SetAtIndex(0, 27);

    State<double>* integrator3_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator1());
    integrator3_x->continuous_state->get_mutable_state()->SetAtIndex(0, 81);
  }

  const int kLength = 1;

  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<ExampleDiagram> subdiagram0_;
  std::unique_ptr<ExampleDiagram> subdiagram1_;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that a diagram composed of diagrams can be evaluated.
TEST_F(DiagramOfDiagramsTest, EvalOutput) {
  diagram_->EvalOutput(*context_, output_.get());
  // The outputs of subsystem0_ are:
  //   output0 = 8 + 64 + 512 = 584
  //   output1 = output0 + 8 + 64 = 656
  //   output2 = 9 (state of integrator1_)

  // So, the outputs of subsytem1_, and thus of the whole diagram, are:
  //   output0 = 584 + 656 + 9 = 1249
  //   output1 = output0 + 584 + 656 = 2489
  //   output2 = 81 (state of integrator1_)
  EXPECT_EQ(1249, output_->get_vector_data(0)->get_value().x());
  EXPECT_EQ(2489, output_->get_vector_data(1)->get_value().x());
  EXPECT_EQ(81, output_->get_vector_data(2)->get_value().x());
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
    builder.BuildInto(this);
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

// PublishingSystem has an input port for a single double. It publishes that
// double to a function provided in the constructor.
class PublishingSystem : public LeafSystem<double> {
 public:
  explicit PublishingSystem(std::function<void(int)> callback)
      : callback_(callback) {
    this->DeclareInputPort(kVectorValued, 1, kInheritedSampling);
  }

  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override {}

 protected:
  void DoPublish(const ContextBase<double>& context) const override {
    CheckValidContext(context);
    callback_(context.get_vector_input(0)->get_value()[0]);
  }

 private:
  std::function<void(int)> callback_;
};

// PublishNumberDiagram publishes a double provided to its constructor.
class PublishNumberDiagram : public Diagram<double> {
 public:
  explicit PublishNumberDiagram(double constant) : Diagram<double>() {
    constant_.reset(new ConstantVectorSource<double>(Vector1d{constant}));
    publisher_.reset(new PublishingSystem([this](double v) { this->set(v); }));

    DiagramBuilder<double> builder;
    builder.Connect(constant_->get_output_port(0),
                    publisher_->get_input_port(0));
    builder.BuildInto(this);
  }

  double get() const { return published_value_; }

 private:
  void set(double value) { published_value_ = value; }

  std::unique_ptr<ConstantVectorSource<double>> constant_;
  std::unique_ptr<PublishingSystem> publisher_;
  double published_value_ = 0;
};

GTEST_TEST(DiagramPublishTest, Publish) {
  PublishNumberDiagram publishing_diagram(42.0);
  EXPECT_EQ(0, publishing_diagram.get());
  auto context = publishing_diagram.CreateDefaultContext();
  publishing_diagram.Publish(*context);
  EXPECT_EQ(42.0, publishing_diagram.get());
}

}  // namespace
}  // namespace systems
}  // namespace drake
