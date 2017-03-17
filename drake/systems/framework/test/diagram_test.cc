#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

/// ExampleDiagram has the following structure:
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
/// integrator1_: A              -> C
/// integrator2_: C              -> output 2
class ExampleDiagram : public Diagram<double> {
 public:
  explicit ExampleDiagram(int size) {
    DiagramBuilder<double> builder;

    adder0_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder0_->set_name("adder0");
    adder1_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder1_->set_name("adder1");
    adder2_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder2_->set_name("adder2");

    integrator0_ = builder.AddSystem<Integrator<double>>(size);
    integrator0_->set_name("integrator0");
    integrator1_ = builder.AddSystem<Integrator<double>>(size);
    integrator1_->set_name("integrator1");

    builder.Connect(adder0_->get_output_port(), adder1_->get_input_port(0));
    builder.Connect(adder0_->get_output_port(), adder2_->get_input_port(0));
    builder.Connect(adder1_->get_output_port(), adder2_->get_input_port(1));

    builder.Connect(adder0_->get_output_port(),
                    integrator0_->get_input_port());
    builder.Connect(integrator0_->get_output_port(),
                    integrator1_->get_input_port());

    builder.ExportInput(adder0_->get_input_port(0));
    builder.ExportInput(adder0_->get_input_port(1));
    builder.ExportInput(adder1_->get_input_port(1));
    builder.ExportOutput(adder1_->get_output_port());
    builder.ExportOutput(adder2_->get_output_port());
    builder.ExportOutput(integrator1_->get_output_port());

    builder.BuildInto(this);
  }

  Adder<double>* adder0() { return adder0_; }
  Adder<double>* adder1() { return adder1_; }
  Adder<double>* adder2() { return adder2_; }
  Integrator<double>* integrator0() { return integrator0_; }
  Integrator<double>* integrator1() { return integrator1_; }

 private:
  Adder<double>* adder0_ = nullptr;
  Adder<double>* adder1_ = nullptr;
  Adder<double>* adder2_ = nullptr;

  Integrator<double>* integrator0_ = nullptr;
  Integrator<double>* integrator1_ = nullptr;
};

class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    diagram_ = std::make_unique<ExampleDiagram>(kSize);
    diagram_->set_name("Unicode Snowman's Favorite Diagram!!1!☃!");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_ = BasicVector<double>::Make({1, 2, 4});
    input1_ = BasicVector<double>::Make({8, 16, 32});
    input2_ = BasicVector<double>::Make({64, 128, 256});

    // Initialize the integrator states.
    auto integrator0_xc = GetMutableContinuousState(integrator0());
    ASSERT_TRUE(integrator0_xc != nullptr);
    integrator0_xc->get_mutable_vector()->SetAtIndex(0, 3);
    integrator0_xc->get_mutable_vector()->SetAtIndex(1, 9);
    integrator0_xc->get_mutable_vector()->SetAtIndex(2, 27);

    auto integrator1_xc = GetMutableContinuousState(integrator1());
    ASSERT_TRUE(integrator1_xc != nullptr);
    integrator1_xc->get_mutable_vector()->SetAtIndex(0, 81);
    integrator1_xc->get_mutable_vector()->SetAtIndex(1, 243);
    integrator1_xc->get_mutable_vector()->SetAtIndex(2, 729);
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>* GetMutableContinuousState(
      const System<double>* system) {
    return diagram_->GetMutableSubsystemState(context_.get(), system)
        ->get_mutable_continuous_state();
  }

  // Asserts that output_ is what it should be for the default values
  // of input0_, input1_, and input2_.
  void ExpectDefaultOutputs() {
    Eigen::Vector3d expected_output0(
        1 + 8 + 64,
        2 + 16 + 128,
        4 + 32 + 256);  // B

    Eigen::Vector3d expected_output1(
        1 + 8,
        2 + 16,
        4 + 32);  // A
    expected_output1 += expected_output0;       // A + B

    Eigen::Vector3d expected_output2(81, 243, 729);  // state of integrator1_

    const BasicVector<double>* output0 = output_->get_vector_data(0);
    ASSERT_TRUE(output0 != nullptr);
    EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
    EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
    EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

    const BasicVector<double>* output1 = output_->get_vector_data(1);
    ASSERT_TRUE(output1 != nullptr);
    EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
    EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
    EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

    const BasicVector<double>* output2 = output_->get_vector_data(2);
    ASSERT_TRUE(output2 != nullptr);
    EXPECT_EQ(expected_output2[0], output2->get_value()[0]);
    EXPECT_EQ(expected_output2[1], output2->get_value()[1]);
    EXPECT_EQ(expected_output2[2], output2->get_value()[2]);
  }

  void AttachInputs() {
    context_->FixInputPort(0, std::move(input0_));
    context_->FixInputPort(1, std::move(input1_));
    context_->FixInputPort(2, std::move(input2_));
  }

  Adder<double>* adder0() { return diagram_->adder0(); }
  Integrator<double>* integrator0() { return diagram_->integrator0(); }
  Integrator<double>* integrator1() { return diagram_->integrator1(); }

  const int kSize = 3;

  std::unique_ptr<ExampleDiagram> diagram_;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram exports the correct topology.
TEST_F(DiagramTest, Topology) {
  ASSERT_EQ(kSize, diagram_->get_num_input_ports());
  for (int i = 0; i < kSize; ++i) {
    const auto& descriptor = diagram_->get_input_port(i);
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kSize, descriptor.size());
  }

  ASSERT_EQ(kSize, diagram_->get_num_output_ports());
  for (int i = 0; i < kSize; ++i) {
    const auto& descriptor = diagram_->get_output_port(i);
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kSize, descriptor.size());
  }

  // The diagram has direct feedthrough.
  EXPECT_TRUE(diagram_->HasAnyDirectFeedthrough());
  // Specifically, outputs 0 and 1 have direct feedthrough, but not output 2.
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(0));
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(1));
  EXPECT_FALSE(diagram_->HasDirectFeedthrough(2));
}

TEST_F(DiagramTest, Path) {
  const std::string path = adder0()->GetPath();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);
}

// Tests that both variants of GetMutableSubsystemState do what they say on
// the tin.
TEST_F(DiagramTest, GetMutableSubsystemState) {
  State<double>* state_from_context = diagram_->GetMutableSubsystemState(
      context_.get(), diagram_->integrator0());
  ASSERT_NE(nullptr, state_from_context);
  State<double>* state_from_state = diagram_->GetMutableSubsystemState(
      context_->get_mutable_state(), diagram_->integrator0());
  ASSERT_NE(nullptr, state_from_state);

  EXPECT_EQ(state_from_context, state_from_state);
  const ContinuousState<double>& xc =
      *state_from_context->get_continuous_state();
  EXPECT_EQ(3, xc[0]);
  EXPECT_EQ(9, xc[1]);
  EXPECT_EQ(27, xc[2]);
}
// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, CalcOutput) {
  AttachInputs();
  diagram_->CalcOutput(*context_, output_.get());

  ASSERT_EQ(kSize, output_->get_num_ports());
  ExpectDefaultOutputs();
}

TEST_F(DiagramTest, CalcTimeDerivatives) {
  AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();

  diagram_->CalcTimeDerivatives(*context_, derivatives.get());

  ASSERT_EQ(6, derivatives->size());
  ASSERT_EQ(0, derivatives->get_generalized_position().size());
  ASSERT_EQ(0, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(6, derivatives->get_misc_continuous_state().size());

  // The derivative of the first integrator is A.
  const ContinuousState<double>* integrator0_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator0());
  ASSERT_TRUE(integrator0_xcdot != nullptr);
  EXPECT_EQ(1 + 8, integrator0_xcdot->get_vector().GetAtIndex(0));
  EXPECT_EQ(2 + 16, integrator0_xcdot->get_vector().GetAtIndex(1));
  EXPECT_EQ(4 + 32, integrator0_xcdot->get_vector().GetAtIndex(2));

  // The derivative of the second integrator is the state of the first.
  const ContinuousState<double>* integrator1_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator1());
  ASSERT_TRUE(integrator1_xcdot != nullptr);
  EXPECT_EQ(3, integrator1_xcdot->get_vector().GetAtIndex(0));
  EXPECT_EQ(9, integrator1_xcdot->get_vector().GetAtIndex(1));
  EXPECT_EQ(27, integrator1_xcdot->get_vector().GetAtIndex(2));
}

// Tests the AllocateInput logic.
TEST_F(DiagramTest, AllocateInputs) {
  auto context = diagram_->CreateDefaultContext();

  for (int port = 0; port < 3; port++) {
    const BasicVector<double>* vec = diagram_->EvalVectorInput(*context, port);
    EXPECT_EQ(vec, nullptr);
  }

  diagram_->AllocateFreestandingInputs(context.get());

  for (int port = 0; port < 3; port++) {
    const BasicVector<double>* vec = diagram_->EvalVectorInput(*context, port);
    EXPECT_NE(vec, nullptr);
    EXPECT_EQ(vec->size(), kSize);
  }
}

/// Tests that a diagram can be transmogrified to AutoDiffXd.
TEST_F(DiagramTest, ToAutoDiffXd) {
  std::unique_ptr<System<AutoDiffXd>> ad_diagram =
      System<double>::ToAutoDiffXd(*diagram_);
  std::unique_ptr<Context<AutoDiffXd>> context =
      ad_diagram->CreateDefaultContext();
  std::unique_ptr<SystemOutput<AutoDiffXd>> output =
      ad_diagram->AllocateOutput(*context);

  // Set up some inputs, computing gradients with respect to every other input.
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
/// integrator1_: A              -> C
/// integrator2_: C              -> output 2
  auto input0 = std::make_unique<BasicVector<AutoDiffXd>>(3);
  auto input1 = std::make_unique<BasicVector<AutoDiffXd>>(3);
  auto input2 = std::make_unique<BasicVector<AutoDiffXd>>(3);
  for (int i = 0; i < 3; ++i) {
    (*input0)[i].value() = 1 + 0.1 * i;
    (*input0)[i].derivatives() = Eigen::VectorXd::Unit(9, i);
    (*input1)[i].value() = 2 + 0.2 * i;
    (*input1)[i].derivatives() = Eigen::VectorXd::Unit(9, 3 + i);
    (*input2)[i].value() = 3 + 0.3 * i;
    (*input2)[i].derivatives() = Eigen::VectorXd::Unit(9, 6 + i);
  }
  context->FixInputPort(0, std::move(input0));
  context->FixInputPort(1, std::move(input1));
  context->FixInputPort(2, std::move(input2));

  ad_diagram->CalcOutput(*context, output.get());
  ASSERT_EQ(kSize, output->get_num_ports());

  // Spot-check some values and gradients.
  // A = [1.0 + 2.0, 1.1 + 2.2, 1.2 + 2.4]
  // output0 = B = A + [3.0, 3.3, 3.6]
  const BasicVector<AutoDiffXd>* output0 = output->get_vector_data(0);
  EXPECT_DOUBLE_EQ(1.2 + 2.4 + 3.6, (*output0)[2].value());
  // ∂B[2]/∂input0,1,2[2] is 1.  Other partials are zero.
  for (int i = 0; i < 9; ++i) {
    if (i == 2 || i == 5 || i == 8) {
      EXPECT_EQ(1.0, (*output0)[2].derivatives()[i]);
    } else {
      EXPECT_EQ(0.0, (*output0)[2].derivatives()[i]);
    }
  }
  // output1 = A + B = 2A + [3.0, 3.3, 3.6]
  const BasicVector<AutoDiffXd>* output1 = output->get_vector_data(1);
  EXPECT_DOUBLE_EQ(2 * (1.1 + 2.2) + 3.3, (*output1)[1].value());
  // ∂B[1]/∂input0,1[1] is 2. ∂B[1]/∂input2[1] is 1.  Other partials are zero.
  for (int i = 0; i < 9; ++i) {
    if (i == 1 || i == 4) {
      EXPECT_EQ(2.0, (*output1)[1].derivatives()[i]);
    } else if (i == 7) {
      EXPECT_EQ(1.0, (*output1)[1].derivatives()[i]);
    } else {
      EXPECT_EQ(0.0, (*output1)[1].derivatives()[i]);
    }
  }
}

// Tests that the same diagram can be evaluated into the same output with
// different contexts interchangeably.
TEST_F(DiagramTest, Clone) {
  context_->FixInputPort(0, std::move(input0_));
  context_->FixInputPort(1, std::move(input1_));
  context_->FixInputPort(2, std::move(input2_));

  // Compute the output with the default inputs and sanity-check it.
  diagram_->CalcOutput(*context_, output_.get());
  ExpectDefaultOutputs();

  // Create a clone of the context and change an input.
  auto clone = context_->Clone();

  auto next_input_0 = std::make_unique<BasicVector<double>>(kSize);
  next_input_0->get_mutable_value() << 3, 6, 9;
  clone->FixInputPort(0, std::move(next_input_0));

  // Recompute the output and check the values.
  diagram_->CalcOutput(*clone, output_.get());

  Eigen::Vector3d expected_output0(
      3 + 8 + 64,
      6 + 16 + 128,
      9 + 32 + 256);  // B
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Eigen::Vector3d expected_output1(
      3 + 8,
      6 + 16,
      9 + 32);  // A
  expected_output1 += expected_output0;       // A + B
  const BasicVector<double>* output1 = output_->get_vector_data(1);
  ASSERT_TRUE(output1 != nullptr);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
  EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

  // Check that the context that was cloned is unaffected.
  diagram_->CalcOutput(*context_, output_.get());
  ExpectDefaultOutputs();
}

// Tests that, when asked for the state derivatives of Systems that are
// stateless, Diagram returns an empty state.
TEST_F(DiagramTest, DerivativesOfStatelessSystemAreEmpty) {
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();
  EXPECT_EQ(0,
            diagram_->GetSubsystemDerivatives(*derivatives, adder0())->size());
}

class DiagramOfDiagramsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    DiagramBuilder<double> builder;
    subdiagram0_ = builder.AddSystem<ExampleDiagram>(kSize);
    subdiagram0_->set_name("subdiagram0");
    subdiagram1_ = builder.AddSystem<ExampleDiagram>(kSize);
    subdiagram1_->set_name("subdiagram1");

    // Hook up the two diagrams in portwise series.
    for (int i = 0; i < 3; i++) {
      builder.ExportInput(subdiagram0_->get_input_port(i));
      builder.Connect(subdiagram0_->get_output_port(i),
                      subdiagram1_->get_input_port(i));
      builder.ExportOutput(subdiagram1_->get_output_port(i));
    }

    diagram_ = builder.Build();
    diagram_->set_name("DiagramOfDiagrams");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_ = BasicVector<double>::Make({8});
    input1_ = BasicVector<double>::Make({64});
    input2_ = BasicVector<double>::Make({512});

    context_->FixInputPort(0, std::move(input0_));
    context_->FixInputPort(1, std::move(input1_));
    context_->FixInputPort(2, std::move(input2_));

    // Initialize the integrator states.
    Context<double>* d0_context =
        diagram_->GetMutableSubsystemContext(context_.get(), subdiagram0_);
    Context<double>* d1_context =
        diagram_->GetMutableSubsystemContext(context_.get(), subdiagram1_);

    State<double>* integrator0_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator0());
    integrator0_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 3);

    State<double>* integrator1_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator1());
    integrator1_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 9);

    State<double>* integrator2_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator0());
    integrator2_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 27);

    State<double>* integrator3_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator1());
    integrator3_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 81);
  }

  const int kSize = 1;

  std::unique_ptr<Diagram<double>> diagram_ = nullptr;
  ExampleDiagram* subdiagram0_ = nullptr;
  ExampleDiagram* subdiagram1_ = nullptr;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that a diagram composed of diagrams can be evaluated.
TEST_F(DiagramOfDiagramsTest, EvalOutput) {
  diagram_->CalcOutput(*context_, output_.get());
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
    DiagramBuilder<double> builder;

    constant_ = builder.AddSystem<ConstantVectorSource>(Vector1d{constant});
    adder_ = builder.AddSystem<Adder>(2 /* inputs */, 1 /* size */);

    builder.Connect(constant_->get_output_port(), adder_->get_input_port(1));
    builder.ExportInput(adder_->get_input_port(0));
    builder.ExportOutput(adder_->get_output_port());
    builder.BuildInto(this);
  }

 private:
  Adder<double>* adder_ = nullptr;
  ConstantVectorSource<double>* constant_ = nullptr;
};

GTEST_TEST(DiagramSubclassTest, TwelvePlusSevenIsNineteen) {
  AddConstantDiagram plus_seven(7.0);
  auto context = plus_seven.CreateDefaultContext();
  auto output = plus_seven.AllocateOutput(*context);
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(output != nullptr);

  auto vec = std::make_unique<BasicVector<double>>(1 /* size */);
  vec->get_mutable_value() << 12.0;
  context->FixInputPort(0, std::move(vec));

  plus_seven.CalcOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(1, output_vector->size());
  EXPECT_EQ(19.0, output_vector->get_value().x());
}

// PublishingSystem has an input port for a single double. It publishes that
// double to a function provided in the constructor.
class PublishingSystem : public LeafSystem<double> {
 public:
  explicit PublishingSystem(std::function<void(int)> callback)
      : callback_(callback) {
    this->DeclareInputPort(kVectorValued, 1);
  }

 protected:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoPublish(const Context<double>& context) const override {
    callback_(this->EvalVectorInput(context, 0)->get_value()[0]);
  }

 private:
  std::function<void(int)> callback_;
};

// PublishNumberDiagram publishes a double provided to its constructor.
class PublishNumberDiagram : public Diagram<double> {
 public:
  explicit PublishNumberDiagram(double constant) : Diagram<double>() {
    DiagramBuilder<double> builder;

    constant_ =
        builder.AddSystem<ConstantVectorSource<double>>(Vector1d{constant});
    publisher_ =
        builder.AddSystem<PublishingSystem>([this](double v) { this->set(v); });

    builder.Connect(constant_->get_output_port(),
                    publisher_->get_input_port(0));
    builder.BuildInto(this);
  }

  double get() const { return published_value_; }

 private:
  void set(double value) { published_value_ = value; }

  ConstantVectorSource<double>* constant_ = nullptr;
  PublishingSystem* publisher_ = nullptr;
  double published_value_ = 0;
};

GTEST_TEST(DiagramPublishTest, Publish) {
  PublishNumberDiagram publishing_diagram(42.0);
  EXPECT_EQ(0, publishing_diagram.get());
  auto context = publishing_diagram.CreateDefaultContext();
  publishing_diagram.Publish(*context);
  EXPECT_EQ(42.0, publishing_diagram.get());
}

// FeedbackDiagram is a diagram containing a feedback loop of two
// constituent diagrams, an Integrator and a Gain. The Integrator is not
// direct-feedthrough, so there is no algebraic loop.
class FeedbackDiagram : public Diagram<double> {
 public:
  FeedbackDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;

    DiagramBuilder<double> integrator_builder;
    integrator_ = integrator_builder.AddSystem<Integrator>(1 /* size */);
    integrator_builder.ExportInput(integrator_->get_input_port());
    integrator_builder.ExportOutput(integrator_->get_output_port());
    integrator_diagram_ = builder.AddSystem(integrator_builder.Build());

    DiagramBuilder<double> gain_builder;
    gain_ = gain_builder.AddSystem<Gain>(1.0 /* gain */, 1 /* length */);
    gain_builder.ExportInput(gain_->get_input_port());
    gain_builder.ExportOutput(gain_->get_output_port());
    gain_diagram_ = builder.AddSystem(gain_builder.Build());

    builder.Connect(*integrator_diagram_, *gain_diagram_);
    builder.Connect(*gain_diagram_, *integrator_diagram_);
    builder.BuildInto(this);
  }

 private:
  Integrator<double>* integrator_ = nullptr;
  Gain<double>* gain_ = nullptr;
  Diagram<double>* integrator_diagram_ = nullptr;
  Diagram<double>* gain_diagram_ = nullptr;
};

// Tests that since there are no outputs, there is no direct feedthrough.
GTEST_TEST(FeedbackDiagramTest, HasDirectFeedthrough) {
  FeedbackDiagram diagram;
  EXPECT_FALSE(diagram.HasAnyDirectFeedthrough());
}

// Tests that a FeedbackDiagram's context can be deleted without accessing
// already-freed memory. https://github.com/RobotLocomotion/drake/issues/3349
GTEST_TEST(FeedbackDiagramTest, DeletionIsMemoryClean) {
  FeedbackDiagram diagram;
  auto context = diagram.CreateDefaultContext();
  EXPECT_NO_THROW(context.reset());
}

// A vector with a scalar configuration and scalar velocity.
class SecondOrderStateVector : public BasicVector<double> {
 public:
  SecondOrderStateVector() : BasicVector<double>(2) {}

  double q() const { return GetAtIndex(0); }
  double v() const { return GetAtIndex(1); }

  void set_q(double q) { SetAtIndex(0, q); }
  void set_v(double v) { SetAtIndex(1, v); }

 protected:
  SecondOrderStateVector* DoClone() const override {
    return new SecondOrderStateVector;
  }
};

// A minimal system that has second-order state.
class SecondOrderStateSystem : public LeafSystem<double> {
 public:
  SecondOrderStateSystem() { DeclareInputPort(kVectorValued, 1); }

  SecondOrderStateVector* x(Context<double>* context) const {
    return dynamic_cast<SecondOrderStateVector*>(
        context->get_mutable_continuous_state_vector());
  }

 protected:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  std::unique_ptr<ContinuousState<double>> AllocateContinuousState()
      const override {
    return std::make_unique<ContinuousState<double>>(
        std::make_unique<SecondOrderStateVector>(), 1 /* num_q */,
        1 /* num_v */, 0 /* num_z */);
  }

  // qdot = 2 * v.
  void DoMapVelocityToQDot(
      const Context<double>& context,
      const Eigen::Ref<const VectorX<double>>& generalized_velocity,
      VectorBase<double>* qdot) const override {
    qdot->SetAtIndex(
        0, 2 * generalized_velocity[0]);
  }

  // v = 1/2 * qdot.
  void DoMapQDotToVelocity(
      const Context<double>& context,
      const Eigen::Ref<const VectorX<double>>& qdot,
      VectorBase<double>* generalized_velocity) const override {
    generalized_velocity->SetAtIndex(
        0, 0.5 * qdot[0]);
  }
};

// A diagram that has second-order state.
class SecondOrderStateDiagram : public Diagram<double> {
 public:
  SecondOrderStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    sys1_ = builder.template AddSystem<SecondOrderStateSystem>();
    sys2_ = builder.template AddSystem<SecondOrderStateSystem>();
    builder.ExportInput(sys1_->get_input_port(0));
    builder.ExportInput(sys2_->get_input_port(0));
    builder.BuildInto(this);
  }

  SecondOrderStateSystem* sys1() { return sys1_; }
  SecondOrderStateSystem* sys2() { return sys2_; }

  // Returns the state of the given subsystem.
  SecondOrderStateVector* x(Context<double>* context,
                            const SecondOrderStateSystem* subsystem) {
    Context<double>* subsystem_context =
        GetMutableSubsystemContext(context, subsystem);
    return subsystem->x(subsystem_context);
  }

 private:
  SecondOrderStateSystem* sys1_ = nullptr;
  SecondOrderStateSystem* sys2_ = nullptr;
};

// Tests that MapVelocityToQDot and MapQDotToVelocity recursively invoke
// MapVelocityToQDot (resp. MapQDotToVelocity) on the constituent systems,
// and preserve placewise correspondence.
GTEST_TEST(SecondOrderStateTest, MapVelocityToQDot) {
  SecondOrderStateDiagram diagram;
  std::unique_ptr<Context<double>> context = diagram.CreateDefaultContext();
  diagram.x(context.get(), diagram.sys1())->set_v(13);
  diagram.x(context.get(), diagram.sys2())->set_v(17);

  BasicVector<double> qdot(2);
  const VectorBase<double>& v = context->get_continuous_state()->
                                         get_generalized_velocity();
  diagram.MapVelocityToQDot(*context, v, &qdot);

  // The order of these derivatives is arbitrary, so this test is brittle.
  // TODO(david-german-tri): Use UnorderedElementsAre once gmock is available
  // in the superbuild. https://github.com/RobotLocomotion/drake/issues/3133
  EXPECT_EQ(qdot.GetAtIndex(0), 34);
  EXPECT_EQ(qdot.GetAtIndex(1), 26);

  // Now map the configuration derivatives back to v.
  // TODO(david-german-tri): Address the issue broached immediately above
  // here too.
  BasicVector<double> vmutable(v.size());
  diagram.MapQDotToVelocity(*context, qdot, &vmutable);
  EXPECT_EQ(vmutable.GetAtIndex(0), 17);
  EXPECT_EQ(vmutable.GetAtIndex(1), 13);
}

// Test for GetSystems.
GTEST_TEST(GetSystemsTest, GetSystems) {
  auto diagram = std::make_unique<ExampleDiagram>(2);
  EXPECT_EQ((std::vector<const System<double>*>{
                diagram->adder0(), diagram->adder1(), diagram->adder2(),
                diagram->integrator0(), diagram->integrator1(),
            }),
            diagram->GetSystems());
}

const double kTestPublishPeriod = 19.0;

class TestPublishingSystem : public LeafSystem<double> {
 public:
  TestPublishingSystem() {
    this->DeclarePublishPeriodSec(kTestPublishPeriod);
  }

  ~TestPublishingSystem() override {}


  bool published() { return published_; }

 protected:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoPublish(const Context<double>& context) const override {
    published_ = true;
  }

 private:
  mutable bool published_{false};
};

// A diagram that has discrete state and publishers.
class DiscreteStateDiagram : public Diagram<double> {
 public:
  DiscreteStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    hold1_ = builder.template AddSystem<ZeroOrderHold<double>>(2.0, kSize);
    hold2_ = builder.template AddSystem<ZeroOrderHold<double>>(3.0, kSize);
    publisher_ = builder.template AddSystem<TestPublishingSystem>();
    builder.ExportInput(hold1_->get_input_port());
    builder.ExportInput(hold2_->get_input_port());
    builder.BuildInto(this);
  }

  ZeroOrderHold<double>* hold1() { return hold1_; }
  ZeroOrderHold<double>* hold2() { return hold2_; }
  TestPublishingSystem* publisher() { return publisher_; }

 private:
  const int kSize = 1;
  ZeroOrderHold<double>* hold1_ = nullptr;
  ZeroOrderHold<double>* hold2_ = nullptr;
  TestPublishingSystem* publisher_ = nullptr;
};

class DiscreteStateTest : public ::testing::Test {
 public:
  void SetUp() override {
    context_ = diagram_.CreateDefaultContext();
    context_->FixInputPort(0, BasicVector<double>::Make({17.0}));
    context_->FixInputPort(1, BasicVector<double>::Make({23.0}));
  }

 protected:
  DiscreteStateDiagram diagram_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that the next update time after 0.05 is 2.0.
TEST_F(DiscreteStateTest, CalcNextUpdateTimeHold1) {
  context_->set_time(0.05);
  UpdateActions<double> actions;
  diagram_.CalcNextUpdateTime(*context_, &actions);

  EXPECT_EQ(2.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that the next update time after 5.1 is 6.0.
TEST_F(DiscreteStateTest, CalcNextUpdateTimeHold2) {
  context_->set_time(5.1);
  UpdateActions<double> actions;
  diagram_.CalcNextUpdateTime(*context_, &actions);

  // Even though two subsystems are updating, there is only one update action
  // on the Diagram.
  EXPECT_EQ(6.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that on the 9-second tick, only hold2 latches its inputs. Then, on
// the 12-second tick, both hold1 and hold2 latch their inputs.
TEST_F(DiscreteStateTest, UpdateDiscreteVariables) {
  // Initialize the zero-order holds to different values than their input ports.
  Context<double>* ctx1 =
      diagram_.GetMutableSubsystemContext(context_.get(), diagram_.hold1());
  ctx1->get_mutable_discrete_state(0)->SetAtIndex(0, 1001.0);
  Context<double>* ctx2 =
      diagram_.GetMutableSubsystemContext(context_.get(), diagram_.hold2());
  ctx2->get_mutable_discrete_state(0)->SetAtIndex(0, 1002.0);

  // Allocate the discrete variables.
  std::unique_ptr<DiscreteState<double>> updates =
      diagram_.AllocateDiscreteVariables();

  // Set the time to 8.5, so only hold2 updates.
  context_->set_time(8.5);

  // Request the next update time.
  UpdateActions<double> actions;
  diagram_.CalcNextUpdateTime(*context_, &actions);
  EXPECT_EQ(9.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());

  // Fast forward to 9.0 sec and do the update.
  context_->set_time(9.0);
  diagram_.CalcDiscreteVariableUpdates(*context_,
                                       actions.events[0],
                                       updates.get());
  context_->get_mutable_discrete_state()->SetFrom(*updates);
  EXPECT_EQ(1001.0, ctx1->get_discrete_state(0)->GetAtIndex(0));
  EXPECT_EQ(23.0, ctx2->get_discrete_state(0)->GetAtIndex(0));

  // Restore hold2 to its original value.
  ctx2->get_mutable_discrete_state(0)->SetAtIndex(0, 1002.0);
  // Set the time to 11.5, so both hold1 and hold2 update.
  context_->set_time(11.5);
  diagram_.CalcNextUpdateTime(*context_, &actions);
  EXPECT_EQ(12.0, actions.time);
  // A single update event on the Diagram is expanded to update events on
  // each constituent system.
  ASSERT_EQ(1u, actions.events.size());

  // Fast forward to 12.0 sec and do the update again.
  context_->set_time(12.0);
  diagram_.CalcDiscreteVariableUpdates(*context_,
                                       actions.events[0],
                                       updates.get());
  context_->get_mutable_discrete_state()->SetFrom(*updates);
  EXPECT_EQ(17.0, ctx1->get_discrete_state(0)->GetAtIndex(0));
  EXPECT_EQ(23.0, ctx2->get_discrete_state(0)->GetAtIndex(0));
}

// Tests that a publish action is taken at 19 sec.
TEST_F(DiscreteStateTest, Publish) {
  context_->set_time(18.5);
  UpdateActions<double> actions;
  diagram_.CalcNextUpdateTime(*context_, &actions);

  EXPECT_EQ(19.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kPublishAction, actions.events[0].action);

  // Fast forward to 19.0 sec and do the publish.
  EXPECT_EQ(false, diagram_.publisher()->published());
  context_->set_time(19.0);
  diagram_.Publish(*context_, actions.events[0]);
  // Check that publication occurred.
  EXPECT_EQ(true, diagram_.publisher()->published());
}

class SystemWithAbstractState : public LeafSystem<double> {
 public:
  SystemWithAbstractState(int id, double update_period) : id_(id) {
    DeclarePeriodicUnrestrictedUpdate(update_period, 0);
  }

  ~SystemWithAbstractState() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    std::vector<std::unique_ptr<AbstractValue>> values;
    values.push_back({PackValue<double>(id_)});
    return std::make_unique<AbstractValues>(std::move(values));
  }

  // Abstract state is set to time + id.
  void DoCalcUnrestrictedUpdate(const Context<double>& context,
                                State<double>* state) const override {
    double& state_num = state->get_mutable_abstract_state()
                            ->get_mutable_value(0)
                            .GetMutableValue<double>();
    state_num = id_ + context.get_time();
  }

  int get_id() const { return id_; }

 private:
  int id_{0};
};

class AbstractStateDiagram : public Diagram<double> {
 public:
  AbstractStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    sys0_ = builder.template AddSystem<SystemWithAbstractState>(0, 2.);
    sys1_ = builder.template AddSystem<SystemWithAbstractState>(1, 3.);
    builder.BuildInto(this);
  }

  SystemWithAbstractState* get_mutable_sys0() { return sys0_; }
  SystemWithAbstractState* get_mutable_sys1() { return sys1_; }

 private:
  SystemWithAbstractState* sys0_{nullptr};
  SystemWithAbstractState* sys1_{nullptr};
};

class AbstractStateDiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = diagram_.CreateDefaultContext();
  }

  double get_sys0_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*context_, diagram_.get_mutable_sys0());
    return sys_context.get_abstract_state<double>(0);
  }

  double get_sys1_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*context_, diagram_.get_mutable_sys1());
    return sys_context.get_abstract_state<double>(0);
  }

  AbstractStateDiagram diagram_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(AbstractStateDiagramTest, CalcUnrestrictedUpdate) {
  double time = 1;
  context_->set_time(time);

  // The abstract data should be initialized to their ids.
  EXPECT_EQ(get_sys0_abstract_data_as_double(), 0);
  EXPECT_EQ(get_sys1_abstract_data_as_double(), 1);

  // First action time should be 2 sec, and only sys0 will be updating.
  systems::UpdateActions<double> update_actions;
  diagram_.CalcNextUpdateTime(*context_, &update_actions);
  EXPECT_EQ(update_actions.time, 2);
  EXPECT_EQ(update_actions.events.size(), 1u);
  EXPECT_EQ(update_actions.events.front().action,
      DiscreteEvent<double>::ActionType::kUnrestrictedUpdateAction);

  // Creates a temp state and does unrestricted updates.
  std::unique_ptr<State<double>> x_buf = context_->CloneState();
  diagram_.CalcUnrestrictedUpdate(*context_, update_actions.events.front(),
                                  x_buf.get());

  // The abstract data in the current context should be the same as before.
  EXPECT_EQ(get_sys0_abstract_data_as_double(), 0);
  EXPECT_EQ(get_sys1_abstract_data_as_double(), 1);

  // Swaps in the new state, and the abstract data for sys0 should be updated.
  context_->get_mutable_state()->CopyFrom(*x_buf);
  EXPECT_EQ(get_sys0_abstract_data_as_double(), (time + 0));
  EXPECT_EQ(get_sys1_abstract_data_as_double(), 1);

  // Sets time to 5.5, both system should be updating at 6 sec.
  time = 5.5;
  context_->set_time(time);
  diagram_.CalcNextUpdateTime(*context_, &update_actions);
  EXPECT_EQ(update_actions.time, 6);
  // One action to update all subsystems' state.
  EXPECT_EQ(update_actions.events.size(), 1u);

  diagram_.CalcUnrestrictedUpdate(*context_, update_actions.events.front(),
                                  x_buf.get());
  // Both sys0 and sys1's abstract data should be updated.
  context_->get_mutable_state()->CopyFrom(*x_buf);
  EXPECT_EQ(get_sys0_abstract_data_as_double(), (time + 0));
  EXPECT_EQ(get_sys1_abstract_data_as_double(), (time + 1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
