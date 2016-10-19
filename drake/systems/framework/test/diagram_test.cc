#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/gain.h"
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
                    integrator0_->get_input_port(0));
    builder.Connect(integrator0_->get_output_port(0),
                    integrator1_->get_input_port(0));

    builder.ExportInput(adder0_->get_input_port(0));
    builder.ExportInput(adder0_->get_input_port(1));
    builder.ExportInput(adder1_->get_input_port(1));
    builder.ExportOutput(adder1_->get_output_port());
    builder.ExportOutput(adder2_->get_output_port());
    builder.ExportOutput(integrator1_->get_output_port(0));

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
    Eigen::Vector3d expected_output0;
    expected_output0 << 1 + 8 + 64, 2 + 16 + 128, 4 + 32 + 256;  // B

    Eigen::Vector3d expected_output1;
    expected_output1 << 1 + 8, 2 + 16, 4 + 32;  // A
    expected_output1 += expected_output0;       // A + B

    Eigen::Vector3d expected_output2;
    expected_output2 << 81, 243, 729;  // state of integrator1_

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
    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));
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
  for (const auto& descriptor : diagram_->get_input_ports()) {
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kInputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
    EXPECT_EQ(kInheritedSampling, descriptor.get_sampling());
  }

  ASSERT_EQ(kSize, diagram_->get_num_output_ports());
  for (const auto& descriptor : diagram_->get_output_ports()) {
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kOutputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
  }

  // The adder output ports have inherited sampling.
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(0).get_sampling());
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(1).get_sampling());
  // The integrator output port has continuous sampling.
  EXPECT_EQ(kContinuousSampling, diagram_->get_output_port(2).get_sampling());

  // The diagram has direct feedthrough.
  EXPECT_TRUE(diagram_->has_any_direct_feedthrough());
}

TEST_F(DiagramTest, Path) {
  const std::string path = adder0()->GetPath();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);
}

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {
  AttachInputs();
  diagram_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(kSize, output_->get_num_ports());
  ExpectDefaultOutputs();
}

TEST_F(DiagramTest, EvalTimeDerivatives) {
  AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();

  diagram_->EvalTimeDerivatives(*context_, derivatives.get());

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

  auto next_input_0 = std::make_unique<BasicVector<double>>(kSize);
  next_input_0->get_mutable_value() << 3, 6, 9;
  clone->SetInputPort(0, MakeInput(std::move(next_input_0)));

  // Recompute the output and check the values.
  diagram_->EvalOutput(*clone, output_.get());

  Eigen::Vector3d expected_output0;
  expected_output0 << 3 + 8 + 64, 6 + 16 + 128, 9 + 32 + 256;  // B
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Eigen::Vector3d expected_output1;
  expected_output1 << 3 + 8, 6 + 16, 9 + 32;  // A
  expected_output1 += expected_output0;       // A + B
  const BasicVector<double>* output1 = output_->get_vector_data(1);
  ASSERT_TRUE(output1 != nullptr);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
  EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

  // Check that the context that was cloned is unaffected.
  diagram_->EvalOutput(*context_, output_.get());
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

    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));

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
  context->SetInputPort(0, MakeInput(std::move(vec)));

  plus_seven.EvalOutput(*context, output.get());

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
    this->DeclareInputPort(kVectorValued, 1, kInheritedSampling);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

 protected:
  void DoPublish(const Context<double>& context) const override {
    CheckValidContext(context);
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
    integrator_builder.ExportInput(integrator_->get_input_port(0));
    integrator_builder.ExportOutput(integrator_->get_output_port(0));
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
  EXPECT_FALSE(diagram.has_any_direct_feedthrough());
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
};

// A minimal system that has second-order state.
class SecondOrderStateSystem : public LeafSystem<double> {
 public:
  SecondOrderStateSystem() {
    DeclareInputPort(kVectorValued, 1, kContinuousSampling);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

  SecondOrderStateVector* x(Context<double>* context) const {
    return dynamic_cast<SecondOrderStateVector*>(
        context->get_mutable_continuous_state_vector());
  }

 protected:
  std::unique_ptr<ContinuousState<double>> AllocateContinuousState()
      const override {
    return std::make_unique<ContinuousState<double>>(
        std::make_unique<SecondOrderStateVector>(), 1 /* num_q */,
        1 /* num_v */, 0 /* num_z */);
  }

  // qdot = 2 * v.
  void DoMapVelocityToConfigurationDerivatives(
      const Context<double>& context,
      const Eigen::Ref<const VectorX<double>>& generalized_velocity,
      VectorBase<double>* configuration_derivatives) const override {
    configuration_derivatives->SetAtIndex(
        0, 2 * generalized_velocity[0]);
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

// Tests that MapVelocityToConfigurationDerivatives recursively invokes
// MapVelocityToConfigurationDerivatives on the constituent systems,
// and preserves placewise correspondence.
GTEST_TEST(SecondOrderStateTest, MapVelocityToConfigurationDerivatives) {
  SecondOrderStateDiagram diagram;
  std::unique_ptr<Context<double>> context = diagram.CreateDefaultContext();
  diagram.x(context.get(), diagram.sys1())->set_v(13);
  diagram.x(context.get(), diagram.sys2())->set_v(17);

  BasicVector<double> configuration_derivatives(2);
  const VectorBase<double>& v =
      context->get_continuous_state()->get_generalized_velocity();
  diagram.MapVelocityToConfigurationDerivatives(*context, v,
                                                &configuration_derivatives);

  // The order of these derivatives is arbitrary, so this test is brittle.
  // TODO(david-german-tri): Use UnorderedElementsAre once gmock is available
  // in the superbuild. https://github.com/RobotLocomotion/drake/issues/3133
  EXPECT_EQ(configuration_derivatives.GetAtIndex(0), 34);
  EXPECT_EQ(configuration_derivatives.GetAtIndex(1), 26);
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

}  // namespace
}  // namespace systems
}  // namespace drake
