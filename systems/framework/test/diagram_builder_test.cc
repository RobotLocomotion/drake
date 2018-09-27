#include "drake/systems/framework/diagram_builder.h"

#include <regex>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"

namespace drake {
namespace systems {
namespace {

// Tests ::empty().
GTEST_TEST(DiagramBuilderTest, Empty) {
  DiagramBuilder<double> builder;
  const DiagramBuilder<double>& const_builder = builder;
  EXPECT_TRUE(const_builder.empty());
  builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  EXPECT_FALSE(const_builder.empty());
}

// A special class to distinguish between cycles and algebraic loops. The system
// has one input and two outputs. One output simply "echoes" the input (direct
// feedthrough). The other output merely outputs a const value. That means, the
// system *has* feedthrough, but a cycle in the diagram graph does not imply
// an algebraic loop.
template <typename T>
class ConstAndEcho final : public LeafSystem<T> {
 public:
  ConstAndEcho() : LeafSystem<T>(SystemTypeTag<systems::ConstAndEcho>{}) {
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareVectorOutputPort(BasicVector<T>(1), &ConstAndEcho::CalcEcho);
    this->DeclareVectorOutputPort(BasicVector<T>(1),
                                  &ConstAndEcho::CalcConstant);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit ConstAndEcho(const ConstAndEcho<U>&) : ConstAndEcho() {}

  const systems::InputPort<T>& get_vec_input_port() {
    return this->get_input_port(0);
  }

  const systems::OutputPort<T>& get_echo_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  const systems::OutputPort<T>& get_const_output_port() const {
    return systems::System<T>::get_output_port(1);
  }

  void CalcConstant(const Context<T>& context,
                    BasicVector<T>* const_value) const {
    const_value->get_mutable_value() << 17;
  }

  void CalcEcho(const Context<T>& context, BasicVector<T>* echo) const {
    const BasicVector<T>* input_vector = this->EvalVectorInput(context, 0);
    echo->get_mutable_value() = input_vector->get_value();
  }
};

// Tests that an exception is thrown if the diagram contains an algebraic loop.
GTEST_TEST(DiagramBuilderTest, AlgebraicLoop) {
  DiagramBuilder<double> builder;
  auto adder = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  const std::string name{"adder"};
  adder->set_name(name);
  // Connect the output port to the input port.
  builder.Connect(adder->get_output_port(), adder->get_input_port(0));
  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(), std::runtime_error,
                             "Algebraic loop detected in DiagramBuilder:\n"
                             "\\s+" + name + ":Out\\(0\\).*\n."
                             "\\s*" + name + ":In\\(0\\)");
}

// Tests that a cycle which is not an algebraic loop is recognized as valid.
// The system has direct feedthrough; but, at the port level, it is wired
// without an algebraic loop at the port level.
GTEST_TEST(DiagramBuilderTest, CycleButNoLoopPortLevel) {
  DiagramBuilder<double> builder;

  //    +----------+
  //    |---+  +---|
  // +->| I |->| E |
  // |  |---+  +---|
  // |  |      +---|
  // |  |      | C |--+
  // |  |      +---|  |
  // |  |__________|  |
  // |                |
  // +----------------+
  //
  // The input feeds through to the echo output, but it is the constant output
  // that is connected to input. So, the system has direct feedthrough, the
  // diagram has a cycle at the *system* level, but there is no algebraic loop.

  auto echo = builder.AddSystem<ConstAndEcho>();
  echo->set_name("echo");
  builder.Connect(echo->get_const_output_port(), echo->get_vec_input_port());
  EXPECT_NO_THROW(builder.Build());
}

// Contrasts with CycleButNoLoopPortLevel. In this case, the cycle *does*
// represent an algebraic loop.
GTEST_TEST(DiagramBuilderTest, CycleAtLoopPortLevel) {
  DiagramBuilder<double> builder;

  //    +----------+
  //    |---+  +---|
  // +->| I |->| E |--+
  // |  |---+  +---|  |
  // |  |      +---|  |
  // |  |      | C |  |
  // |  |      +---|  |
  // |  |__________|  |
  // |                |
  // +----------------+
  //
  // The input feeds through to the echo output, but it is the constant output
  // that is connected to input. So, the system has direct feeedthrough, the
  // diagram has a cycle at the *system* level, but there is no algebraic loop.

  auto echo = builder.AddSystem<ConstAndEcho>();
  const std::string name{"echo"};
  echo->set_name(name);
  builder.Connect(echo->get_echo_output_port(), echo->get_vec_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(), std::runtime_error,
                             "Algebraic loop detected in DiagramBuilder:\n"
                             "\\s+" + name + ":Out\\(0\\).*\n."
                             "\\s*" + name + ":In\\(0\\)");
}

// Tests that a cycle which is not an algebraic loop is recognized as valid.
// The cycle contains a system with no direct feedthrough; so the apparent loop
// is broken at the *system* level.
GTEST_TEST(DiagramBuilderTest, CycleButNoAlgebraicLoopSystemLevel) {
  DiagramBuilder<double> builder;

  // Create the following diagram:
  //
  // input --->| 0       |
  //           |   Adder +---> Integrator -|
  //        |->| 1       |                 |---> output
  //        |------------------------------|
  auto adder = builder.AddSystem<Adder>(2 /* inputs */, 1 /* size */);
  adder->set_name("adder");
  auto integrator = builder.AddSystem<Integrator>(1 /* size */);
  integrator->set_name("integrator");

  builder.Connect(integrator->get_output_port(), adder->get_input_port(1));

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that multiple cascaded elements that are not direct-feedthrough
// are buildable.
GTEST_TEST(DiagramBuilderTest, CascadedNonDirectFeedthrough) {
  DiagramBuilder<double> builder;

  auto integrator1 = builder.AddSystem<Integrator>(1 /* size */);
  integrator1->set_name("integrator1");
  auto integrator2 = builder.AddSystem<Integrator>(1 /* size */);
  integrator2->set_name("integrator2");

  builder.Connect(integrator1->get_output_port(),
                  integrator2->get_input_port());

  // There is no algebraic loop, so we should not throw.
  EXPECT_NO_THROW(builder.Build());
}

// Tests that an exception is thrown when building an empty diagram.
GTEST_TEST(DiagramBuilderTest, FinalizeWhenEmpty) {
  DiagramBuilder<double> builder;
  EXPECT_THROW(builder.Build(), std::logic_error);
}

GTEST_TEST(DiagramBuilderTest, SystemsThatAreNotAddedThrow) {
  DiagramBuilder<double> builder;
  Adder<double> adder(1 /* inputs */, 1 /* size */);
  EXPECT_THROW(builder.Connect(adder, adder), std::exception);
  EXPECT_THROW(builder.ExportInput(adder.get_input_port(0)), std::exception);
  EXPECT_THROW(builder.ExportOutput(adder.get_output_port()), std::exception);
}

// Helper class that has one input port, and no output ports.
template <typename T>
class Sink : public LeafSystem<T> {
 public:
  Sink() { this->DeclareInputPort("in", kVectorValued, 1); }
};

// Helper class that has no input port, and one output port.
template <typename T>
class Source : public LeafSystem<T> {
 public:
  Source() { this->DeclareVectorOutputPort("out", &Source<T>::CalcOutput); }
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const {}
};

GTEST_TEST(DiagramBuilderTest, DefaultInputPortNamesAreUniqueTest) {
  DiagramBuilder<double> builder;

  auto sink1 = builder.AddSystem<Sink<double>>();
  auto sink2 = builder.AddSystem<Sink<double>>();

  builder.ExportInput(sink1->get_input_port(0));
  builder.ExportInput(sink2->get_input_port(0));

  // If the port names were not unique, then the build step would throw.
  EXPECT_NO_THROW(builder.Build());
}

GTEST_TEST(DiagramBuilderTest, DefaultOutputPortNamesAreUniqueTest) {
  DiagramBuilder<double> builder;

  auto source1 = builder.AddSystem<Source<double>>();
  auto source2 = builder.AddSystem<Source<double>>();

  builder.ExportOutput(source1->get_output_port(0));
  builder.ExportOutput(source2->get_output_port(0));

  // If the port names were not unique, then the build step would throw.
  EXPECT_NO_THROW(builder.Build());
}

GTEST_TEST(DiagramBuilderTest, DefaultPortNamesAreUniqueTest2) {
  DiagramBuilder<double> builder;

  // This time, we assign system names manually.
  auto sink1 = builder.AddSystem<Sink<double>>();
  sink1->set_name("sink1");
  auto sink2 = builder.AddSystem<Sink<double>>();
  sink2->set_name("sink2");

  auto source1 = builder.AddSystem<Source<double>>();
  source1->set_name("source1");
  auto source2 = builder.AddSystem<Source<double>>();
  source2->set_name("source2");

  const auto sink1_in = builder.ExportInput(sink1->get_input_port(0));
  const auto sink2_in = builder.ExportInput(sink2->get_input_port(0));
  const auto source1_out = builder.ExportOutput(source1->get_output_port(0));
  const auto source2_out = builder.ExportOutput(source2->get_output_port(0));

  auto diagram = builder.Build();

  EXPECT_EQ(diagram->get_input_port(sink1_in).get_name(), "sink1_in");
  EXPECT_EQ(diagram->get_input_port(sink2_in).get_name(), "sink2_in");
  EXPECT_EQ(diagram->get_output_port(source1_out).get_name(), "source1_out");
  EXPECT_EQ(diagram->get_output_port(source2_out).get_name(), "source2_out");
}

GTEST_TEST(DiagramBuilderTest, SetPortNamesTest) {
  DiagramBuilder<double> builder;

  auto sink1 = builder.AddSystem<Sink<double>>();
  auto sink2 = builder.AddSystem<Sink<double>>();
  auto source1 = builder.AddSystem<Source<double>>();
  auto source2 = builder.AddSystem<Source<double>>();

  const auto sink1_in = builder.ExportInput(sink1->get_input_port(0), "sink1");
  const auto sink2_in = builder.ExportInput(sink2->get_input_port(0), "sink2");
  const auto source1_out =
      builder.ExportOutput(source1->get_output_port(0), "source1");
  const auto source2_out =
      builder.ExportOutput(source2->get_output_port(0), "source2");

  auto diagram = builder.Build();

  EXPECT_EQ(diagram->get_input_port(sink1_in).get_name(), "sink1");
  EXPECT_EQ(diagram->get_input_port(sink2_in).get_name(), "sink2");
  EXPECT_EQ(diagram->get_output_port(source1_out).get_name(), "source1");
  EXPECT_EQ(diagram->get_output_port(source2_out).get_name(), "source2");
}

GTEST_TEST(DiagramBuilderTest, DuplicateInputPortNamesThrow) {
  DiagramBuilder<double> builder;

  auto sink1 = builder.AddSystem<Sink<double>>();
  auto sink2 = builder.AddSystem<Sink<double>>();

  builder.ExportInput(sink1->get_input_port(0), "sink");
  builder.ExportInput(sink2->get_input_port(0), "sink");

  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(), std::logic_error,
                              ".*already has an input port named.*");
}

GTEST_TEST(DiagramBuilderTest, DuplicateOutputPortNamesThrow) {
  DiagramBuilder<double> builder;

  auto sink1 = builder.AddSystem<Source<double>>();
  auto sink2 = builder.AddSystem<Source<double>>();

  builder.ExportOutput(sink1->get_output_port(0), "source");
  builder.ExportOutput(sink2->get_output_port(0), "source");

  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(), std::logic_error,
                              ".*already has an output port named.*");
}


// Tests the sole-port based overload of Connect().
class DiagramBuilderSolePortsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    out1_ = builder_.AddSystem<ConstantVectorSource>(Vector1d::Ones());
    out1_->set_name("constant");
    in1_ = builder_.AddSystem<Sink>();
    in1_->set_name("sink");
    in1out1_ = builder_.AddSystem<Gain>(1.0 /* gain */, 1 /* size */);
    in1out1_->set_name("gain");
    in2out1_ = builder_.AddSystem<Adder>(2 /* inputs */, 1 /* size */);
    in2out1_->set_name("adder");
    in1out2_ = builder_.AddSystem<Demultiplexer>(2 /* size */);
    in1out2_->set_name("demux");
  }

  DiagramBuilder<double> builder_;
  ConstantVectorSource<double>* out1_ = nullptr;
  Sink<double>* in1_ = nullptr;
  Gain<double>* in1out1_ = nullptr;
  Adder<double>* in2out1_ = nullptr;
  Demultiplexer<double>* in1out2_ = nullptr;
};

// A diagram of Source->Gain->Sink is successful.
TEST_F(DiagramBuilderSolePortsTest, SourceGainSink) {
  EXPECT_NO_THROW(builder_.Connect(*out1_, *in1out1_));
  EXPECT_NO_THROW(builder_.Connect(*in1out1_, *in1_));
  EXPECT_NO_THROW(builder_.Build());
}

// The cascade synonym also works.
TEST_F(DiagramBuilderSolePortsTest, SourceGainSinkCascade) {
  EXPECT_NO_THROW(builder_.Cascade(*out1_, *in1out1_));
  EXPECT_NO_THROW(builder_.Cascade(*in1out1_, *in1_));
  EXPECT_NO_THROW(builder_.Build());
}

// A diagram of Gain->Source is has too few dest inputs.
TEST_F(DiagramBuilderSolePortsTest, TooFewDestInputs) {
  EXPECT_THROW(builder_.Connect(*in1out1_, *out1_), std::exception);
}

// A diagram of Source->In2out1 is has too many dest inputs.
TEST_F(DiagramBuilderSolePortsTest, TooManyDestInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*out1_, *in2out1_), std::exception);
}

// A diagram of Sink->Gain is has too few src inputs.
TEST_F(DiagramBuilderSolePortsTest, TooFewSrcInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*in1_, *in1out1_), std::exception);
}

// A diagram of Demux->Gain is has too many src inputs.
TEST_F(DiagramBuilderSolePortsTest, TooManySrcInputs) {
  using std::exception;
  EXPECT_THROW(builder_.Connect(*in1out2_, *in1out1_), std::exception);
}

// Test for GetMutableSystems.
GTEST_TEST(DiagramBuilderTest, GetMutableSystems) {
  DiagramBuilder<double> builder;
  auto adder1 = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  adder1->set_name("adder1");
  auto adder2 = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  adder2->set_name("adder2");
  EXPECT_EQ((std::vector<System<double>*>{adder1, adder2}),
            builder.GetMutableSystems());
}

// Tests that the returned exported input / output port id matches the
// number of ExportInput() / ExportOutput() calls.
GTEST_TEST(DiagramBuilderTest, ExportInputOutputIndex) {
  DiagramBuilder<double> builder;
  auto adder1 = builder.AddSystem<Adder>(3 /* inputs */, 1 /* size */);
  adder1->set_name("adder1");
  auto adder2 = builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  adder2->set_name("adder2");

  EXPECT_EQ(builder.ExportInput(
        adder1->get_input_port(0)), 0 /* exported input port id */);
  EXPECT_EQ(builder.ExportInput(
        adder1->get_input_port(1)), 1 /* exported input port id */);

  EXPECT_EQ(builder.ExportOutput(
        adder1->get_output_port()), 0 /* exported output port id */);
  EXPECT_EQ(builder.ExportOutput(
        adder2->get_output_port()), 1 /* exported output port id */);
}

}  // namespace
}  // namespace systems
}  // namespace drake
