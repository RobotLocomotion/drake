#include "drake/systems/framework/diagram_builder.h"

#include <regex>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/pass_through.h"

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

// Tests ::AddNamedSystem() post-condition.
GTEST_TEST(DiagramBuilderTest, AddNamedSystem) {
  DiagramBuilder<double> builder;
  auto a = builder.AddNamedSystem("a", std::make_unique<Adder<double>>(2, 1));
  EXPECT_EQ(a->get_name(), "a");
  auto b = builder.AddNamedSystem<Adder<double>>("b", 2, 1);
  EXPECT_EQ(b->get_name(), "b");
  auto c = builder.AddNamedSystem<Adder>("c", 2 , 1);
  EXPECT_EQ(c->get_name(), "c");
}

// Tests one example of ThrowIfAlreadyBuilt().
GTEST_TEST(DiagramBuilderTest, AlreadyBuilt) {
  DiagramBuilder<double> builder;
  builder.AddSystem<Adder>(1 /* inputs */, 1 /* size */);
  auto diagram = builder.Build();
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.AddSystem<Adder>(2, 2),
      ".*DiagramBuilder may no longer be used.*");
}

// A special class to distinguish between cycles and algebraic loops. The system
// has one input and two outputs. One output simply "echoes" the input (direct
// feedthrough). The other output merely outputs a const value. That means, the
// system *has* feedthrough, but a cycle in the diagram graph does not imply
// an algebraic loop.
template <typename T>
class ConstAndEcho final : public LeafSystem<T> {
 public:
  ConstAndEcho() : LeafSystem<T>(SystemTypeTag<ConstAndEcho>{}) {
    this->DeclareInputPort("input", kVectorValued, 1);
    this->DeclareVectorOutputPort("echo", 1, &ConstAndEcho::CalcEcho);
    this->DeclareVectorOutputPort("constant", 1, &ConstAndEcho::CalcConstant);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit ConstAndEcho(const ConstAndEcho<U>&) : ConstAndEcho() {}

  const systems::InputPort<T>& get_vec_input_port() const {
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
    const auto& input = this->get_vec_input_port().Eval(context);
    echo->get_mutable_value() = input;
  }
};

// Tests that an exception is thrown if the diagram contains an algebraic loop.
GTEST_TEST(DiagramBuilderTest, AlgebraicLoop) {
  DiagramBuilder<double> builder;
  auto adder = builder.AddNamedSystem<Adder>(
      "adder", 1 /* inputs */, 1 /* size */);
  auto pass = builder.AddNamedSystem<PassThrough>("pass", 1 /* size */);
  // Connect the output port to the input port.
  builder.Connect(adder->get_output_port(), pass->get_input_port());
  builder.Connect(pass->get_output_port(), adder->get_input_port(0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Build(), std::runtime_error,
      fmt::format(
          "Reported algebraic loop detected in DiagramBuilder:\n"
          "  InputPort.0. .u0. of {adder} is direct-feedthrough to\n"
          "  OutputPort.0. .sum. of {adder} is connected to\n"
          "  InputPort.0. .u. of {pass} is direct-feedthrough to\n"
          "  OutputPort.0. .y. of {pass} is connected to\n"
          "  InputPort.0. .u0. of {adder}\n"
          ".*conservatively reported.*",
          fmt::arg("adder", "System ::adder .Adder<double>."),
          fmt::arg("pass", "System ::pass .PassThrough<double>.")));
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

  auto echo = builder.AddNamedSystem<ConstAndEcho>("echo");
  builder.Connect(echo->get_const_output_port(), echo->get_vec_input_port());
  DRAKE_EXPECT_NO_THROW(builder.Build());
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

  auto echo = builder.AddNamedSystem<ConstAndEcho>("echo");
  builder.Connect(echo->get_echo_output_port(), echo->get_vec_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Build(), std::runtime_error,
      fmt::format(
          "Reported algebraic loop detected in DiagramBuilder:\n"
          "  InputPort.0. .input. of {sys} is direct-feedthrough to\n"
          "  OutputPort.0. .echo. of {sys} is connected to\n"
          "  InputPort.0. .input. of {sys}\n"
          ".*conservatively reported.*",
          fmt::arg("sys", "System ::echo .ConstAndEcho<double>.")));
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
  auto adder = builder.AddNamedSystem<Adder>(
      "adder", 2 /* inputs */, 1 /* size */);
  auto integrator = builder.AddNamedSystem<Integrator>(
      "integrator", 1 /* size */);

  builder.Connect(integrator->get_output_port(), adder->get_input_port(1));

  // There is no algebraic loop, so we should not throw.
  DRAKE_EXPECT_NO_THROW(builder.Build());
}

// Tests that multiple cascaded elements that are not direct-feedthrough
// are buildable.
GTEST_TEST(DiagramBuilderTest, CascadedNonDirectFeedthrough) {
  DiagramBuilder<double> builder;

  auto integrator1 = builder.AddNamedSystem<Integrator>(
      "integrator1", 1 /* size */);
  auto integrator2 = builder.AddNamedSystem<Integrator>(
      "integrator2, ", 1 /* size */);

  builder.Connect(integrator1->get_output_port(),
                  integrator2->get_input_port());

  // There is no algebraic loop, so we should not throw.
  DRAKE_EXPECT_NO_THROW(builder.Build());
}

// Tests that an exception is thrown when building an empty diagram.
GTEST_TEST(DiagramBuilderTest, FinalizeWhenEmpty) {
  DiagramBuilder<double> builder;
  EXPECT_THROW(builder.Build(), std::logic_error);
}

GTEST_TEST(DiagramBuilderTest, SystemsThatAreNotAddedThrow) {
  DiagramBuilder<double> builder;
  Adder<double> adder(1 /* inputs */, 1 /* size */);
  adder.set_name("adder");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(adder, adder),
      std::logic_error,
      "DiagramBuilder: Cannot operate on ports of System adder "
      "until it has been registered using AddSystem");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ExportInput(adder.get_input_port(0)),
      std::logic_error,
      "DiagramBuilder: Cannot operate on ports of System adder "
      "until it has been registered using AddSystem");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ExportOutput(adder.get_output_port()),
      std::logic_error,
      "DiagramBuilder: Cannot operate on ports of System adder "
      "until it has been registered using AddSystem");
}

GTEST_TEST(DiagramBuilderTest, ConnectVectorToAbstractThrow) {
  DiagramBuilder<double> builder;
  auto vector_system = builder.AddNamedSystem<PassThrough<double>>(
      "vector_system", 1 /* size */);
  auto abstract_system = builder.AddNamedSystem<PassThrough<double>>(
      "abstract_system", Value<int>{});
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          vector_system->get_output_port(),
          abstract_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Cannot mix vector-valued and abstract-valued ports while connecting "
      "output port y of System vector_system to "
      "input port u of System abstract_system");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          abstract_system->get_output_port(),
          vector_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Cannot mix vector-valued and abstract-valued ports while connecting "
      "output port y of System abstract_system to "
      "input port u of System vector_system");
}

GTEST_TEST(DiagramBuilderTest, ExportInputVectorToAbstractThrow) {
  DiagramBuilder<double> builder;
  auto vector_system = builder.AddSystem<PassThrough<double>>(1 /* size */);
  vector_system->set_name("vector_system");
  auto abstract_system = builder.AddSystem<PassThrough<double>>(Value<int>{});
  abstract_system->set_name("abstract_system");
  auto port_index = builder.ExportInput(vector_system->get_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ConnectInput(port_index, abstract_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::ConnectInput: "
      "Cannot mix vector-valued and abstract-valued ports while connecting "
      "input port u of System abstract_system to "
      "input port vector_system_u of Diagram");
}

GTEST_TEST(DiagramBuilderTest, ExportInputAbstractToVectorThrow) {
  DiagramBuilder<double> builder;
  auto vector_system = builder.AddSystem<PassThrough<double>>(1 /* size */);
  vector_system->set_name("vector_system");
  auto abstract_system = builder.AddSystem<PassThrough<double>>(Value<int>{});
  abstract_system->set_name("abstract_system");
  auto port_index = builder.ExportInput(abstract_system->get_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ConnectInput(port_index, vector_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::ConnectInput: "
      "Cannot mix vector-valued and abstract-valued ports while connecting "
      "input port u of System vector_system to "
      "input port abstract_system_u of Diagram");
}

GTEST_TEST(DiagramBuilderTest, ConnectVectorSizeMismatchThrow) {
  DiagramBuilder<double> builder;
  auto size1_system = builder.AddNamedSystem<PassThrough<double>>(
      "size1_system", 1 /* size */);
  auto size2_system = builder.AddNamedSystem<PassThrough<double>>(
      "size2_system", 2 /* size */);
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          size1_system->get_output_port(),
          size2_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Mismatched vector sizes while connecting "
      "output port y of System size1_system \\(size 1\\) to "
      "input port u of System size2_system \\(size 2\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          size2_system->get_output_port(),
          size1_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Mismatched vector sizes while connecting "
      "output port y of System size2_system \\(size 2\\) to "
      "input port u of System size1_system \\(size 1\\)");
}

GTEST_TEST(DiagramBuilderTest, ExportInputVectorSizeMismatchThrow) {
  DiagramBuilder<double> builder;
  auto size1_system = builder.AddSystem<PassThrough<double>>(1 /* size */);
  size1_system->set_name("size1_system");
  auto size2_system = builder.AddSystem<PassThrough<double>>(2 /* size */);
  size2_system->set_name("size2_system");
  auto port_index = builder.ExportInput(size1_system->get_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ConnectInput(port_index, size2_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::ConnectInput: "
      "Mismatched vector sizes while connecting "
      "input port u of System size2_system \\(size 2\\) to "
      "input port size1_system_u of Diagram \\(size 1\\)");
}

GTEST_TEST(DiagramBuilderTest, ConnectAbstractTypeMismatchThrow) {
  DiagramBuilder<double> builder;
  auto int_system = builder.AddNamedSystem<PassThrough<double>>(
      "int_system", Value<int>{});
  auto char_system = builder.AddNamedSystem<PassThrough<double>>(
      "char_system", Value<char>{});
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          int_system->get_output_port(),
          char_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Mismatched value types while connecting "
      "output port y of System int_system \\(type int\\) to "
      "input port u of System char_system \\(type char\\)");
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.Connect(
          char_system->get_output_port(),
          int_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::Connect: "
      "Mismatched value types while connecting "
      "output port y of System char_system \\(type char\\) to "
      "input port u of System int_system \\(type int\\)");
}

GTEST_TEST(DiagramBuilderTest, ExportInputAbstractTypeMismatchThrow) {
  DiagramBuilder<double> builder;
  auto int_system = builder.AddSystem<PassThrough<double>>(Value<int>{});
  int_system->set_name("int_system");
  auto char_system = builder.AddSystem<PassThrough<double>>(Value<char>{});
  char_system->set_name("char_system");
  auto port_index = builder.ExportInput(int_system->get_input_port());
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ConnectInput(port_index, char_system->get_input_port()),
      std::logic_error,
      "DiagramBuilder::ConnectInput: "
      "Mismatched value types while connecting "
      "input port u of System char_system \\(type char\\) to "
      "input port int_system_u of Diagram \\(type int\\)");
}

// Test that port connections can be polymorphic, especially for types that are
// both copyable and cloneable.  {ExponentialPlus,}PiecewisePolynomial are both
// copyable (to themselves) and cloneable (to a common base class, Trajectory).
// To connect them in a diagram, we must specify we want to use the subtyping.
GTEST_TEST(DiagramBuilderTest, ConnectAbstractSubtypes) {
  using Trajectoryd = trajectories::Trajectory<double>;
  using PiecewisePolynomiald = trajectories::PiecewisePolynomial<double>;
  using ExponentialPlusPiecewisePolynomiald =
      trajectories::ExponentialPlusPiecewisePolynomial<double>;

  DiagramBuilder<double> builder;
  auto sys1 = builder.AddSystem<PassThrough<double>>(
      Value<Trajectoryd>(PiecewisePolynomiald{}));
  auto sys2 = builder.AddSystem<PassThrough<double>>(
      Value<Trajectoryd>(ExponentialPlusPiecewisePolynomiald{}));
  DRAKE_EXPECT_NO_THROW(builder.Connect(*sys1, *sys2));
  EXPECT_FALSE(builder.IsConnectedOrExported(sys1->get_input_port()));
  builder.ExportInput(sys1->get_input_port());
  EXPECT_TRUE(builder.IsConnectedOrExported(sys1->get_input_port()));
  builder.ExportOutput(sys2->get_output_port());
  auto diagram = builder.Build();

  // We can feed PiecewisePolynomial through the Diagram.
  {
    auto context = diagram->CreateDefaultContext();
    const PiecewisePolynomiald input(Vector1d(22.0));
    diagram->get_input_port(0).FixValue(
        context.get(), Value<Trajectoryd>(input));
    const auto& output =
        dynamic_cast<const PiecewisePolynomiald&>(
            diagram->get_output_port(0).Eval<Trajectoryd>(*context));
    EXPECT_EQ(output.value(0.0)(0), 22.0);
  }

  // We can feed ExponentialPlusPiecewisePolynomial through the Diagram.
  {
    auto context = diagram->CreateDefaultContext();
    const ExponentialPlusPiecewisePolynomiald input(
        PiecewisePolynomiald::ZeroOrderHold(Eigen::Vector2d(0., 1.),
                                            Eigen::RowVector2d(22.0, 22.0)));
    diagram->get_input_port(0).FixValue(
        context.get(), Value<Trajectoryd>(input));
    const auto& output =
        dynamic_cast<const ExponentialPlusPiecewisePolynomiald&>(
            diagram->get_output_port(0).Eval<Trajectoryd>(*context));
    EXPECT_EQ(output.value(0.0)(0), 22.0);
  }
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

  EXPECT_FALSE(builder.IsConnectedOrExported(sink1->get_input_port(0)));
  EXPECT_FALSE(builder.IsConnectedOrExported(sink2->get_input_port(0)));
  builder.ExportInput(sink1->get_input_port(0));
  builder.ExportInput(sink2->get_input_port(0));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink1->get_input_port(0)));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink2->get_input_port(0)));

  // If the port names were not unique, then the build step would throw.
  DRAKE_EXPECT_NO_THROW(builder.Build());
}

GTEST_TEST(DiagramBuilderTest, DefaultOutputPortNamesAreUniqueTest) {
  DiagramBuilder<double> builder;

  auto source1 = builder.AddSystem<Source<double>>();
  auto source2 = builder.AddSystem<Source<double>>();

  builder.ExportOutput(source1->get_output_port(0));
  builder.ExportOutput(source2->get_output_port(0));

  // If the port names were not unique, then the build step would throw.
  DRAKE_EXPECT_NO_THROW(builder.Build());
}

GTEST_TEST(DiagramBuilderTest, DefaultPortNamesAreUniqueTest2) {
  DiagramBuilder<double> builder;

  // This time, we assign system names manually.
  auto sink1 = builder.AddNamedSystem<Sink<double>>("sink1");
  auto sink2 = builder.AddNamedSystem<Sink<double>>("sink2");

  auto source1 = builder.AddNamedSystem<Source<double>>("source1");
  auto source2 = builder.AddNamedSystem<Source<double>>("source2");

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
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.ExportInput(sink2->get_input_port(0), "sink"),
      std::logic_error, ".*already has an input port named.*");
}

GTEST_TEST(DiagramBuilderTest, InputPortNamesFanout) {
  DiagramBuilder<double> builder;

  auto sink1 = builder.AddSystem<Sink<double>>();
  auto sink2 = builder.AddSystem<Sink<double>>();
  auto sink3 = builder.AddSystem<Sink<double>>();
  auto sink4 = builder.AddSystem<Sink<double>>();

  std::vector<InputPortIndex> indices;
  // Name these ports just to make comparing easier later.
  EXPECT_FALSE(builder.IsConnectedOrExported(sink1->get_input_port(0)));
  indices.push_back(builder.ExportInput(sink1->get_input_port(0), "in1"));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink1->get_input_port(0)));

  EXPECT_FALSE(builder.IsConnectedOrExported(sink2->get_input_port(0)));
  indices.push_back(builder.ExportInput(sink2->get_input_port(0), "in2"));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink2->get_input_port(0)));

  // Fan-out connect by index.
  EXPECT_FALSE(builder.IsConnectedOrExported(sink3->get_input_port(0)));
  builder.ConnectInput(indices[0], sink3->get_input_port(0));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink3->get_input_port(0)));

  // Fan-out connect by name.
  EXPECT_FALSE(builder.IsConnectedOrExported(sink4->get_input_port(0)));
  builder.ConnectInput("in1", sink4->get_input_port(0));
  EXPECT_TRUE(builder.IsConnectedOrExported(sink4->get_input_port(0)));

  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 1);

  auto diagram = builder.Build();
  EXPECT_EQ(diagram->num_input_ports(), 2);
  EXPECT_EQ(diagram->get_input_port(0).get_name(), "in1");
  EXPECT_EQ(diagram->get_input_port(1).get_name(), "in2");

  auto sink_fanout = diagram->GetInputPortLocators(InputPortIndex(0));
  EXPECT_EQ(sink_fanout.size(), 3);
  auto sinkhole_fanout = diagram->GetInputPortLocators(InputPortIndex(1));
  EXPECT_EQ(sinkhole_fanout.size(), 1);
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
    out1_ = builder_.AddNamedSystem<ConstantVectorSource>(
        "constant", Vector1d::Ones());
    in1_ = builder_.AddNamedSystem<Sink>("sink");
    in1out1_ = builder_.AddNamedSystem<Gain>(
        "gain", 1.0 /* gain */, 1 /* size */);
    in2out1_ = builder_.AddNamedSystem<Adder>(
        "adder", 2 /* inputs */, 1 /* size */);
    in1out2_ = builder_.AddNamedSystem<Demultiplexer>("demux", 2 /* size */);
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
  EXPECT_FALSE(builder_.IsConnectedOrExported(in1out1_->get_input_port()));
  DRAKE_EXPECT_NO_THROW(builder_.Connect(*out1_, *in1out1_));
  EXPECT_TRUE(builder_.IsConnectedOrExported(in1out1_->get_input_port()));
  DRAKE_EXPECT_NO_THROW(builder_.Connect(*in1out1_, *in1_));
  DRAKE_EXPECT_NO_THROW(builder_.Build());
}

// The cascade synonym also works.
TEST_F(DiagramBuilderSolePortsTest, SourceGainSinkCascade) {
  DRAKE_EXPECT_NO_THROW(builder_.Cascade(*out1_, *in1out1_));
  DRAKE_EXPECT_NO_THROW(builder_.Cascade(*in1out1_, *in1_));
  DRAKE_EXPECT_NO_THROW(builder_.Build());
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

// Test for GetSystems and GetMutableSystems.
GTEST_TEST(DiagramBuilderTest, GetMutableSystems) {
  DiagramBuilder<double> builder;
  auto adder1 = builder.AddNamedSystem<Adder>(
      "adder1", 1 /* inputs */, 1 /* size */);
  auto adder2 = builder.AddNamedSystem<Adder>(
      "adder2", 1 /* inputs */, 1 /* size */);
  EXPECT_EQ((std::vector<const System<double>*>{adder1, adder2}),
            builder.GetSystems());
  EXPECT_EQ((std::vector<System<double>*>{adder1, adder2}),
            builder.GetMutableSystems());
}

// Tests that the returned exported input / output port id matches the
// number of ExportInput() / ExportOutput() calls.
GTEST_TEST(DiagramBuilderTest, ExportInputOutputIndex) {
  DiagramBuilder<double> builder;
  auto adder1 = builder.AddNamedSystem<Adder>(
      "adder1", 3 /* inputs */, 1 /* size */);
  auto adder2 = builder.AddNamedSystem<Adder>(
      "adder2", 1 /* inputs */, 1 /* size */);

  EXPECT_EQ(builder.ExportInput(
        adder1->get_input_port(0)), 0 /* exported input port id */);
  EXPECT_EQ(builder.ExportInput(
        adder1->get_input_port(1)), 1 /* exported input port id */);

  EXPECT_EQ(builder.ExportOutput(
        adder1->get_output_port()), 0 /* exported output port id */);
  EXPECT_EQ(builder.ExportOutput(
        adder2->get_output_port()), 1 /* exported output port id */);
}

class DtorTraceSystem final : public LeafSystem<double> {
 public:
  explicit DtorTraceSystem(int nonce) : nonce_(nonce) {}
  ~DtorTraceSystem() { destroyed_nonces().push_back(nonce_); }

  static std::vector<int>& destroyed_nonces() {
    static never_destroyed<std::vector<int>> nonces;
    return nonces.access();
  }

 private:
  int nonce_{};
};

// Tests the destruction order of DiagramBuilder.
GTEST_TEST(DiagramBuilderTest, DtorOrder_Builder) {
  auto& nonces = DtorTraceSystem::destroyed_nonces();

  nonces.clear();
  auto dut = std::make_unique<DiagramBuilder<double>>();
  dut->AddSystem<DtorTraceSystem>(1);
  dut->AddSystem<DtorTraceSystem>(2);
  EXPECT_EQ(nonces.size(), 0);
  dut.reset();
  EXPECT_EQ(nonces, (std::vector<int>{2, 1}));
}

// Tests the destruction order of Diagram.
GTEST_TEST(DiagramBuilderTest, DtorOrder_Built) {
  auto& nonces = DtorTraceSystem::destroyed_nonces();

  nonces.clear();
  auto builder = std::make_unique<DiagramBuilder<double>>();
  builder->AddSystem<DtorTraceSystem>(-1);
  builder->AddSystem<DtorTraceSystem>(-2);
  auto dut = builder->Build();
  EXPECT_EQ(nonces.size(), 0);
  dut.reset();
  EXPECT_EQ(nonces, (std::vector<int>{-2, -1}));
}

}  // namespace
}  // namespace systems
}  // namespace drake
