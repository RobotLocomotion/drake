#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/analysis/test_utilities/stateless_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

class DoubleOnlySystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleOnlySystem);
  DoubleOnlySystem() = default;
};

/// A stateless system that can set an arbitrary periodic discrete update.
template <class T>
class EmptySystem : public LeafSystem<T> {
 public:
  ~EmptySystem() override {}

  // Adds an arbitrary periodic discrete update.
  void AddPeriodicDiscreteUpdate() {
    const double default_period = 1.125;
    const double default_offset = 2.25;
    this->DeclarePeriodicDiscreteUpdate(default_period, default_offset);
  }

  // Adds a specific periodic discrete update.
  void AddPeriodicDiscreteUpdate(double period, double offset) {
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }
};

/// A recursive diagram of purely empty systems used for testing that diagram
/// mechanics are working for periodic discrete update events.
class EmptySystemDiagram : public Diagram<double> {
 public:
  // Enum for how many periodic discrete updates are performed at each level
  // of the diagram.
  enum UpdateType {
    kTwoUpdatesPerLevel,
    kOneUpdatePerLevelSys1,
    kOneUpdatePerLevelSys2,
    kOneUpdateAtLastLevelSys1,
    kOneUpdateAtLastLevelSys2,
    kTwoUpdatesAtLastLevel,
  };

  // Creates a diagram of "empty" systems with the specified recursion depth.
  // A recursion depth of zero will create two empty systems only; Otherwise,
  // 2*`recursion_depth` empty systems will be created.
  EmptySystemDiagram(UpdateType num_periodic_discrete_updates,
                     int recursion_depth,
                     bool unique_updates) {
    DRAKE_DEMAND(recursion_depth >= 0);

    DiagramBuilder<double> builder;

    // Add in two empty systems.
    auto sys1 = builder.AddSystem<EmptySystem<double>>();
    auto sys2 = builder.AddSystem<EmptySystem<double>>();

    switch (num_periodic_discrete_updates) {
      case kTwoUpdatesPerLevel:
        if (unique_updates) {
          sys1->AddPeriodicDiscreteUpdate();
          sys2->AddPeriodicDiscreteUpdate();
        } else {
          sys1->AddPeriodicDiscreteUpdate(recursion_depth + 1,
                                          recursion_depth * 3);
          sys2->AddPeriodicDiscreteUpdate(recursion_depth + 3,
                                          recursion_depth * 5);
        }
        break;

      case kOneUpdatePerLevelSys1:
        if (unique_updates) {
          sys1->AddPeriodicDiscreteUpdate();
        } else {
          sys1->AddPeriodicDiscreteUpdate(recursion_depth * 7, recursion_depth);
        }
        break;

      case kOneUpdatePerLevelSys2:
        if (unique_updates) {
          sys2->AddPeriodicDiscreteUpdate();
        } else {
          sys2->AddPeriodicDiscreteUpdate(recursion_depth,
                                          recursion_depth * 11);
        }
        break;

      case kOneUpdateAtLastLevelSys1:
        if (recursion_depth == 0) {
          if (unique_updates) {
            sys1->AddPeriodicDiscreteUpdate();
          } else {
            sys1->AddPeriodicDiscreteUpdate(13, 17);
          }
        }
        break;

      case kOneUpdateAtLastLevelSys2:
        if (recursion_depth == 0) {
          if (unique_updates) {
            sys2->AddPeriodicDiscreteUpdate();
          } else {
            sys2->AddPeriodicDiscreteUpdate(19, 23);
          }
        }
        break;

      case kTwoUpdatesAtLastLevel:
        if (recursion_depth == 0) {
          if (unique_updates) {
            sys1->AddPeriodicDiscreteUpdate();
            sys2->AddPeriodicDiscreteUpdate();
          } else {
            sys1->AddPeriodicDiscreteUpdate(29, 31);
            sys2->AddPeriodicDiscreteUpdate(37, 43);
          }
        }
        break;
    }

    // Now add a sub-StatelessDiagram with one less recursion depth (if the
    // recursion depth is not zero).
    if (recursion_depth > 0) {
      builder.AddSystem<EmptySystemDiagram>(
        num_periodic_discrete_updates,
        recursion_depth - 1,
        unique_updates);
    }
    builder.BuildInto(this);
    EXPECT_FALSE(IsDifferenceEquationSystem());
  }
};

void CheckPeriodAndOffset(const PeriodicEventData& data) {
  EXPECT_EQ(data.period_sec(), 1.125);
  EXPECT_EQ(data.offset_sec(), 2.25);
}

// Tests whether the diagram exhibits the correct behavior for
// GetUniquePeriodicDiscreteUpdateAttribute().
GTEST_TEST(EmptySystemDiagramTest, CheckPeriodicTriggerDiscreteUpdateUnique) {
  // Check diagrams with no recursion.
  std::optional<PeriodicEventData> periodic_data;
  EmptySystemDiagram d_sys2upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys1, 0, true);
  EmptySystemDiagram d_sys1upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys2, 0, true);
  EmptySystemDiagram d_bothupd_zero(EmptySystemDiagram::kTwoUpdatesPerLevel, 0,
      true);
  ASSERT_TRUE(periodic_data =
      d_sys2upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset(periodic_data.value());
  ASSERT_TRUE(periodic_data =
      d_sys1upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset(periodic_data.value());
  ASSERT_TRUE(periodic_data =
      d_bothupd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset(periodic_data.value());

  // Check systems with up to three levels of recursion.
  for (int i = 1; i <= 3; ++i) {
    // Create the systems.
    EmptySystemDiagram d_sys1upd(
        EmptySystemDiagram::kOneUpdatePerLevelSys1, i, true);
    EmptySystemDiagram d_sys2upd(
        EmptySystemDiagram::kOneUpdatePerLevelSys2, i, true);
    EmptySystemDiagram d_bothupd(
        EmptySystemDiagram::kTwoUpdatesPerLevel, i, true);
    EmptySystemDiagram d_sys1_last(
        EmptySystemDiagram::kOneUpdateAtLastLevelSys1, i, true);
    EmptySystemDiagram d_sys2_last(
        EmptySystemDiagram::kOneUpdateAtLastLevelSys2, i, true);
    EmptySystemDiagram d_both_last(
        EmptySystemDiagram::kTwoUpdatesAtLastLevel, i, true);

    // All of these should return "true". Check them.
    ASSERT_TRUE(periodic_data =
        d_sys1upd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
    ASSERT_TRUE(periodic_data =
        d_sys2upd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
    ASSERT_TRUE(periodic_data =
        d_bothupd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
    ASSERT_TRUE(periodic_data =
        d_both_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
    ASSERT_TRUE(periodic_data =
        d_sys1_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
    ASSERT_TRUE(periodic_data =
        d_sys2_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset(periodic_data.value());
  }
}

// Tests whether the diagram exhibits the correct behavior for
// GetUniquePeriodicDiscreteUpdateAttribute() with non-unique updates
GTEST_TEST(EmptySystemDiagramTest, CheckPeriodicTriggerDiscreteUpdate) {
  // Check diagrams with no recursion.
  PeriodicEventData periodic_data;
  EmptySystemDiagram d_sys2upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys1, 0, false);
  EmptySystemDiagram d_sys1upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys2, 0, false);
  EmptySystemDiagram d_bothupd_zero(EmptySystemDiagram::kTwoUpdatesPerLevel, 0,
      false);
  EXPECT_TRUE(d_sys2upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  EXPECT_TRUE(d_sys1upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  EXPECT_FALSE(d_bothupd_zero.GetUniquePeriodicDiscreteUpdateAttribute());

  // Check systems with up to three levels of recursion.
  for (int i = 1; i <= 3; ++i) {
    // Create the systems.
    EmptySystemDiagram d_sys1upd(
        EmptySystemDiagram::kOneUpdatePerLevelSys1, i, false);
    EmptySystemDiagram d_sys2upd(
        EmptySystemDiagram::kOneUpdatePerLevelSys2, i, false);
    EmptySystemDiagram d_bothupd(
        EmptySystemDiagram::kTwoUpdatesPerLevel, i, false);
    EmptySystemDiagram d_sys1_last(
        EmptySystemDiagram::kOneUpdateAtLastLevelSys1, i, false);
    EmptySystemDiagram d_sys2_last(
        EmptySystemDiagram::kOneUpdateAtLastLevelSys2, i, false);
    EmptySystemDiagram d_both_last(
        EmptySystemDiagram::kTwoUpdatesAtLastLevel, i, false);

    // None of these should have a unique periodic event.
    EXPECT_FALSE(d_sys1upd.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_sys1upd.GetPeriodicEvents().size(), i + 1);
    EXPECT_FALSE(d_sys2upd.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_sys2upd.GetPeriodicEvents().size(), i + 1);
    EXPECT_FALSE(d_bothupd.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_bothupd.GetPeriodicEvents().size(), 2 * (i + 1));
    EXPECT_FALSE(d_both_last.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_both_last.GetPeriodicEvents().size(), 2);

    // All of these should have a unique periodic event.
    EXPECT_TRUE(d_sys1_last.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_sys1_last.GetPeriodicEvents().size(), 1);
    EXPECT_TRUE(d_sys2_last.GetUniquePeriodicDiscreteUpdateAttribute());
    EXPECT_EQ(d_sys2_last.GetPeriodicEvents().size(), 1);
  }
}

/* This Diagram contains a sub-Diagram with one of its input ports unconnected.
This is a specialized test to cover a case that was otherwise missed -- a
Diagram has an input port that is neither exported nor fixed.

         +--------------------------+
         |                          |
         |   +------------------+   |
         |   |                  |   |
         |   |  +-----------+   |   |
      ----------> u0        |   |   |
         |   |  |           |   |   |
         |   |  | Adder1    |   |   |
         |   |  |           +----------> y0
         |   |  |           |   |   |
         | X----> u1        |   |   |
         |   |  +-----------+   |   |   X = forgot to export
         |   |                  |   |
         |   +------------------+   |
         |     InsideBadDiagram     |
         |                          |
         +--------------------------+
                  BadDiagram

*/
class InsideBadDiagram : public Diagram<double> {
 public:
  InsideBadDiagram() {
    const int kSize = 1;
    DiagramBuilder<double> builder;
    adder0_ = builder.AddSystem<Adder<double>>(2 /* inputs */, kSize);
    adder0_->set_name("adder0");
    builder.ExportInput(adder0().get_input_port(0));
    builder.ExportInput(adder0().get_input_port(1));
    builder.ExportOutput(adder0().get_output_port());
    builder.BuildInto(this);
  }

  const Adder<double>& adder0() { return *adder0_; }

 private:
  Adder<double>* adder0_{};
};

class BadDiagram : public Diagram<double> {
 public:
  BadDiagram() {
    DiagramBuilder<double> builder;
    inside_ = builder.AddSystem<InsideBadDiagram>();
    inside_->set_name("inside");
    builder.ExportInput(inside().get_input_port(0));
    // Oops -- "forgot" to export this.
    // builder.ExportInput(inside().get_input_port(1));
    builder.ExportOutput(inside().get_output_port(0));
    builder.BuildInto(this);
  }

  const InsideBadDiagram& inside() { return *inside_; }

 private:
  InsideBadDiagram* inside_{};
};

GTEST_TEST(BadDiagramTest, UnconnectedInsideInputPort) {
  BadDiagram diagram;
  auto context = diagram.AllocateContext();

  diagram.get_input_port(0).FixValue(context.get(), 1.0);

  const Context<double>& inside_context =
      diagram.GetSubsystemContext(diagram.inside(), *context);

  // This should get the value we just fixed above.
  EXPECT_EQ(diagram.inside().get_input_port(0).Eval(inside_context)[0], 1);

  // This is neither exported, connected, nor fixed so shouldn't have a value.
  EXPECT_FALSE(diagram.inside().get_input_port(1).HasValue(inside_context));
}

/* This System declares at least one of every kind of state and parameter
so we can check if the SystemBase "declared sizes" counts work correctly.
(FYI this has nothing to do with dishwashing -- think "everything but the
kitchen sink"!) */
template <typename T>
class KitchenSinkStateAndParameters final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KitchenSinkStateAndParameters)

  KitchenSinkStateAndParameters() :
      LeafSystem<T>(systems::SystemTypeTag<KitchenSinkStateAndParameters>{}) {
    this->DeclareContinuousState(4, 3, 5);  // nq, nv, nz
    for (int i = 0; i < 2; ++i)  // Make two "groups" of discrete variables.
      this->DeclareDiscreteState(4 + i);
    for (int i = 0; i < 10; ++i)  // Ten abstract state variables.
      this->DeclareAbstractState(Value<int>(3 + i));
    for (int i = 0; i < 3; ++i)  // Three "groups" of numeric parameters.
      this->DeclareNumericParameter(BasicVector<T>(1 + i));
    for (int i = 0; i < 7; ++i)  // Seven abstract parameters.
      this->DeclareAbstractParameter(Value<int>(29 + i));
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit KitchenSinkStateAndParameters(
      const KitchenSinkStateAndParameters<U>&)
      : KitchenSinkStateAndParameters<T>() {}

 private:
  // Must provide derivatives since we have continuous states.
  void DoCalcTimeDerivatives(
      const Context<T>& context,
      ContinuousState<T>* derivatives) const override {
    // xdot = 0.
    derivatives->SetFromVector(
        VectorX<T>::Zero(this->num_continuous_states()));
  }
};

GTEST_TEST(KitchenSinkStateAndParametersTest, LeafSystemCounts) {
  KitchenSinkStateAndParameters<double> kitchen_sink;
  EXPECT_EQ(kitchen_sink.num_continuous_states(), 12);
  EXPECT_EQ(kitchen_sink.num_discrete_state_groups(), 2);
  EXPECT_EQ(kitchen_sink.num_abstract_states(), 10);
  EXPECT_EQ(kitchen_sink.num_numeric_parameter_groups(), 3);
  EXPECT_EQ(kitchen_sink.num_abstract_parameters(), 7);
}

// Helper class that has one input port, and no output ports.
template <typename T>
class Sink final : public LeafSystem<T> {
 public:
  explicit Sink(int size) : LeafSystem<T>(SystemTypeTag<Sink>{}) {
    this->DeclareInputPort("in", kVectorValued, size);
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Sink(const Sink<U>& other): Sink<T>(other.get_input_port().size()) {}
};

/* ExampleDiagram has the following structure:
adder0_: (input0_ + input1_) -> A
adder1_: (A + input2_)       -> B, output 0
adder2_: (A + B)             -> output 1
integrator1_: A              -> C
integrator2_: C              -> output 2
sink_ : (input2_)
It also uses a StatelessSystem to verify Diagram's ability to retrieve
witness functions from its subsystems.

             +----------------------------------------------------------+
             |                                                          |
             |  +--------+                       +------------------------->
1, 2, 4  +------>        |                       |                   B  | y0
          u0 |  | Adder0 | A  +-----------+      |                      |
             |  |        +-+--> u0        |      |                      |
8, 16, 32+------>        | |  |           |      |                      |
          u1 |  +--------+ |  | Adder1    |  B   |       +-----------+  |
             |             |  |           +------+-------> u1        |  |
64, 128, 256 |             |  |           | 73,146,292   |           |  |
         +----------+---------> u1        |              | Adder2    |  |
          u2 |      |      |  +-----------+              |           +----->
             |      |      |                 A           |           |  | y1
             |      |      +-----------------------------> u0        |  |
             |      |      |     9, 18, 36               +-----------+  |  82
             |      |      |                                            | 164
             |      |      |                                            | 328
             |      |      |                                            |
             |      |      |  +------------+             +-----------+  |
             |      |      |  |            |             |           |  |
             |      |    A |  |            |     C       |           |  |
             |      |      +--> Integ0     +-------------> Integ1    +----->
             |      |         |            |             |           |  | y2
             |      |         |  3, 9, 27  |             |81,243,729 |  |
             |      |         +------------+             +-----------+  |
             |      |                                                   |
             |      |         +------+                                  |
             |      |         |      |                                  |
             |      +---------> Sink |                                  |
             |                |      |                                  |
             |                +------+                                  |
             |                                                          |
             |  +-----------+  +----------------+  +----------------+   |
             |  |           |  | ConstantVector |  |                |   |
             |  | Stateless |  |   or           |  |  KitchenSink   |   |
             |  |           |  | DoubleOnly     |  |                |   |
             |  +-----------+  +----------------+  +----------------+   |
             |                                                          |
             +----------------------------------------------------------|
*/
class ExampleDiagram : public Diagram<double> {
 public:
  explicit ExampleDiagram(
      int size, bool use_abstract = false, bool use_double_only = false) {
    DiagramBuilder<double> builder;

    adder0_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder0_->set_name("adder0");
    adder1_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder1_->set_name("adder1");
    adder2_ = builder.AddSystem<Adder<double>>(2 /* inputs */, size);
    adder2_->set_name("adder2");
    stateless_ = builder.AddSystem<analysis_test::StatelessSystem<double>>(
        1.0 /* trigger time */,
        WitnessFunctionDirection::kCrossesZero);
    stateless_->set_name("stateless");

    integrator0_ = builder.AddSystem<Integrator<double>>(size);
    integrator0_->set_name("integrator0");
    integrator1_ = builder.AddSystem<Integrator<double>>(size);
    integrator1_->set_name("integrator1");
    sink_ = builder.AddSystem<Sink>(size);
    sink_->set_name("sink");

    builder.Connect(adder0_->get_output_port(), adder1_->get_input_port(0));
    builder.Connect(adder0_->get_output_port(), adder2_->get_input_port(0));
    builder.Connect(adder1_->get_output_port(), adder2_->get_input_port(1));

    builder.Connect(adder0_->get_output_port(),
                    integrator0_->get_input_port());
    builder.Connect(integrator0_->get_output_port(),
                    integrator1_->get_input_port());

    builder.ExportInput(adder0_->get_input_port(0));
    builder.ExportInput(adder0_->get_input_port(1), "adder0");
    const auto port_index = builder.ExportInput(adder1_->get_input_port(1));
    builder.ConnectInput(port_index, sink_->get_input_port());
    builder.ExportOutput(adder1_->get_output_port());
    builder.ExportOutput(adder2_->get_output_port(), "adder2");
    builder.ExportOutput(integrator1_->get_output_port());

    if (use_abstract) {
      builder.AddSystem<ConstantValueSource<double>>(Value<int>(0));
    }
    if (use_double_only) {
      builder.AddSystem<DoubleOnlySystem>();
    }

    kitchen_sink_ = builder.AddSystem<KitchenSinkStateAndParameters<double>>();
    kitchen_sink_->set_name("kitchen_sink");

    builder.BuildInto(this);
  }

  Adder<double>* adder0() { return adder0_; }
  Adder<double>* adder1() { return adder1_; }
  Adder<double>* adder2() { return adder2_; }
  Integrator<double>* integrator0() { return integrator0_; }
  Integrator<double>* integrator1() { return integrator1_; }
  Sink<double>* sink() { return sink_; }
  analysis_test::StatelessSystem<double>* stateless() { return stateless_; }
  KitchenSinkStateAndParameters<double>* kitchen_sink() {
    return kitchen_sink_;
  }

 private:
  Adder<double>* adder0_ = nullptr;
  Adder<double>* adder1_ = nullptr;
  Adder<double>* adder2_ = nullptr;
  analysis_test::StatelessSystem<double>* stateless_ = nullptr;

  Integrator<double>* integrator0_ = nullptr;
  Integrator<double>* integrator1_ = nullptr;
  Sink<double>* sink_ = nullptr;

  KitchenSinkStateAndParameters<double>* kitchen_sink_ = nullptr;
};

class DiagramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    diagram_ = std::make_unique<ExampleDiagram>(kSize);
    diagram_->set_name("Unicode Snowman's Favorite Diagram!!1!☃!");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput();

    // Make sure caching is on locally, even if it is off by default.
    // Do not remove this line; we want to show that these tests function
    // correctly with caching on. It is easier to pass with caching off!
    context_->EnableCaching();

    // Initialize the integrator states.
    auto& integrator0_xc = GetMutableContinuousState(integrator0());
    integrator0_xc.get_mutable_vector()[0] = 3;
    integrator0_xc.get_mutable_vector()[1] = 9;
    integrator0_xc.get_mutable_vector()[2] = 27;

    auto& integrator1_xc = GetMutableContinuousState(integrator1());
    integrator1_xc.get_mutable_vector()[0] = 81;
    integrator1_xc.get_mutable_vector()[1] = 243;
    integrator1_xc.get_mutable_vector()[2] = 729;
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>& GetMutableContinuousState(
      const System<double>* system) {
    return diagram_->GetMutableSubsystemState(*system, context_.get())
        .get_mutable_continuous_state();
  }

  // Asserts that output_ is what it should be for the default values
  // of input0_, input1_, and input2_.
  void ExpectDefaultOutputs() {
    Vector3d expected_output0(
        1 + 8 + 64,
        2 + 16 + 128,
        4 + 32 + 256);  // B

    Vector3d expected_output1(
        1 + 8,
        2 + 16,
        4 + 32);  // A
    expected_output1 += expected_output0;       // A + B

    Vector3d expected_output2(81, 243, 729);  // state of integrator1_

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
    diagram_->get_input_port(0).FixValue(context_.get(), input0_);
    diagram_->get_input_port(1).FixValue(context_.get(), input1_);
    diagram_->get_input_port(2).FixValue(context_.get(), input2_);
  }

  Adder<double>* adder0() { return diagram_->adder0(); }
  Integrator<double>* integrator0() { return diagram_->integrator0(); }
  Integrator<double>* integrator1() { return diagram_->integrator1(); }

  const int kSize = 3;

  std::unique_ptr<ExampleDiagram> diagram_;

  const Vector3d input0_{1, 2, 4};
  const Vector3d input1_{8, 16, 32};
  const Vector3d input2_{64, 128, 256};

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram returns the correct number of continuous states
// without a context. The class ExampleDiagram above contains two integrators,
// each of which has three state variables, plus the "kitchen sink" which
// has 12.
TEST_F(DiagramTest, NumberOfContinuousStates) {
  EXPECT_EQ(3+3+12, diagram_->num_continuous_states());
}

// Tests that the diagram returns the correct number of witness functions and
// that the witness function can be called correctly.
TEST_F(DiagramTest, Witness) {
  std::vector<const WitnessFunction<double>*> wf;
  diagram_->GetWitnessFunctions(*context_, &wf);

  // Stateless function always returns the ClockWitness.
  ASSERT_EQ(wf.size(), 1);
  EXPECT_LT(diagram_->CalcWitnessValue(*context_, *wf.front()), 0);
}

// Tests that the diagram exports the correct topology.
TEST_F(DiagramTest, Topology) {
  ASSERT_EQ(kSize, diagram_->num_input_ports());
  for (int i = 0; i < kSize; ++i) {
    const auto& input_port = diagram_->get_input_port(i);
    EXPECT_EQ(diagram_.get(), &input_port.get_system());
    EXPECT_EQ(kVectorValued, input_port.get_data_type());
    EXPECT_EQ(kSize, input_port.size());
  }

  ASSERT_EQ(kSize, diagram_->num_output_ports());
  for (int i = 0; i < kSize; ++i) {
    const auto& port = diagram_->get_output_port(i);
    EXPECT_EQ(diagram_.get(), &port.get_system());
    EXPECT_EQ(kVectorValued, port.get_data_type());
    EXPECT_EQ(kSize, port.size());
  }

  // The diagram has direct feedthrough.
  EXPECT_TRUE(diagram_->HasAnyDirectFeedthrough());
  // Specifically, outputs 0 and 1 have direct feedthrough, but not output 2.
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(0));
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(1));
  EXPECT_FALSE(diagram_->HasDirectFeedthrough(2));
  // Specifically, outputs 0 and 1 have direct feedthrough from all inputs.
  for (int i = 0; i < kSize; ++i) {
    EXPECT_TRUE(diagram_->HasDirectFeedthrough(i, 0));
    EXPECT_TRUE(diagram_->HasDirectFeedthrough(i, 1));
    EXPECT_FALSE(diagram_->HasDirectFeedthrough(i, 2));
  }
}

// Confirms that the Diagram::AreConnected() query reports the correct results.
TEST_F(DiagramTest, AreConnected) {
  // Several verifiably connected ports.
  EXPECT_TRUE(diagram_->AreConnected(diagram_->adder0()->get_output_port(),
                                     diagram_->adder1()->get_input_port(0)));
  EXPECT_TRUE(diagram_->AreConnected(diagram_->adder1()->get_output_port(),
                                     diagram_->adder2()->get_input_port(1)));
  EXPECT_TRUE(diagram_->AreConnected(diagram_->adder0()->get_output_port(),
                                     diagram_->adder2()->get_input_port(0)));

  // A couple unconnected ports -- but they all belong to the diagram.
  EXPECT_FALSE(diagram_->AreConnected(diagram_->adder0()->get_output_port(),
                                      diagram_->adder1()->get_input_port(1)));
  EXPECT_FALSE(diagram_->AreConnected(diagram_->adder1()->get_output_port(),
                                      diagram_->adder2()->get_input_port(0)));

  // Test against a port that does *not* belong to the diagram.
  Adder<double> temp_adder(2, kSize);
  EXPECT_FALSE(diagram_->AreConnected(temp_adder.get_output_port(),
                                      diagram_->adder1()->get_input_port(1)));
  EXPECT_FALSE(diagram_->AreConnected(diagram_->adder1()->get_output_port(),
                                      temp_adder.get_input_port(1)));
}

// Tests that dependency wiring is set up correctly between
//  1. input ports and their source output ports,
//  2. diagram output ports and their source leaf output ports,
//  3. diagram input ports and the corresponding exported leaf inputs,
//  4. diagram states/parameters and their constituent leaf states/parameters.
TEST_F(DiagramTest, CheckPortSubscriptions) {
  const Context<double>& adder1_subcontext =
      diagram_->GetSubsystemContext(*diagram_->adder1(), *context_);
  const OutputPortBase& adder1_oport0 =
      diagram_->adder1()->get_output_port_base(OutputPortIndex(0));
  const DependencyTracker& adder1_oport0_tracker =
      adder1_subcontext.get_tracker(adder1_oport0.ticket());
  const Context<double>& adder2_subcontext =
      diagram_->GetSubsystemContext(*diagram_->adder2(), *context_);
  const InputPortBase& adder2_iport1 =
      diagram_->adder2()->get_input_port_base(InputPortIndex(1));
  const DependencyTracker& adder2_iport1_tracker =
      adder2_subcontext.get_tracker(adder2_iport1.ticket());

  // 1. Adder2's input port 1 depends on adder1's output port 0.
  EXPECT_TRUE(adder1_oport0_tracker.HasSubscriber(adder2_iport1_tracker));
  EXPECT_TRUE(adder2_iport1_tracker.HasPrerequisite(adder1_oport0_tracker));

  const OutputPortBase& diagram_oport1 =
      diagram_->get_output_port_base(OutputPortIndex(1));
  const DependencyTracker& diagram_oport1_tracker =
      context_->get_tracker(diagram_oport1.ticket());
  const OutputPortBase& adder2_oport0 =
      diagram_->adder2()->get_output_port_base(OutputPortIndex(0));
  const DependencyTracker& adder2_oport0_tracker =
      adder2_subcontext.get_tracker(adder2_oport0.ticket());

  // 2. Diagram's output port 1 is exported from adder2's output port 0.
  EXPECT_TRUE(adder2_oport0_tracker.HasSubscriber(diagram_oport1_tracker));
  EXPECT_TRUE(diagram_oport1_tracker.HasPrerequisite(adder2_oport0_tracker));

  const InputPortBase& diagram_iport2 =
      diagram_->get_input_port_base(InputPortIndex(2));
  const DependencyTracker& diagram_iport2_tracker =
      context_->get_tracker(diagram_iport2.ticket());
  const InputPortBase& adder1_iport1 =
      diagram_->adder1()->get_input_port_base(InputPortIndex(1));
  const DependencyTracker& adder1_iport1_tracker =
      adder1_subcontext.get_tracker(adder1_iport1.ticket());

  // 3. Diagram's input port 2 is imported to adder1's input port 1.
  EXPECT_TRUE(diagram_iport2_tracker.HasSubscriber(adder1_iport1_tracker));
  EXPECT_TRUE(adder1_iport1_tracker.HasPrerequisite(diagram_iport2_tracker));

  // 4. Diagrams q, v, z, xd, xa state trackers and pn, pa parameter trackers
  //    subscribe to the corresponding leaf trackers.
  auto systems = diagram_->GetSystems();
  for (auto subsystem : systems) {
    const auto& subcontext =
        diagram_->GetSubsystemContext(*subsystem, *context_);
    // Not checking "HasSubscriber" separately to cut down on fluff;
    // prerequisite and subscriber are set in the same call and checked above.
    EXPECT_TRUE(context_->get_tracker(diagram_->q_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->q_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->v_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->v_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->z_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->z_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->xd_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->xd_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->xa_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->xa_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->pn_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->pn_ticket())));
    EXPECT_TRUE(context_->get_tracker(diagram_->pa_ticket())
        .HasPrerequisite(subcontext.get_tracker(subsystem->pa_ticket())));
  }
}

TEST_F(DiagramTest, Path) {
  const std::string path = adder0()->GetSystemPathname();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);

  // Just the root.
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!",
            diagram_->GetSystemPathname());
}

TEST_F(DiagramTest, Graphviz) {
  const std::string id = std::to_string(
      reinterpret_cast<int64_t>(diagram_.get()));
  const std::string dot = diagram_->GetGraphvizString();
  // Check that the Diagram is labeled with its name.
  EXPECT_NE(std::string::npos, dot.find(
      "label=\"Unicode Snowman's Favorite Diagram!!1!☃!\";")) << dot;
  // Check that input ports are declared in blue, and output ports in green.
  EXPECT_NE(std::string::npos, dot.find(
    "_" + id + "_u1[color=blue, label=\"adder0\"")) << dot;
  EXPECT_NE(std::string::npos, dot.find(
    "_" + id + "_y1[color=green, label=\"adder2\"")) << dot;
  // Check that subsystem records appear.
  EXPECT_NE(
      std::string::npos,
      dot.find(
          "[shape=record, label=\"adder1|{{<u0>u0|<u1>u1} | {<y0>sum}}\"]"))
      << dot;
  // Check that internal edges appear.
  const std::string adder1_id = std::to_string(
      reinterpret_cast<int64_t>(diagram_->adder1()));
  const std::string adder2_id = std::to_string(
      reinterpret_cast<int64_t>(diagram_->adder2()));
  // [Adder 1, output 0] -> [Adder 2, input 1]
  EXPECT_NE(std::string::npos,
            dot.find(adder1_id + ":y0 -> " + adder2_id + ":u1;")) << dot;
  // Check that synthetic I/O edges appear: inputs in blue, outputs in green.
  // [Diagram Input 2] -> [Adder 1, input 1]
  EXPECT_NE(std::string::npos, dot.find(
      "_" + id + "_u2 -> " + adder1_id + ":u1 [color=blue];")) << dot;
  // [Diagram Input 2] -> [Sink, input]
  const std::string sink_id = std::to_string(
      reinterpret_cast<int64_t>(diagram_->sink()));
  EXPECT_NE(std::string::npos, dot.find(
      "_" + id + "_u2 -> " + sink_id + ":u0 [color=blue];")) << dot;
  // [Adder 2, output 0] -> [Diagram Output 1]
  EXPECT_NE(std::string::npos, dot.find(
      adder2_id + ":y0 -> _" + id + "_y1 [color=green];")) << dot;
}

// Tests that both variants of GetMutableSubsystemState do what they say on
// the tin.
TEST_F(DiagramTest, GetMutableSubsystemState) {
  State<double>& state_from_context = diagram_->GetMutableSubsystemState(
      *diagram_->integrator0(), context_.get());
  State<double>& state_from_state = diagram_->GetMutableSubsystemState(
      *diagram_->integrator0(), &context_->get_mutable_state());

  EXPECT_EQ(&state_from_context, &state_from_state);
  const ContinuousState<double>& xc =
      state_from_context.get_continuous_state();
  EXPECT_EQ(3, xc[0]);
  EXPECT_EQ(9, xc[1]);
  EXPECT_EQ(27, xc[2]);
}

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, CalcOutput) {
  AttachInputs();
  diagram_->CalcOutput(*context_, output_.get());

  ASSERT_EQ(kSize, output_->num_ports());
  ExpectDefaultOutputs();
}

// Tests that Diagram output ports are built with the right prerequisites
// and that Eval caches as expected.
TEST_F(DiagramTest, EvalOutput) {
  AttachInputs();

  // Sanity check output port prerequisites. Diagram ports should designate
  // a subsystem since they are resolved internally. We don't know the right
  // dependency ticket, but at least it should be valid.
  ASSERT_EQ(diagram_->num_output_ports(), 3);
  const int expected_subsystem[] = {1, 2, 5};  // From drawing above.
  for (OutputPortIndex i(0); i < diagram_->num_output_ports(); ++i) {
    internal::OutputPortPrerequisite prereq =
        diagram_->get_output_port(i).GetPrerequisite();
    EXPECT_EQ(*prereq.child_subsystem, expected_subsystem[i]);
    EXPECT_TRUE(prereq.dependency.is_valid());
  }
}

TEST_F(DiagramTest, CalcTimeDerivatives) {
  AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();
  EXPECT_EQ(derivatives->get_system_id(), context_->get_system_id());

  diagram_->CalcTimeDerivatives(*context_, derivatives.get());

  ASSERT_EQ(18, derivatives->size());
  ASSERT_EQ(4, derivatives->get_generalized_position().size());
  ASSERT_EQ(3, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(11, derivatives->get_misc_continuous_state().size());

  // The derivative of the first integrator is A.
  const ContinuousState<double>& integrator0_xcdot =
      diagram_->GetSubsystemDerivatives(*integrator0(), *derivatives);
  EXPECT_EQ(1 + 8, integrator0_xcdot.get_vector()[0]);
  EXPECT_EQ(2 + 16, integrator0_xcdot.get_vector()[1]);
  EXPECT_EQ(4 + 32, integrator0_xcdot.get_vector()[2]);

  // The derivative of the second integrator is the state of the first.
  const ContinuousState<double>& integrator1_xcdot =
      diagram_->GetSubsystemDerivatives(*integrator1(), *derivatives);
  EXPECT_EQ(3, integrator1_xcdot.get_vector()[0]);
  EXPECT_EQ(9, integrator1_xcdot.get_vector()[1]);
  EXPECT_EQ(27, integrator1_xcdot.get_vector()[2]);
}

TEST_F(DiagramTest, ContinuousStateBelongsWithSystem) {
  AttachInputs();

  // Successfully calc using storage that was created by the system.
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();
  DRAKE_EXPECT_NO_THROW(
      diagram_->CalcTimeDerivatives(*context_, derivatives.get()));

  // Successfully calc using storage that was indirectly created by the system.
  auto temp_context = diagram_->AllocateContext();
  ContinuousState<double>& temp_xc =
      temp_context->get_mutable_continuous_state();
  DRAKE_EXPECT_NO_THROW(
      diagram_->CalcTimeDerivatives(*context_, &temp_xc));

  // Cannot ask the other_diagram to calc into storage that was created by the
  // original diagram.
  const ExampleDiagram other_diagram(kSize);
  auto other_context = other_diagram.AllocateContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      other_diagram.CalcTimeDerivatives(*other_context, derivatives.get()),
      std::logic_error,
      ".*::ContinuousState<double> was not created for.*::ExampleDiagram.*");
}

// Tests the AllocateInput logic.
TEST_F(DiagramTest, AllocateInputs) {
  auto context = diagram_->CreateDefaultContext();

  for (int port = 0; port < 3; port++) {
    EXPECT_FALSE(diagram_->get_input_port(port).HasValue(*context));
  }

  diagram_->AllocateFixedInputs(context.get());

  for (int port = 0; port < 3; port++) {
    EXPECT_TRUE(diagram_->get_input_port(port).HasValue(*context));
    EXPECT_EQ(diagram_->get_input_port(port).Eval(*context).size(), kSize);
  }
}

TEST_F(DiagramTest, GetSubsystemByName) {
  const System<double>& stateless = diagram_->GetSubsystemByName("stateless");
  EXPECT_NE(
      dynamic_cast<const analysis_test::StatelessSystem<double>*>(&stateless),
      nullptr);

  DRAKE_EXPECT_THROWS_MESSAGE(
      diagram_->GetSubsystemByName("not_a_subsystem"), std::logic_error,
      "System .* does not have a subsystem named not_a_subsystem");
}

// Tests that ContextBase methods for affecting cache behavior propagate
// through to subcontexts. Since leaf output ports have cache entries in their
// corresponding subcontext, we'll pick one and check its behavior.
TEST_F(DiagramTest, CachingChangePropagation) {
  AttachInputs();  // So we can Eval() below.
  const Context<double>& adder1_subcontext =
      diagram_->GetSubsystemContext(*diagram_->adder1(), *context_);
  const OutputPort<double>& adder1_output =
      diagram_->adder1()->get_output_port();
  auto& adder1_leaf_output =
      dynamic_cast<const LeafOutputPort<double>&>(adder1_output);

  auto& cache_entry = adder1_leaf_output.cache_entry();
  EXPECT_FALSE(cache_entry.is_cache_entry_disabled(adder1_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(adder1_subcontext));

  context_->DisableCaching();
  EXPECT_TRUE(cache_entry.is_cache_entry_disabled(adder1_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(adder1_subcontext));

  context_->EnableCaching();
  EXPECT_FALSE(cache_entry.is_cache_entry_disabled(adder1_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(adder1_subcontext));

  // Bring the cache entry up to date.
  cache_entry.EvalAbstract(adder1_subcontext);
  EXPECT_FALSE(cache_entry.is_out_of_date(adder1_subcontext));

  context_->SetAllCacheEntriesOutOfDate();
  EXPECT_TRUE(cache_entry.is_out_of_date(adder1_subcontext));
}

// Tests that a diagram can be transmogrified to AutoDiffXd.
TEST_F(DiagramTest, ToAutoDiffXd) {
  std::string double_inputs;
  for (int k = 0; k < diagram_->num_input_ports(); k++) {
    double_inputs += diagram_->get_input_port(k).get_name();
    double_inputs += ",";
  }
  std::unique_ptr<System<AutoDiffXd>> ad_diagram =
      diagram_->ToAutoDiffXd();
  std::string ad_inputs;
  for (int k = 0; k < ad_diagram->num_input_ports(); k++) {
    ad_inputs += ad_diagram->get_input_port(k).get_name();
    ad_inputs += ",";
  }
  ASSERT_EQ(double_inputs, ad_inputs);
  std::unique_ptr<Context<AutoDiffXd>> context =
      ad_diagram->CreateDefaultContext();
  std::unique_ptr<SystemOutput<AutoDiffXd>> output =
      ad_diagram->AllocateOutput();

  // The name was preserved.
  EXPECT_EQ(diagram_->get_name(), ad_diagram->get_name());

  // Set up some inputs, computing gradients with respect to every other input.
  /// adder0_: (input0_ + input1_) -> A
  /// adder1_: (A + input2_)       -> B, output 0
  /// adder2_: (A + B)             -> output 1
  /// integrator1_: A              -> C
  /// integrator2_: C              -> output 2
  VectorX<AutoDiffXd> input0(3);
  VectorX<AutoDiffXd> input1(3);
  VectorX<AutoDiffXd> input2(3);
  for (int i = 0; i < 3; ++i) {
    input0[i].value() = 1 + 0.1 * i;
    input0[i].derivatives() = VectorXd::Unit(9, i);
    input1[i].value() = 2 + 0.2 * i;
    input1[i].derivatives() = VectorXd::Unit(9, 3 + i);
    input2[i].value() = 3 + 0.3 * i;
    input2[i].derivatives() = VectorXd::Unit(9, 6 + i);
  }
  ad_diagram->get_input_port(0).FixValue(context.get(), input0);
  ad_diagram->get_input_port(1).FixValue(context.get(), input1);
  ad_diagram->get_input_port(2).FixValue(context.get(), input2);

  ad_diagram->CalcOutput(*context, output.get());
  ASSERT_EQ(kSize, output->num_ports());

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

  // Make sure that the input port names survive type conversion.
  for (InputPortIndex i{0}; i < diagram_->num_input_ports(); i++) {
    EXPECT_EQ(diagram_->get_input_port(i).get_name(),
              ad_diagram->get_input_port(i).get_name());
  }

  // Make sure that the output port names survive type conversion.
  for (OutputPortIndex i{0}; i < diagram_->num_output_ports(); i++) {
    EXPECT_EQ(diagram_->get_output_port(i).get_name(),
              ad_diagram->get_output_port(i).get_name());
  }

  // When the Diagram contains a System that does not support AutoDiffXd,
  // we cannot transmogrify the Diagram to AutoDiffXd.
  const bool use_abstract = false;
  const bool use_double_only = true;
  auto diagram_with_double_only = std::make_unique<ExampleDiagram>(
      kSize, use_abstract, use_double_only);
  DRAKE_EXPECT_THROWS_MESSAGE(
      diagram_with_double_only->ToAutoDiffXd(), std::exception,
      ".*ExampleDiagram.*AutoDiffXd.*DoubleOnlySystem.*");
}

/// Tests that a diagram can be transmogrified to symbolic.
TEST_F(DiagramTest, ToSymbolic) {
  // We manually specify the template argument so that is_symbolic_convertible
  // asserts the result is merely a Diagram, not an ExampleDiagram.
  EXPECT_TRUE(is_symbolic_convertible<systems::Diagram>(*diagram_));

  // No symbolic support when one of the subsystems does not declare support.
  const bool use_abstract = false;
  const bool use_double_only = true;
  auto diagram_with_double_only = std::make_unique<ExampleDiagram>(
      kSize, use_abstract, use_double_only);
  EXPECT_THROW(diagram_with_double_only->ToSymbolic(), std::exception);
}

// Tests that the same diagram can be evaluated into the same output with
// different contexts interchangeably.
TEST_F(DiagramTest, Clone) {
  AttachInputs();

  // Compute the output with the default inputs and sanity-check it.
  diagram_->CalcOutput(*context_, output_.get());
  ExpectDefaultOutputs();

  // Verify that we start with a DiagramContext with Diagram ingredients.
  EXPECT_TRUE(is_dynamic_castable<DiagramContext<double>>(context_));
  EXPECT_TRUE(is_dynamic_castable<DiagramContinuousState<double>>(
      diagram_->AllocateTimeDerivatives()));
  EXPECT_TRUE(is_dynamic_castable<DiagramDiscreteValues<double>>(
      diagram_->AllocateDiscreteVariables()));
  const State<double>& state = context_->get_state();
  EXPECT_TRUE(is_dynamic_castable<DiagramState<double>>(&state));
  EXPECT_TRUE(is_dynamic_castable<DiagramContinuousState<double>>(
      &state.get_continuous_state()));
  EXPECT_TRUE(is_dynamic_castable<DiagramDiscreteValues<double>>(
      &state.get_discrete_state()));
  // FYI there is no DiagramAbstractValues.

  // Create a clone of the context and change an input.
  auto clone = context_->Clone();

  // Verify that the clone is also a DiagramContext with Diagram ingredients.
  EXPECT_TRUE(is_dynamic_castable<DiagramContext<double>>(clone));
  const State<double>& clone_state = context_->get_state();
  EXPECT_TRUE(is_dynamic_castable<DiagramState<double>>(&clone_state));
  EXPECT_TRUE(is_dynamic_castable<DiagramContinuousState<double>>(
      &clone_state.get_continuous_state()));
  EXPECT_TRUE(is_dynamic_castable<DiagramDiscreteValues<double>>(
      &clone_state.get_discrete_state()));

  DRAKE_DEMAND(kSize == 3);
  diagram_->get_input_port(0).FixValue(clone.get(), Vector3d(3, 6, 9));

  // Recompute the output and check the values.
  diagram_->CalcOutput(*clone, output_.get());
  Vector3d expected_output0(
      3 + 8 + 64,
      6 + 16 + 128,
      9 + 32 + 256);  // B
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Vector3d expected_output1(
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
            diagram_->GetSubsystemDerivatives(*adder0(), *derivatives).size());
}

// Tests that, when asked for the discrete state of Systems that are
// stateless, Diagram returns an empty state.
TEST_F(DiagramTest, DiscreteValuesOfStatelessSystemAreEmpty) {
  std::unique_ptr<DiscreteValues<double>> updates =
      diagram_->AllocateDiscreteVariables();
  EXPECT_EQ(
      0,
      diagram_->GetSubsystemDiscreteValues(*adder0(), *updates).num_groups());
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

    // Make sure caching is on locally, even if it is off by default.
    // Do not remove this line; we want to show that these tests function
    // correctly with caching on. It is easier to pass with caching off!
    context_->EnableCaching();

    output_ = diagram_->AllocateOutput();

    diagram_->get_input_port(0).FixValue(context_.get(), 8.0);
    diagram_->get_input_port(1).FixValue(context_.get(), 64.0);
    diagram_->get_input_port(2).FixValue(context_.get(), 512.0);

    // Initialize the integrator states.
    Context<double>& d0_context =
        diagram_->GetMutableSubsystemContext(*subdiagram0_, context_.get());
    Context<double>& d1_context =
        diagram_->GetMutableSubsystemContext(*subdiagram1_, context_.get());

    State<double>& integrator0_x = subdiagram0_->GetMutableSubsystemState(
        *subdiagram0_->integrator0(), &d0_context);
    integrator0_x.get_mutable_continuous_state()
        .get_mutable_vector()[0] = 3;

    State<double>& integrator1_x = subdiagram0_->GetMutableSubsystemState(
        *subdiagram0_->integrator1(), &d0_context);
    integrator1_x.get_mutable_continuous_state()
        .get_mutable_vector()[0] = 9;

    State<double>& integrator2_x = subdiagram1_->GetMutableSubsystemState(
        *subdiagram1_->integrator0(), &d1_context);
    integrator2_x.get_mutable_continuous_state()
        .get_mutable_vector()[0] = 27;

    State<double>& integrator3_x = subdiagram1_->GetMutableSubsystemState(
        *subdiagram1_->integrator1(), &d1_context);
    integrator3_x.get_mutable_continuous_state()
        .get_mutable_vector()[0] = 81;
  }

  const int kSize = 1;

  std::unique_ptr<Diagram<double>> diagram_ = nullptr;
  ExampleDiagram* subdiagram0_ = nullptr;
  ExampleDiagram* subdiagram1_ = nullptr;

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Now we can check that the nested Diagram reports correct declared sizes.
// It is sufficient to check the Diagram declared sizes with the actual
// Context sizes since we verify elsewhere that Diagrams produce correct
// Contexts.
TEST_F(DiagramOfDiagramsTest, DeclaredContextSizes) {
  EXPECT_EQ(diagram_->num_continuous_states(),
            context_->num_continuous_states());
  EXPECT_EQ(diagram_->num_discrete_state_groups(),
            context_->num_discrete_state_groups());
  EXPECT_EQ(diagram_->num_abstract_states(),
            context_->num_abstract_states());
  EXPECT_EQ(diagram_->num_numeric_parameter_groups(),
            context_->num_numeric_parameter_groups());
  EXPECT_EQ(diagram_->num_abstract_parameters(),
            context_->num_abstract_parameters());
}

// ContextSizes for a Diagram must be accumulated recursively. We checked
// above that a Diagram built using DiagramBuilder counts properly. Diagrams
// can also be built via scalar conversion. We'll check here that the
// context sizes are correct that way also.
TEST_F(DiagramOfDiagramsTest, ScalarConvertAndCheckContextSizes) {
  Diagram<AutoDiffXd> diagram_ad(*diagram_);

  // Anticipated context sizes should be unchanged from the original.
  EXPECT_EQ(diagram_ad.num_continuous_states(),
            context_->num_continuous_states());
  EXPECT_EQ(diagram_ad.num_discrete_state_groups(),
            context_->num_discrete_state_groups());
  EXPECT_EQ(diagram_ad.num_abstract_states(),
            context_->num_abstract_states());
  EXPECT_EQ(diagram_ad.num_numeric_parameter_groups(),
            context_->num_numeric_parameter_groups());
  EXPECT_EQ(diagram_ad.num_abstract_parameters(),
            context_->num_abstract_parameters());
}

TEST_F(DiagramOfDiagramsTest, Graphviz) {
  const std::string dot = diagram_->GetGraphvizString();
  // Check that both subdiagrams appear.
  EXPECT_NE(std::string::npos, dot.find("label=\"subdiagram0\""));
  EXPECT_NE(std::string::npos, dot.find("label=\"subdiagram1\""));
  // Check that edges between the two subdiagrams exist.
  const std::string id0 = std::to_string(
      reinterpret_cast<int64_t>(subdiagram0_));
  const std::string id1 = std::to_string(
      reinterpret_cast<int64_t>(subdiagram1_));
  EXPECT_NE(std::string::npos, dot.find("_" + id0 + "_y0 -> _" + id1 + "_u0"));

  const int max_depth = 0;
  const std::string dot_no_depth = diagram_->GetGraphvizString(max_depth);
  // Check that the subdiagrams no longer appear.
  EXPECT_EQ(std::string::npos, dot_no_depth.find("label=\"subdiagram0\""));
  EXPECT_EQ(std::string::npos, dot_no_depth.find("label=\"subdiagram1\""));
}

// Tests that a diagram composed of diagrams can be evaluated, and gives
// correct answers with caching enabled.
TEST_F(DiagramOfDiagramsTest, EvalOutput) {
  context_->EnableCaching();  // Just to be sure.

  EXPECT_EQ(diagram_->get_input_port(0).Eval(*context_)[0], 8.);
  EXPECT_EQ(diagram_->get_input_port(1).Eval(*context_)[0], 64.);
  EXPECT_EQ(diagram_->get_input_port(2).Eval(*context_)[0], 512.);

  // The outputs of subsystem0_ are:
  //   output0 = 8 + 64 + 512 = 584
  //   output1 = output0 + 8 + 64 = 656
  //   output2 = 9 (state of integrator1_)

  // So, the outputs of subsystem1_, and thus of the whole diagram, are:
  //   output0 = 584 + 656 + 9 = 1249
  //   output1 = output0 + 584 + 656 = 2489
  //   output2 = 81 (state of integrator1_)
  EXPECT_EQ(1249, diagram_->get_output_port(0).Eval(*context_)[0]);
  EXPECT_EQ(2489, diagram_->get_output_port(1).Eval(*context_)[0]);
  EXPECT_EQ(81, diagram_->get_output_port(2).Eval(*context_)[0]);

  // Check that invalidation flows through input ports properly, either due
  // to replacing the fixed value or by modifying it.
  //
  // First we'll replace the fixed input value 8 for input port 0 with
  // a new object that has value 10. That should cause everything to get
  // recalculated when the ports are Eval()-ed.
  // The outputs of subsystem0_ are now:
  //   output0 = 10 + 64 + 512 = 586
  //   output1 = output0 + 10 + 64 = 660
  //   output2 = 9 (state of integrator1_)

  // So, the outputs of subsystem1_, and thus of the whole diagram, are:
  //   output0 = 586 + 660 + 9 = 1255
  //   output1 = output0 + 586 + 660 = 2501
  //   output2 = 81 (state of integrator1_)
  FixedInputPortValue& port_value =
      diagram_->get_input_port(0).FixValue(context_.get(), 10.0);
  EXPECT_EQ(1255, diagram_->get_output_port(0).
      Eval<BasicVector<double>>(*context_)[0]);
  EXPECT_EQ(2501, diagram_->get_output_port(1).
      Eval<BasicVector<double>>(*context_)[0]);
  EXPECT_EQ(81, diagram_->get_output_port(2).
      Eval<BasicVector<double>>(*context_)[0]);

  // Now change the value back to 8 using mutable access to the port_value
  // object. Should also cause re-evaluation.
  port_value.GetMutableVectorData<double>()->SetAtIndex(0, 8.0);
  EXPECT_EQ(1249, diagram_->get_output_port(0).
      Eval<BasicVector<double>>(*context_)[0]);
  EXPECT_EQ(2489, diagram_->get_output_port(1).
      Eval<BasicVector<double>>(*context_)[0]);
  EXPECT_EQ(81, diagram_->get_output_port(2).
      Eval<BasicVector<double>>(*context_)[0]);
}

TEST_F(DiagramOfDiagramsTest, DirectFeedthrough) {
  // The diagram has direct feedthrough.
  EXPECT_TRUE(diagram_->HasAnyDirectFeedthrough());
  // Specifically, outputs 0 and 1 have direct feedthrough, but not output 2.
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(0));
  EXPECT_TRUE(diagram_->HasDirectFeedthrough(1));
  EXPECT_FALSE(diagram_->HasDirectFeedthrough(2));
  // Specifically, outputs 0 and 1 have direct feedthrough from all inputs.
  for (int i = 0; i < kSize; ++i) {
    EXPECT_TRUE(diagram_->HasDirectFeedthrough(i, 0));
    EXPECT_TRUE(diagram_->HasDirectFeedthrough(i, 1));
    EXPECT_FALSE(diagram_->HasDirectFeedthrough(i, 2));
  }
}

// A Diagram that adds a constant to an input, and outputs the sum.
class AddConstantDiagram : public Diagram<double> {
 public:
  explicit AddConstantDiagram(double constant) : Diagram<double>() {
    DiagramBuilder<double> builder;

    constant_ = builder.AddSystem<ConstantVectorSource>(Vector1d{constant});
    constant_->set_name("constant");
    adder_ = builder.AddSystem<Adder>(2 /* inputs */, 1 /* size */);
    adder_->set_name("adder");

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
  auto output = plus_seven.AllocateOutput();
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(output != nullptr);

  plus_seven.get_input_port(0).FixValue(context.get(), 12.0);
  plus_seven.CalcOutput(*context, output.get());

  ASSERT_EQ(1, output->num_ports());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(1, output_vector->size());
  EXPECT_EQ(19.0, output_vector->get_value().x());
}

// PublishingSystem has an input port for a single double. It publishes that
// double to a function provided in the constructor.
class PublishingSystem : public LeafSystem<double> {
 public:
  explicit PublishingSystem(std::function<void(double)> callback)
      : callback_(callback) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 1);
    this->DeclareForcedPublishEvent(&PublishingSystem::InvokeCallback);
  }

 private:
  // TODO(15465) When forced event declaration sugar allows for callbacks
  // whose result is "assumed to succeed", change this to void return.
  EventStatus InvokeCallback(const Context<double>& context) const {
    callback_(this->get_input_port(0).Eval(context)[0]);
    return EventStatus::Succeeded();
  }

  std::function<void(int)> callback_;
};

// PublishNumberDiagram publishes a double provided to its constructor.
class PublishNumberDiagram : public Diagram<double> {
 public:
  explicit PublishNumberDiagram(double constant) : Diagram<double>() {
    DiagramBuilder<double> builder;

    constant_ =
        builder.AddSystem<ConstantVectorSource<double>>(Vector1d{constant});
    constant_->set_name("constant");
    publisher_ =
        builder.AddSystem<PublishingSystem>([this](double v) { this->set(v); });
    publisher_->set_name("publisher");

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
//
// (N.B. Normally a class that supports scalar conversion, but does not offer a
// SystemScalarConverter-accepting constructor would be marked `final`, but we
// leave the `final` off here to test what happens during wrong-subclassing.)
template <typename T>
class FeedbackDiagram : public Diagram<T> {
 public:
  FeedbackDiagram()
      // We choose this constructor from our parent class so that we have a
      // useful Diagram for the SubclassTransmogrificationTest.
      : Diagram<T>(SystemTypeTag<FeedbackDiagram>{}) {
    constexpr int kSize = 1;

    DiagramBuilder<T> builder;

    DiagramBuilder<T> integrator_builder;
    Integrator<T>* const integrator =
        integrator_builder.template AddSystem<Integrator>(kSize);
    integrator->set_name("integrator");
    integrator_builder.ExportInput(integrator->get_input_port());
    integrator_builder.ExportOutput(integrator->get_output_port());
    Diagram<T>* const integrator_diagram =
        builder.AddSystem(integrator_builder.Build());
    integrator_diagram->set_name("integrator_diagram");

    DiagramBuilder<T> gain_builder;
    Gain<T>* const gain =
        gain_builder.template AddSystem<Gain>(1.0 /* gain */, kSize);
    gain->set_name("gain");
    gain_builder.ExportInput(gain->get_input_port());
    gain_builder.ExportOutput(gain->get_output_port());
    Diagram<T>* const gain_diagram =
        builder.AddSystem(gain_builder.Build());
    gain_diagram->set_name("gain_diagram");

    builder.Connect(*integrator_diagram, *gain_diagram);
    builder.Connect(*gain_diagram, *integrator_diagram);
    builder.BuildInto(this);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit FeedbackDiagram(const FeedbackDiagram<U>& other)
      : Diagram<T>(other) {}
};

// Tests that since there are no outputs, there is no direct feedthrough.
GTEST_TEST(FeedbackDiagramTest, HasDirectFeedthrough) {
  FeedbackDiagram<double> diagram;
  EXPECT_FALSE(diagram.HasAnyDirectFeedthrough());
}

// Tests that a FeedbackDiagram's context can be deleted without accessing
// already-freed memory. https://github.com/RobotLocomotion/drake/issues/3349
GTEST_TEST(FeedbackDiagramTest, DeletionIsMemoryClean) {
  FeedbackDiagram<double> diagram;
  auto context = diagram.CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(context.reset());
}

// If a SystemScalarConverter is passed into the Diagram constructor, then
// transmogrification will preserve the subtype.
TEST_F(DiagramTest, SubclassTransmogrificationTest) {
  const FeedbackDiagram<double> dut;
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [](const auto& converted) {
    EXPECT_FALSE(converted.HasAnyDirectFeedthrough());
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [](const auto& converted) {
    EXPECT_FALSE(converted.HasAnyDirectFeedthrough());
  }));

  // Diagram subclasses that declare a specific SystemTypeTag but then use a
  // subclass at runtime will fail-fast.
  class SubclassOfFeedbackDiagram : public FeedbackDiagram<double> {};
  const SubclassOfFeedbackDiagram subclass_dut{};
  EXPECT_THROW(({
    try {
      subclass_dut.ToAutoDiffXd();
    } catch (const std::runtime_error& e) {
      EXPECT_THAT(
          std::string(e.what()),
          testing::MatchesRegex(
              ".*convert a .*::FeedbackDiagram<double>.* called with a"
              ".*::SubclassOfFeedbackDiagram at runtime"));
      throw;
    }
  }), std::runtime_error);
}

// A simple class that consumes *two* inputs and passes one input through. The
// sink input (S) is consumed, the feed through input (F) is passed through to
// the output (O). This system has direct feedthrough, but only with respect to
// one input. Support class for the PortDependentFeedthroughTest.
//      +-----------+
//      +--+        |
//      +S |        |
//      +--+     +--+
//      |     +->+O |
//      +--+  |  +--+
//      |F +--+     |
//      +--+        |
//      +-----------+
class Reduce : public LeafSystem<double> {
 public:
  Reduce() {
    const auto& input0 = this->DeclareInputPort(
        kUseDefaultName, kVectorValued, 1);
    feedthrough_input_ = input0.get_index();
    sink_input_ = this->DeclareInputPort(
        kUseDefaultName, kVectorValued, 1).get_index();
    this->DeclareVectorOutputPort(kUseDefaultName, 1, &Reduce::CalcFeedthrough,
                                  {input0.ticket()});
  }

  const systems::InputPort<double>& get_sink_input() const {
    return this->get_input_port(sink_input_);
  }

  const systems::InputPort<double>& get_feedthrough_input() const {
    return this->get_input_port(feedthrough_input_);
  }

  void CalcFeedthrough(const Context<double>& context,
                       BasicVector<double>* output) const {
    const auto& input = get_feedthrough_input().Eval(context);
    output->get_mutable_value() = input;
  }

 private:
  int feedthrough_input_{-1};
  int sink_input_{-1};
};

// Diagram input and output are connected by a system that has direct
// feedthrough, but the path between diagram input and output doesn't follow
// this path.
GTEST_TEST(PortDependentFeedthroughTest, DetectNoFeedthrough) {
// This is a diagram that wraps a Reduce Leaf system, exporting the output
// and the *sink* input. There should be *no* direct feedthrough on the diagram.
//        +-----------------+
//        |                 |
//        |  +-----------+  |
//        |  +--+        |  |
//   I +---->+S |        |  |
//        |  +--+     +--+  |
//        |  |     +->+O +------> O
//        |  +--+  |  +--+  |
//        |  |F +--+     |  |
//        |  +--+        |  |
//        |  +-----------+  |
//        |                 |
//        +-----------------+
  DiagramBuilder<double> builder;
  auto reduce = builder.AddSystem<Reduce>();
  builder.ExportInput(reduce->get_sink_input());
  builder.ExportOutput(reduce->get_output_port(0));
  auto diagram = builder.Build();
  EXPECT_FALSE(diagram->HasAnyDirectFeedthrough());
  EXPECT_FALSE(diagram->HasDirectFeedthrough(0));
  EXPECT_FALSE(diagram->HasDirectFeedthrough(0, 0));
}

// Diagram input and output are connected by a system that has direct
// feedthrough, and the input and output *are* connected by that path.
GTEST_TEST(PortDependentFeedthroughTest, DetectFeedthrough) {
// This is a diagram that wraps a Reduce Leaf system, exporting the output
// and the *feedthrough* input. There should be direct feedthrough on the
// diagram.
//        +-----------------+
//        |                 |
//        |  +-----------+  |
//        |  +--+        |  |
//        |  |S |        |  |
//        |  +--+     +--+  |
//        |  |     +->+O +------> O
//        |  +--+  |  +--+  |
//   I +---->|F +--+     |  |
//        |  +--+        |  |
//        |  +-----------+  |
//        |                 |
//        +-----------------+
  DiagramBuilder<double> builder;
  auto reduce = builder.AddSystem<Reduce>();
  builder.ExportInput(reduce->get_feedthrough_input());
  builder.ExportOutput(reduce->get_output_port(0));
  auto diagram = builder.Build();
  EXPECT_TRUE(diagram->HasAnyDirectFeedthrough());
  EXPECT_TRUE(diagram->HasDirectFeedthrough(0));
  EXPECT_TRUE(diagram->HasDirectFeedthrough(0, 0));
}

// A system with a random source inputs.
class RandomInputSystem : public LeafSystem<double> {
 public:
  RandomInputSystem() {
    this->DeclareInputPort("deterministic", kVectorValued, 1);
    this->DeclareInputPort("uniform", kVectorValued, 1,
                           RandomDistribution::kUniform);
    this->DeclareInputPort("gaussian", kVectorValued, 1,
                           RandomDistribution::kGaussian);
  }
};

GTEST_TEST(RandomInputSystemTest, RandomInputTest) {
  DiagramBuilder<double> builder;
  auto random = builder.AddSystem<RandomInputSystem>();
  builder.ExportInput(random->get_input_port(0));
  builder.ExportInput(random->get_input_port(1));
  builder.ExportInput(random->get_input_port(2));
  auto diagram = builder.Build();
  EXPECT_FALSE(diagram->get_input_port(0).get_random_type());
  EXPECT_EQ(diagram->get_input_port(1).get_random_type(),
            RandomDistribution::kUniform);
  EXPECT_EQ(diagram->get_input_port(2).get_random_type(),
            RandomDistribution::kGaussian);
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
  [[nodiscard]] SecondOrderStateVector* DoClone() const override {
    return new SecondOrderStateVector;
  }
};

// A minimal system that has second-order state.
class SecondOrderStateSystem : public LeafSystem<double> {
 public:
  SecondOrderStateSystem() {
    DeclareInputPort(kUseDefaultName, kVectorValued, 1);
    DeclareContinuousState(SecondOrderStateVector{},
                           1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  }

  SecondOrderStateVector* x(Context<double>* context) const {
    return dynamic_cast<SecondOrderStateVector*>(
        &context->get_mutable_continuous_state_vector());
  }

 protected:
  // qdot = 2 * v.
  void DoMapVelocityToQDot(
      const Context<double>& context,
      const Eigen::Ref<const VectorX<double>>& generalized_velocity,
      VectorBase<double>* qdot) const override {
    qdot->SetAtIndex(0, 2 * generalized_velocity[0]);
  }

  // v = 1/2 * qdot.
  void DoMapQDotToVelocity(
      const Context<double>& context,
      const Eigen::Ref<const VectorX<double>>& qdot,
      VectorBase<double>* generalized_velocity) const override {
    generalized_velocity->SetAtIndex(0, 0.5 * qdot[0]);
  }
};

// A diagram that has second-order state.
class SecondOrderStateDiagram : public Diagram<double> {
 public:
  SecondOrderStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    sys1_ = builder.template AddSystem<SecondOrderStateSystem>();
    sys1_->set_name("sys1");
    sys2_ = builder.template AddSystem<SecondOrderStateSystem>();
    sys2_->set_name("sys2");
    builder.ExportInput(sys1_->get_input_port(0));
    builder.ExportInput(sys2_->get_input_port(0));
    builder.BuildInto(this);
  }

  SecondOrderStateSystem* sys1() { return sys1_; }
  SecondOrderStateSystem* sys2() { return sys2_; }

  // Returns the state of the given subsystem.
  SecondOrderStateVector* x(Context<double>* context,
                            const SecondOrderStateSystem* subsystem) {
    Context<double>& subsystem_context =
        GetMutableSubsystemContext(*subsystem, context);
    return subsystem->x(&subsystem_context);
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
  const VectorBase<double>& v =
      context->get_continuous_state().get_generalized_velocity();
  diagram.MapVelocityToQDot(*context, v, &qdot);

  // The order of these derivatives is defined to be the same as the order
  // subsystems are added.
  EXPECT_EQ(qdot[0], 26);
  EXPECT_EQ(qdot[1], 34);

  // Now map the configuration derivatives back to v.
  BasicVector<double> vmutable(v.size());
  diagram.MapQDotToVelocity(*context, qdot, &vmutable);
  EXPECT_EQ(vmutable[0], 13);
  EXPECT_EQ(vmutable[1], 17);
}

// Test for GetSystems.
GTEST_TEST(GetSystemsTest, GetSystems) {
  auto diagram = std::make_unique<ExampleDiagram>(2);
  EXPECT_EQ((std::vector<const System<double>*>{
                diagram->adder0(), diagram->adder1(), diagram->adder2(),
                diagram->stateless(),
                diagram->integrator0(), diagram->integrator1(),
                diagram->sink(),
                diagram->kitchen_sink()
            }),
            diagram->GetSystems());
}

const double kTestPublishPeriod = 19.0;

class TestPublishingSystem final : public LeafSystem<double> {
 public:
  TestPublishingSystem() {
    this->DeclarePeriodicPublishEvent(
        kTestPublishPeriod, 0.0, &TestPublishingSystem::HandlePeriodPublish);
    this->DeclarePeriodicPublish(kTestPublishPeriod);

    // Verify that no periodic discrete updates are registered.
    EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
  }

  bool published() const { return published_; }

 private:
  void HandlePeriodPublish(const Context<double>&) const {
    published_ = true;
  }

  // Recording state through event handlers, like this system does, is an
  // anti-pattern, but we do it to simplify testing.
  mutable bool published_{false};
};

// A diagram that has discrete state and publishers.
class DiscreteStateDiagram : public Diagram<double> {
 public:
  DiscreteStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    hold1_ = builder.template AddSystem<ZeroOrderHold<double>>(2.0, kSize);
    hold1_->set_name("hold1");
    hold2_ = builder.template AddSystem<ZeroOrderHold<double>>(3.0, kSize);
    hold2_->set_name("hold2");
    publisher_ = builder.template AddSystem<TestPublishingSystem>();
    publisher_->set_name("publisher");
    builder.ExportInput(hold1_->get_input_port());
    builder.ExportInput(hold2_->get_input_port());
    builder.BuildInto(this);
    EXPECT_FALSE(IsDifferenceEquationSystem());
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

class ForcedPublishingSystem : public LeafSystem<double> {
 public:
  ForcedPublishingSystem() {
    this->DeclareForcedPublishEvent(
        &ForcedPublishingSystem::PublishHandler);
  }
  bool published() const { return published_; }

 private:
  // TODO(15465) When forced event declaration sugar allows for callbacks
  // whose result is "assumed to succeed", change this to void return.
  EventStatus PublishHandler(const Context<double>& context) const {
    published_ = true;
    return EventStatus::Succeeded();
  }

  // Recording state through event handlers, like this system does, is an
  // anti-pattern, but we do it to simplify testing.
  mutable bool published_{false};
};

// A diagram that consists of only forced publishing systems.
class ForcedPublishingSystemDiagram : public Diagram<double> {
 public:
  ForcedPublishingSystemDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    publishing_system_one_ =
        builder.template AddSystem<ForcedPublishingSystem>();
    publishing_system_two_ =
        builder.template AddSystem<ForcedPublishingSystem>();
    builder.BuildInto(this);
  }
  ForcedPublishingSystem* publishing_system_one() const {
      return publishing_system_one_;
  }
  ForcedPublishingSystem* publishing_system_two() const {
      return publishing_system_two_;
  }

 private:
  ForcedPublishingSystem* publishing_system_one_{nullptr};
  ForcedPublishingSystem* publishing_system_two_{nullptr};
};

class DiscreteStateTest : public ::testing::Test {
 public:
  void SetUp() override {
    context_ = diagram_.CreateDefaultContext();
    diagram_.get_input_port(0).FixValue(context_.get(), 17.0);
    diagram_.get_input_port(1).FixValue(context_.get(), 23.0);
  }

 protected:
  DiscreteStateDiagram diagram_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that the next update time after 0.05 is 2.0.
TEST_F(DiscreteStateTest, CalcNextUpdateTimeHold1) {
  context_->SetTime(0.05);
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());

  EXPECT_EQ(2.0, time);
  const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            *diagram_.hold1(), *events);

  EXPECT_TRUE(subevent_collection.get_discrete_update_events().HasEvents());
}

// Tests that the next update time after 5.1 is 6.0.
TEST_F(DiscreteStateTest, CalcNextUpdateTimeHold2) {
  context_->SetTime(5.1);
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());

  EXPECT_EQ(6.0, time);
  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            *diagram_.hold2(), *events);
    EXPECT_TRUE(subevent_collection.get_discrete_update_events().HasEvents());
  }

  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            *diagram_.hold1(), *events);
    EXPECT_TRUE(subevent_collection.get_discrete_update_events().HasEvents());
  }
}

// Tests that on the 9-second tick, only hold2 latches its inputs. Then, on
// the 12-second tick, both hold1 and hold2 latch their inputs.
TEST_F(DiscreteStateTest, UpdateDiscreteVariables) {
  // Initialize the zero-order holds to different values than their input ports.
  Context<double>& ctx1 =
      diagram_.GetMutableSubsystemContext(*diagram_.hold1(), context_.get());
  ctx1.get_mutable_discrete_state(0)[0] = 1001.0;
  Context<double>& ctx2 =
      diagram_.GetMutableSubsystemContext(*diagram_.hold2(), context_.get());
  ctx2.get_mutable_discrete_state(0)[0] = 1002.0;

  // Allocate the discrete variables.
  std::unique_ptr<DiscreteValues<double>> updates =
      diagram_.AllocateDiscreteVariables();
  EXPECT_EQ(updates->get_system_id(), context_->get_system_id());
  const DiscreteValues<double>& updates1 =
      diagram_
          .GetSubsystemDiscreteValues(*diagram_.hold1(), *updates);
  const DiscreteValues<double>& updates2 =
      diagram_
          .GetSubsystemDiscreteValues(*diagram_.hold2(), *updates);
  updates->get_mutable_vector(0)[0] = 0.0;  // Start with known values.
  updates->get_mutable_vector(1)[0] = 0.0;

  // Set the time to 8.5, so only hold2 updates.
  context_->SetTime(8.5);

  // Request the next update time. Note that CalcNextUpdateTime() clears the
  // event collection.
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(9.0, time);
  EXPECT_TRUE(events->HasDiscreteUpdateEvents());

  // Fast forward to 9.0 sec and do the update.
  context_->SetTime(9.0);
  diagram_.CalcDiscreteVariableUpdates(
      *context_, events->get_discrete_update_events(), updates.get());

  // Note that non-participating hold1's state should not have been
  // copied (if it had been it would be 1001.0).
  EXPECT_EQ(0.0, updates1[0]);  // Same as we started with.
  EXPECT_EQ(23.0, updates2[0]);  // Updated.

  // Apply the updates to the context_.
  diagram_.ApplyDiscreteVariableUpdate(events->get_discrete_update_events(),
      updates.get(), context_.get());
  EXPECT_EQ(1001.0, ctx1.get_discrete_state(0)[0]);
  EXPECT_EQ(23.0, ctx2.get_discrete_state(0)[0]);

  // Restore hold2 to its original value.
  ctx2.get_mutable_discrete_state(0)[0] = 1002.0;
  // Set the time to 11.5, so both hold1 and hold2 update.  Note that
  // CalcNextUpdateTime() clears the event collection.
  context_->SetTime(11.5);
  time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(12.0, time);
  EXPECT_TRUE(events->HasDiscreteUpdateEvents());

  // Fast forward to 12.0 sec and do the update again.
  context_->SetTime(12.0);
  diagram_.CalcDiscreteVariableUpdates(
      *context_, events->get_discrete_update_events(), updates.get());
  EXPECT_EQ(17.0, updates1[0]);
  EXPECT_EQ(23.0, updates2[0]);
}

// Tests that in a Diagram where multiple subsystems have discrete variables,
// an update in one subsystem doesn't cause invalidation in the other. Note
// that although we use the caching system to observe what gets invalidated,
// we are testing for proper Diagram behavior here; we're not testing the
// caching system (which is well-tested elsewhere).
TEST_F(DiscreteStateTest, DiscreteUpdateNotificationsAreLocalized) {
  // Initialize the zero-order holds to different values than their input ports.
  Context<double>& ctx1 =
      diagram_.GetMutableSubsystemContext(*diagram_.hold1(), context_.get());
  ctx1.get_mutable_discrete_state(0)[0] = 1001.0;
  Context<double>& ctx2 =
      diagram_.GetMutableSubsystemContext(*diagram_.hold2(), context_.get());
  ctx2.get_mutable_discrete_state(0)[0] = 1002.0;

  // Allocate space to hold the updated discrete variables.
  std::unique_ptr<DiscreteValues<double>> updates =
      diagram_.AllocateDiscreteVariables();

  // Hold1 is due for an update at 2s, so only it should be included in the
  // next update time event collection.
  context_->SetTime(1.5);

  // Request the next update time.
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(2.0, time);
  EXPECT_TRUE(events->HasDiscreteUpdateEvents());
  const auto& discrete_events = events->get_discrete_update_events();

  auto num_notifications = [](const Context<double>& context) {
    return context.get_tracker(SystemBase::xd_ticket())
        .num_notifications_received();
  };

  const int64_t notifications_1 = num_notifications(ctx1);
  const int64_t notifications_2 = num_notifications(ctx2);

  // Fast forward to 2.0 sec and collect the update.
  context_->SetTime(2.0);
  diagram_.CalcDiscreteVariableUpdates(
      *context_, discrete_events, updates.get());

  // Of course nothing should have been notified since nothing's changed yet.
  EXPECT_EQ(num_notifications(ctx1), notifications_1);
  EXPECT_EQ(num_notifications(ctx2), notifications_2);

  // Selectively apply the update; only hold1 should get notified.
  diagram_.ApplyDiscreteVariableUpdate(discrete_events, updates.get(),
                                       context_.get());
  // Sanity check that the update did occur!
  EXPECT_EQ(17.0, ctx1.get_discrete_state(0)[0]);
  EXPECT_EQ(1002.0, ctx2.get_discrete_state(0)[0]);

  EXPECT_EQ(num_notifications(ctx1), notifications_1 + 1);
  EXPECT_EQ(num_notifications(ctx2), notifications_2);

  // Now apply the updates the dumb way. Everyone gets notified.
  context_->get_mutable_discrete_state().SetFrom(*updates);
  EXPECT_EQ(num_notifications(ctx1), notifications_1 + 2);
  EXPECT_EQ(num_notifications(ctx2), notifications_2 + 1);
}

// A system with a single discrete variable that is initialized to the
// system's id as supplied at construction. A periodic update modifies the
// state in a way that lets us see whether the framework passed in the right
// state value to the handler.
class SystemWithDiscreteState : public LeafSystem<double> {
 public:
  SystemWithDiscreteState(int id, double update_period) : id_(id) {
    // Just one discrete variable, initialized to id.
    DeclareDiscreteState(Vector1d(id_));
    DeclarePeriodicDiscreteUpdateEvent(
        update_period, 0, &SystemWithDiscreteState::AddTimeToDiscreteVariable);
  }

  int get_id() const { return id_; }

 private:
  // Discrete state is set to input state value + time. We expect that the
  // initial value for discrete_state is the same as the value in context.
  void AddTimeToDiscreteVariable(
      const Context<double>& context,
      DiscreteValues<double>* discrete_state) const {
    ASSERT_EQ((*discrete_state)[0], context.get_discrete_state(0)[0]);
    (*discrete_state)[0] += context.get_time();
  }

  const int id_;
};

class TwoDiscreteSystemDiagram : public Diagram<double> {
 public:
  enum { kSys1Id = 1, kSys2Id = 2 };
  TwoDiscreteSystemDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    sys1_ = builder.template AddSystem<SystemWithDiscreteState>(kSys1Id, 2.);
    sys2_ = builder.template AddSystem<SystemWithDiscreteState>(kSys2Id, 3.);
    builder.BuildInto(this);
  }

  const SystemWithDiscreteState& get_sys(int i) const {
    DRAKE_DEMAND(i == kSys1Id || i == kSys2Id);
    return i == kSys1Id ? *sys1_ : *sys2_;
  }

 private:
  SystemWithDiscreteState* sys1_{nullptr};
  SystemWithDiscreteState* sys2_{nullptr};
};

// Check that a flat Diagram correctly aggregates the number of discrete
// state groups from its subsystems. Separately we'll check that nested
// Diagrams recurse properly to count up everything in the tree.
GTEST_TEST(DiscreteStateDiagramTest, NumDiscreteStateGroups) {
  DiagramBuilder<double> builder;
  builder.template AddSystem<SystemWithDiscreteState>(1, 2.);
  const auto diagram = builder.Build();
  const auto context = diagram->CreateDefaultContext();
  EXPECT_EQ(diagram->num_discrete_state_groups(),
            context->num_discrete_state_groups());
}

GTEST_TEST(DiscreteStateDiagramTest, IsDifferenceEquationSystem) {
  // Two unique periods, two state groups.
  DiagramBuilder<double> builder;
  builder.template AddSystem<SystemWithDiscreteState>(1, 2.);
  builder.template AddSystem<SystemWithDiscreteState>(2, 3.);
  const auto two_period_diagram = builder.Build();
  EXPECT_FALSE(two_period_diagram->IsDifferenceEquationSystem());

  // One period, one discrete state group.
  const double period = 0.1;
  DiagramBuilder<double> builder2;
  builder2.template AddSystem<SystemWithDiscreteState>(1, period);
  const auto one_period_diagram = builder2.Build();
  double test_period = -3.94;
  EXPECT_TRUE(one_period_diagram->IsDifferenceEquationSystem(&test_period));
  EXPECT_EQ(test_period, period);

  // One unique period, but two discrete state groups.
  DiagramBuilder<double> builder3;
  builder3.template AddSystem<SystemWithDiscreteState>(1, period);
  builder3.template AddSystem<SystemWithDiscreteState>(2, period);
  const auto one_period_two_state_diagram = builder3.Build();
  EXPECT_FALSE(one_period_two_state_diagram->IsDifferenceEquationSystem());
}

// Tests CalcDiscreteVariableUpdates() when there are multiple subsystems and
// only one has an event to handle (call that the "participating subsystem"). We
// want to verify that only the participating subsystem's State gets copied,
// that the copied value matches the current value in the context, and
// that the update gets performed properly. We also check that it works properly
// when multiple subsystems have events to handle.
GTEST_TEST(DiscreteStateDiagramTest, CalcDiscreteVariableUpdates) {
  TwoDiscreteSystemDiagram diagram;
  const int kSys1Id = TwoDiscreteSystemDiagram::kSys1Id;
  const int kSys2Id = TwoDiscreteSystemDiagram::kSys2Id;

  auto context = diagram.CreateDefaultContext();
  context->SetTime(1.5);

  // The discrete states should be initialized to their ids.
  EXPECT_EQ(context->get_discrete_state(0)[0], kSys1Id);
  EXPECT_EQ(context->get_discrete_state(1)[0], kSys2Id);

  // First action time should be 2 sec, and only sys1 will be updating.
  auto events = diagram.AllocateCompositeEventCollection();
  EXPECT_EQ(diagram.CalcNextUpdateTime(*context, events.get()), 2.);
  {
    const auto& subevent_collection =
        diagram.GetSubsystemCompositeEventCollection(diagram.get_sys(kSys1Id),
                                                     *events);
    EXPECT_TRUE(subevent_collection.get_discrete_update_events().HasEvents());
  }
  {
    const auto& subevent_collection =
        diagram.GetSubsystemCompositeEventCollection(diagram.get_sys(kSys2Id),
                                                     *events);
    EXPECT_FALSE(subevent_collection.get_discrete_update_events().HasEvents());
  }

  // Creates a temp state and sets it to some recognizable values.
  std::unique_ptr<DiscreteValues<double>> x_buf =
      diagram.AllocateDiscreteVariables();
  x_buf->get_mutable_vector(0)[0] = 98.0;
  x_buf->get_mutable_vector(1)[0] = 99.0;

  // Fast forward to the event time, and record it for the test below.
  double time = 2.0;
  context->SetTime(time);
  diagram.CalcDiscreteVariableUpdates(
      *context, events->get_discrete_update_events(), x_buf.get());

  // The non-participating sys2 state shouldn't have been copied (if it had
  // it would now be 2). sys1's state should have been copied, replacing the
  // 98 with a 1, then updated by adding time.
  EXPECT_EQ(x_buf->get_vector(0)[0], kSys1Id + time);  // Updated.
  EXPECT_EQ(x_buf->get_vector(1)[0], 99.0);      // Unchanged.

  // Swaps in the new state, and the discrete data for sys1 should be updated.
  diagram.ApplyDiscreteVariableUpdate(events->get_discrete_update_events(),
                                      x_buf.get(), context.get());
  EXPECT_EQ(context->get_discrete_state(0)[0], kSys1Id + time);  // == 3
  EXPECT_EQ(context->get_discrete_state(1)[0], kSys2Id);

  // Sets time to 5.5, both systems should be updating at 6 sec.  Note that
  // CalcNextUpdateTime() clears the event collection.
  context->SetTime(5.5);
  EXPECT_EQ(diagram.CalcNextUpdateTime(*context, events.get()), 6.);
  for (int i : {kSys1Id, kSys2Id}) {
    const auto& subevent_collection =
        diagram.GetSubsystemCompositeEventCollection(diagram.get_sys(i),
                                                     *events);
    EXPECT_TRUE(subevent_collection.get_discrete_update_events().HasEvents());
  }

  // Fast forward to the new event time, and record it for the tests below.
  time = 6.0;
  context->SetTime(time);
  diagram.CalcDiscreteVariableUpdates(
      *context, events->get_discrete_update_events(), x_buf.get());
  // Both sys1 and sys2's discrete data should be updated.
  diagram.ApplyDiscreteVariableUpdate(events->get_discrete_update_events(),
                                      x_buf.get(), context.get());
  EXPECT_EQ(context->get_discrete_state(0)[0], kSys1Id + 2 + time);
  EXPECT_EQ(context->get_discrete_state(1)[0], kSys2Id + time);
}

// Tests that a publish action is taken at 19 sec.
TEST_F(DiscreteStateTest, Publish) {
  context_->SetTime(18.5);
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());

  EXPECT_EQ(19.0, time);
  EXPECT_TRUE(events->HasPublishEvents());

  // Fast forward to 19.0 sec and do the publish.
  EXPECT_EQ(false, diagram_.publisher()->published());
  context_->SetTime(19.0);
  diagram_.Publish(*context_, events->get_publish_events());
  // Check that publication occurred.
  EXPECT_EQ(true, diagram_.publisher()->published());
}

class ForcedPublishingSystemDiagramTest : public ::testing::Test {
 public:
  void SetUp() override {
    context_ = diagram_.CreateDefaultContext();
  }

 protected:
  ForcedPublishingSystemDiagram diagram_;
  std::unique_ptr<Context<double>> context_;
};

// Tests that a forced publish is processed through the event handler.
TEST_F(ForcedPublishingSystemDiagramTest, Publish) {
  auto* forced_publishing_system_one = diagram_.publishing_system_one();
  auto* forced_publishing_system_two = diagram_.publishing_system_two();
  EXPECT_FALSE(forced_publishing_system_one->published());
  EXPECT_FALSE(forced_publishing_system_two->published());
  diagram_.Publish(*context_);
  EXPECT_TRUE(forced_publishing_system_one->published());
  EXPECT_TRUE(forced_publishing_system_two->published());
}

class SystemWithAbstractState : public LeafSystem<double> {
 public:
  SystemWithAbstractState(int id, double update_period) : id_(id) {
    DeclarePeriodicUnrestrictedUpdate(update_period, 0);
    DeclareAbstractState(Value<double>(id_));

    // Verify that no periodic discrete updates are registered.
    EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
  }

  ~SystemWithAbstractState() override {}

  // Abstract state is set to input state value + time.
  void DoCalcUnrestrictedUpdate(
      const Context<double>& context,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const override {
    double& state_num = state->get_mutable_abstract_state()
                            .get_mutable_value(0)
                            .get_mutable_value<double>();
    // The initial value for state should match what's currently in the
    // context.
    ASSERT_EQ(state_num, context.get_abstract_state<double>(0));
    state_num += context.get_time();
  }

  int get_id() const { return id_; }

 private:
  int id_{0};
};

class AbstractStateDiagram : public Diagram<double> {
 public:
  enum { kSys1Id = 1, kSys2Id = 2 };

  AbstractStateDiagram() : Diagram<double>() {
    DiagramBuilder<double> builder;
    sys1_ = builder.template AddSystem<SystemWithAbstractState>(kSys1Id, 2.);
    sys2_ = builder.template AddSystem<SystemWithAbstractState>(kSys2Id, 3.);
    builder.BuildInto(this);
  }

  const SystemWithAbstractState& get_sys(int i) const {
    DRAKE_DEMAND(i == kSys1Id || i == kSys2Id);
    return i == kSys1Id ? *sys1_ : *sys2_;
  }

  SystemWithAbstractState* get_mutable_sys1() { return sys1_; }
  SystemWithAbstractState* get_mutable_sys2() { return sys2_; }

 private:
  SystemWithAbstractState* sys1_{nullptr};
  SystemWithAbstractState* sys2_{nullptr};
};

class AbstractStateDiagramTest : public ::testing::Test {
 protected:
  void SetUp() override { context_ = diagram_.CreateDefaultContext(); }

  double get_sys1_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*diagram_.get_mutable_sys1(), *context_);
    return sys_context.get_abstract_state<double>(0);
  }

  double get_sys2_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*diagram_.get_mutable_sys2(), *context_);
    return sys_context.get_abstract_state<double>(0);
  }

  AbstractStateDiagram diagram_;
  std::unique_ptr<Context<double>> context_;
};

// Check that we count the abstract states correctly for a flat Diagram.
// DiagramOfDiagramsTest below checks nested diagrams.
TEST_F(AbstractStateDiagramTest, NumAbstractStates) {
  EXPECT_EQ(diagram_.num_abstract_states(), context_->num_abstract_states());
}

// Tests CalcUnrestrictedUpdate() when there are multiple subsystems and only
// one has an event to handle (call that the "participating subsystem"). We want
// to verify that only the participating subsystem's State gets copied, that the
// copied value matches the current value in the context, and that
// the update gets performed properly. We also check that it works properly
// when multiple subsystems have events to handle.
TEST_F(AbstractStateDiagramTest, CalcUnrestrictedUpdate) {
  const int kSys1Id = AbstractStateDiagram::kSys1Id;
  const int kSys2Id = AbstractStateDiagram::kSys2Id;

  context_->SetTime(1.5);

  // The abstract data should be initialized to their ids.
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id);
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id);

  // First action time should be 2 sec, and only sys1 will be updating.
  auto events = diagram_.AllocateCompositeEventCollection();
  EXPECT_EQ(diagram_.CalcNextUpdateTime(*context_, events.get()), 2.);
  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(kSys1Id), *events);
    EXPECT_TRUE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }
  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(kSys2Id), *events);
    EXPECT_FALSE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }

  // Creates a temp state and sets it to some recognizable values.
  std::unique_ptr<State<double>> x_buf = context_->CloneState();
  x_buf->get_mutable_abstract_state<double>(0) = 98.0;
  x_buf->get_mutable_abstract_state<double>(1) = 99.0;

  double time = 2.0;
  context_->SetTime(time);
  diagram_.CalcUnrestrictedUpdate(
      *context_, events->get_unrestricted_update_events(), x_buf.get());

  // The non-participating sys2 state shouldn't have been copied (if it had
  // it would now be kSys2Id). sys1's state should have been copied, replacing
  // the 98 with kSys1Id, then updated by adding time.
  EXPECT_EQ(x_buf->get_abstract_state<double>(0), kSys1Id + time);  // Updated.
  EXPECT_EQ(x_buf->get_abstract_state<double>(1), 99.0);  // Unchanged.

  // The abstract data in the current context should be the same as before.
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id);
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id);

  // Swaps in the new state, and the abstract data for sys0 should be updated.
  diagram_.ApplyUnrestrictedUpdate(events->get_unrestricted_update_events(),
      x_buf.get(), context_.get());
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id + time);  // == 3
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id);

  // Sets time to 5.5, both systems should be updating at 6 sec.  Note that
  // CalcNextUpdateTime() clears the event collection.
  context_->SetTime(5.5);
  EXPECT_EQ(diagram_.CalcNextUpdateTime(*context_, events.get()), 6.);
  for (int i : {kSys1Id, kSys2Id}) {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(i), *events);
    EXPECT_TRUE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }

  time = 6.0;
  context_->SetTime(time);
  diagram_.CalcUnrestrictedUpdate(
      *context_, events->get_unrestricted_update_events(), x_buf.get());
  // Both sys1 and sys2's abstract data should be updated.
  diagram_.ApplyUnrestrictedUpdate(events->get_unrestricted_update_events(),
                                   x_buf.get(), context_.get());
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id + 2.0 + time);
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id + time);
}

// Tests that in a Diagram where multiple subsystems have abstract variables,
// an unrestricted update in one subsystem doesn't invalidate the other.
// Note that although we use the caching system to observe what gets
// invalidated, we are testing for proper Diagram behavior here; we're not
// testing the caching system.
TEST_F(AbstractStateDiagramTest, UnrestrictedUpdateNotificationsAreLocalized) {
  const int kSys1Id = AbstractStateDiagram::kSys1Id;
  const int kSys2Id = AbstractStateDiagram::kSys2Id;

  Context<double>& ctx1 = diagram_.GetMutableSubsystemContext(
      diagram_.get_sys(kSys1Id), context_.get());
  Context<double>& ctx2 = diagram_.GetMutableSubsystemContext(
      diagram_.get_sys(kSys2Id), context_.get());

  // Allocate space to hold the updated state
  std::unique_ptr<State<double>> updates = context_->CloneState();

  // sys1 is due for an update at 2s, so only it should be included in the
  // next update time event collection.
  context_->SetTime(1.5);

  // Request the next update time.
  auto events = diagram_.AllocateCompositeEventCollection();
  const double next_time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(2.0, next_time);
  EXPECT_TRUE(events->HasUnrestrictedUpdateEvents());
  const auto& unrestricted_events = events->get_unrestricted_update_events();

  // Unrestricted update will notify xc, xd, xa and composite x. We'll count
  // xa notifications as representative. (We're not trying to prove here that
  // notifications are sent correctly, just that notifications are *not* sent
  // to uninvolved subsystems.)
  auto num_notifications = [](const Context<double>& context) {
    return context.get_tracker(SystemBase::xa_ticket())
        .num_notifications_received();
  };

  const int64_t notifications_1 = num_notifications(ctx1);
  const int64_t notifications_2 = num_notifications(ctx2);

  // The abstract data should be initialized to their ids.
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id);
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id);

  // Fast forward to 2.0 sec and collect the update.
  context_->SetTime(next_time);
  diagram_.CalcUnrestrictedUpdate(
      *context_, unrestricted_events, updates.get());

  // Of course nothing should have been notified since nothing's changed yet.
  EXPECT_EQ(num_notifications(ctx1), notifications_1);
  EXPECT_EQ(num_notifications(ctx2), notifications_2);

  // Selectively apply the update; only hold1 should get notified.
  diagram_.ApplyUnrestrictedUpdate(unrestricted_events, updates.get(),
                                       context_.get());
  // Sanity check that the update actually occurred -- should have added time
  // to sys1's abstract id.
  EXPECT_EQ(get_sys1_abstract_data_as_double(), kSys1Id + next_time);
  EXPECT_EQ(get_sys2_abstract_data_as_double(), kSys2Id);

  EXPECT_EQ(num_notifications(ctx1), notifications_1 + 1);
  EXPECT_EQ(num_notifications(ctx2), notifications_2);

  // Now apply the updates the dumb way. Everyone gets notified.
  context_->get_mutable_state().SetFrom(*updates);
  EXPECT_EQ(num_notifications(ctx1), notifications_1 + 2);
  EXPECT_EQ(num_notifications(ctx2), notifications_2 + 1);
}

/* Test diagram. Top level diagram (big_diagram) has 4 components:
a constant vector source, diagram0, int2, and diagram1. diagram0 has int0
and int1, and diagram1 has int3. Here's a picture:

+---------------------- big_diagram ----------------------+
|                                                         |
|                                   +---- diagram0 ----+  |
|                                   |                  |  |
|                                   |   +-----------+  |  |
|                                   |   |           |  |  |
|                            +------|--->   int0    +------->
|                            |      |   |           |  |  |
|                            |      |   +-----------+  |  |
|                            |      |                  |  |
|                            |      |   +-----------+  |  |
|                            |      |   |           |  |  |
|                            +------|--->   int1    +------->
|                            |      |   |           |  |  |
|                            |      |   +-----------+  |  |
|    +------------------+    |      |                  |  |
|    |                  |    |      +------------------+  |
|    |  ConstantVector  +--->|                            |
|    |                  |    |          +-----------+     |
|    +------------------+    |          |           |     |
|                            +---------->   int2    +------->
|                            |          |           |     |
|                            |          +-----------+     |
|                            |                            |
|                            |      +---- diagram1 ----+  |
|                            |      |                  |  |
|                            |      |   +-----------+  |  |
|                            |      |   |           |  |  |
|                            +------|--->   int3    +------->
|                                   |   |           |  |  |
|                                   |   +-----------+  |  |
|                                   |                  |  |
|                                   +------------------+  |
|                                                         |
+---------------------------------------------------------+
*/
class NestedDiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    DiagramBuilder<double> big_diagram_builder;
    integrator2_ = big_diagram_builder.AddSystem<Integrator<double>>(1);
    integrator2_->set_name("int2");

    {
      DiagramBuilder<double> builder;
      integrator0_ = builder.AddSystem<Integrator<double>>(1);
      integrator0_->set_name("int0");
      integrator1_ = builder.AddSystem<Integrator<double>>(1);
      integrator1_->set_name("int1");

      builder.ExportOutput(integrator0_->get_output_port());
      builder.ExportOutput(integrator1_->get_output_port());
      builder.ExportInput(integrator0_->get_input_port());
      builder.ExportInput(integrator1_->get_input_port());

      diagram0_ = big_diagram_builder.AddSystem(builder.Build());
      diagram0_->set_name("diagram0");
    }

    {
      DiagramBuilder<double> builder;
      integrator3_ = builder.AddSystem<Integrator<double>>(1);
      integrator3_->set_name("int3");

      builder.ExportOutput(integrator3_->get_output_port());
      builder.ExportInput(integrator3_->get_input_port());

      diagram1_ = big_diagram_builder.AddSystem(builder.Build());
      diagram1_->set_name("diagram1");
    }

    big_diagram_builder.ExportOutput(diagram0_->get_output_port(0));
    big_diagram_builder.ExportOutput(diagram0_->get_output_port(1));
    big_diagram_builder.ExportOutput(integrator2_->get_output_port());

    big_diagram_builder.ExportOutput(diagram1_->get_output_port(0));

    auto src = big_diagram_builder.AddSystem<ConstantVectorSource<double>>(1);
    src->set_name("constant");
    big_diagram_builder.Connect(src->get_output_port(),
                                integrator2_->get_input_port());
    big_diagram_builder.Connect(src->get_output_port(),
                                diagram0_->get_input_port(0));
    big_diagram_builder.Connect(src->get_output_port(),
                                diagram0_->get_input_port(1));

    big_diagram_builder.Connect(src->get_output_port(),
                                diagram1_->get_input_port(0));

    big_diagram_ = big_diagram_builder.Build();
    big_diagram_->set_name("big_diagram");
    big_context_ = big_diagram_->CreateDefaultContext();
    big_output_ = big_diagram_->AllocateOutput();

    // Make sure caching is on locally, even if it is off by default.
    // Do not remove this line; we want to show that these tests function
    // correctly with caching on. It is easier to pass with caching off!
    big_context_->EnableCaching();
  }

  Integrator<double>* integrator0_;
  Integrator<double>* integrator1_;
  Integrator<double>* integrator2_;
  Integrator<double>* integrator3_;

  Diagram<double>* diagram0_;
  Diagram<double>* diagram1_;

  std::unique_ptr<Diagram<double>> big_diagram_;
  std::unique_ptr<Context<double>> big_context_;
  std::unique_ptr<SystemOutput<double>> big_output_;
};

// Check functioning and error checking of each method for extracting
// subcontexts.
TEST_F(NestedDiagramContextTest, GetSubsystemContext) {
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 0);

  big_diagram_->GetMutableSubsystemContext(*integrator0_, big_context_.get())
      .get_mutable_continuous_state_vector()[0] = 1;
  big_diagram_->GetMutableSubsystemContext(*integrator1_, big_context_.get())
      .get_mutable_continuous_state_vector()[0] = 2;
  big_diagram_->GetMutableSubsystemContext(*integrator2_, big_context_.get())
      .get_mutable_continuous_state_vector()[0] = 3;
  big_diagram_->GetMutableSubsystemContext(*integrator3_, big_context_.get())
      .get_mutable_continuous_state_vector()[0] = 4;

  // Checks states.
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator0_, *big_context_)
                .get_continuous_state_vector()[0],
            1);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator1_, *big_context_)
                .get_continuous_state_vector()[0],
            2);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator2_, *big_context_)
                .get_continuous_state_vector()[0],
            3);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator3_, *big_context_)
                .get_continuous_state_vector()[0],
            4);

  // Checks output.
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 1);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 2);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 3);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 4);

  // Starting with the root context, the subsystems should be able to find
  // their own contexts.
  // Integrator 1 is a child of a child.
  EXPECT_EQ(integrator1_->GetMyContextFromRoot(*big_context_)
                .get_continuous_state_vector()[0],
            2);
  // Integrator 2 is a direct child.
  EXPECT_EQ(integrator2_->GetMyContextFromRoot(*big_context_)
                .get_continuous_state_vector()[0],
            3);
  // Extract a sub-diagram context.
  Context<double>& diagram0_context =
      diagram0_->GetMyMutableContextFromRoot(&*big_context_);
  // This should fail because this context is not a root context.
  DRAKE_EXPECT_THROWS_MESSAGE(
      integrator1_->GetMyContextFromRoot(diagram0_context), std::logic_error,
      ".*context must be a root context.*");

  // Should fail because integrator3 is not in diagram0.
  DRAKE_EXPECT_THROWS_MESSAGE(
      diagram0_->GetSubsystemContext(*integrator3_, diagram0_context),
      std::logic_error,
      ".*Integrator.*int3.*not contained in.*Diagram.*diagram0.*");

  // Modify through the sub-Diagram context, then read back from root.
  Context<double>& integrator1_context =
      diagram0_->GetMutableSubsystemContext(*integrator1_, &diagram0_context);
  integrator1_context.get_mutable_continuous_state_vector()[0] = 17.;
  EXPECT_EQ(integrator1_->GetMyContextFromRoot(*big_context_)
                .get_continuous_state_vector()[0],
            17.);
}

// Sets the continuous state of all the integrators through
// GetMutableSubsystemState(), and check that they are correctly set.
TEST_F(NestedDiagramContextTest, GetSubsystemState) {
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 0);

  State<double>& big_state = big_context_->get_mutable_state();
  big_diagram_
      ->GetMutableSubsystemState(*integrator0_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()[0] = 1;
  big_diagram_
      ->GetMutableSubsystemState(*integrator1_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()[0] = 2;
  big_diagram_
      ->GetMutableSubsystemState(*integrator2_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()[0] = 3;
  big_diagram_
      ->GetMutableSubsystemState(*integrator3_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()[0] = 4;

  // Checks state.
  EXPECT_EQ(big_diagram_->GetSubsystemState(*integrator0_, big_state)
                .get_continuous_state().get_vector()[0],
            1);
  EXPECT_EQ(big_diagram_->GetSubsystemState(*integrator1_, big_state)
                .get_continuous_state().get_vector()[0],
            2);
  EXPECT_EQ(big_diagram_->GetSubsystemState(*integrator2_, big_state)
                .get_continuous_state().get_vector()[0],
            3);
  EXPECT_EQ(big_diagram_->GetSubsystemState(*integrator3_, big_state)
                .get_continuous_state().get_vector()[0],
            4);

  // Checks output.
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 1);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 2);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 3);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 4);
}

// Tests that ContextBase methods for affecting cache behavior propagate
// all the way through a nested Diagram. Since leaf output ports have cache
// entries in their corresponding subcontext, we'll pick one at the bottom
// and check its behavior.
TEST_F(NestedDiagramContextTest, CachingChangePropagation) {
  const Context<double>& diagram1_subcontext =
      big_diagram_->GetSubsystemContext(*diagram1_, *big_context_);
  const Context<double>& integrator3_subcontext =
      diagram1_->GetSubsystemContext(*integrator3_, diagram1_subcontext);
  const OutputPort<double>& integrator3_output =
      integrator3_->get_output_port();
  auto& integrator3_leaf_output =
      dynamic_cast<const LeafOutputPort<double>&>(integrator3_output);

  auto& cache_entry = integrator3_leaf_output.cache_entry();
  EXPECT_FALSE(cache_entry.is_cache_entry_disabled(integrator3_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(integrator3_subcontext));

  big_context_->DisableCaching();
  EXPECT_TRUE(cache_entry.is_cache_entry_disabled(integrator3_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(integrator3_subcontext));

  big_context_->EnableCaching();
  EXPECT_FALSE(cache_entry.is_cache_entry_disabled(integrator3_subcontext));
  EXPECT_TRUE(cache_entry.is_out_of_date(integrator3_subcontext));

  // Bring the cache entry up to date.
  cache_entry.EvalAbstract(integrator3_subcontext);
  EXPECT_FALSE(cache_entry.is_out_of_date(integrator3_subcontext));

  big_context_->SetAllCacheEntriesOutOfDate();
  EXPECT_TRUE(cache_entry.is_out_of_date(integrator3_subcontext));
}

/* Check that changes made directly to a subcontext still affect the
parent Diagram's behavior properly. Also, time and accuracy must be identical
in every subcontext of a Context tree. They are only permitted to change at the
root so they can be properly propagated down.

        +-----------------------------------------------------+
        |                                                     |
        |       +------------+             +-----------+      |
        |       |            |             |           |      |
      u |    u0 |            | y0       u1 |           | y1   | y (=x1)
Fixed ---------->  integ0    +------------->  integ1   +-------->
value   |       |            |             |           |      |
        |       |     x0     |             |    x1     |      |
        |       +------------+             +-----------+      |
        |                                                     | xdot={u0,u1,u2}
        |                    +------------+            +-------->
        |                    |            |                   |
        |               u2   |            | y2                |
        |       Fixed ------->  integ2    +------>            |
        |       value        |            |                   |
        |                    |     x2     |                   |
        |                    +------------+                   |
        |                                                     |
        |                   diagram x={x0,x1,x2}              |
        |                        xdot={u0,u1,u2}              |
        +-----------------------------------------------------+

Note that integ2's input port is invisible to the diagram, but a value change
to that hidden port should invalidate the diagram's composite derivative
calculation. */
GTEST_TEST(MutateSubcontextTest, DiagramRecalculatesOnSubcontextChange) {
  DiagramBuilder<double> builder;
  auto integ0 = builder.AddSystem<Integrator>(1);  // (xdot = u; y = x)
  auto integ1 = builder.AddSystem<Integrator>(1);
  auto integ2 = builder.AddSystem<Integrator>(1);
  builder.ExportInput(integ0->get_input_port());
  builder.Cascade(*integ0, *integ1);
  builder.ExportOutput(integ1->get_output_port());
  auto diagram = builder.Build();

  auto diagram_context = diagram->AllocateContext();
  diagram_context->EnableCaching();
  diagram->get_input_port(0).FixValue(diagram_context.get(), 1.5);  // u(=u0)

  // Hunt down the cache entry for the Diagram's derivative computation so we
  // can verify that it gets invalidated and recomputed as we expect.
  auto& derivative_tracker =
      diagram_context->get_tracker(diagram->xcdot_ticket());
  ASSERT_NE(derivative_tracker.cache_entry_value(), nullptr);
  const CacheEntryValue& derivative_cache =
      *derivative_tracker.cache_entry_value();
  EXPECT_TRUE(derivative_cache.is_out_of_date());

  // Record the derivative serial number so we can watch out for unnecessary
  // extra computations.
  int64_t expected_derivative_serial = derivative_cache.serial_number();

  VectorXd init_state(3);
  init_state << 5., 6., 7.;  // x0(=y0=u1), x1(=y1), x2(=y2)

  // Set the state from the diagram level, then evaluate the diagram
  // output, which should have copied the second state value (x1). Also, this
  // shouldn't complain even though input u2 is dangling since we don't need it
  // for this computation.
  diagram_context->SetContinuousState(init_state);
  EXPECT_EQ(diagram->get_output_port(0).Eval<BasicVector<double>>(
                *diagram_context)[0],
            6.);

  // That should not have caused derivative evaluations.
  EXPECT_TRUE(derivative_cache.is_out_of_date());
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Must set the dangling input port to a value to evaluate derivatives.
  Context<double>& context2 =
      diagram->GetMutableSubsystemContext(*integ2, &*diagram_context);
  FixedInputPortValue& u2_value =
      integ2->get_input_port(0).FixValue(&context2, 0.75);

  // The diagram derivatives should be (u0,u1,u2)=(u,x0,u2).
  auto& eval_derivs =
      diagram->EvalTimeDerivatives(*diagram_context);
  ++expected_derivative_serial;
  EXPECT_EQ(eval_derivs[0], 1.5);
  EXPECT_EQ(eval_derivs[1], 5.);
  EXPECT_EQ(eval_derivs[2], 0.75);

  EXPECT_FALSE(derivative_cache.is_out_of_date());
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Now try to sneak in a change to subcontext state variables and then ask for
  // the diagram's output value and derivatives again.
  Context<double>& context0 =
      diagram->GetMutableSubsystemContext(*integ0, &*diagram_context);
  Context<double>& context1 =
      diagram->GetMutableSubsystemContext(*integ1, &*diagram_context);

  VectorXd new_x0(1), new_x1(1);
  new_x0 << 13.; new_x1 << 17.;
  context0.SetContinuousState(new_x0);
  context1.SetContinuousState(new_x1);

  // Diagram should know to recopy x1 to the output port cache entry.
  EXPECT_EQ(diagram->get_output_port(0).Eval<BasicVector<double>>(
                *diagram_context)[0],
            17.);

  // Diagram derivatives should be out of date since they depend on x0.
  EXPECT_TRUE(derivative_cache.is_out_of_date());
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Diagram should know that the time derivatives cache entry is out of date
  // so initiate subsystem derivative calculations and pick up the changed x0.
  // This will fail if subcontext *state* modifications aren't transmitted
  // properly to the Diagram.
  diagram->EvalTimeDerivatives(*diagram_context);
  ++expected_derivative_serial;
  EXPECT_EQ(eval_derivs[0], 1.5);
  EXPECT_EQ(eval_derivs[1], 13.);
  EXPECT_EQ(eval_derivs[2], 0.75);

  EXPECT_FALSE(derivative_cache.is_out_of_date());
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Re-evaluate derivatives now and verify that they weren't recomputed.
  diagram->EvalTimeDerivatives(*diagram_context);
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Now let's try modifying the internal input port. That shouldn't have any
  // effect on the Diagram output port, since that depends only on state and
  // hasn't changed. However, input u2 is the derivative result for integ2 and
  // that should be reflected in the diagram derivative result.
  u2_value.GetMutableVectorData<double>()->SetAtIndex(0, 300.);
  EXPECT_EQ(diagram->get_output_port(0).Eval<BasicVector<double>>(
      *diagram_context)[0],
            17.);  // No change.
  EXPECT_TRUE(derivative_cache.is_out_of_date());

  diagram->EvalTimeDerivatives(*diagram_context);
  ++expected_derivative_serial;
  EXPECT_EQ(eval_derivs[0], 1.5);
  EXPECT_EQ(eval_derivs[1], 13.);
  EXPECT_EQ(eval_derivs[2], 300.);

  EXPECT_FALSE(derivative_cache.is_out_of_date());
  EXPECT_EQ(derivative_cache.serial_number(), expected_derivative_serial);

  // Time & accuracy changes are allowed at the root (diagram) level.
  DRAKE_EXPECT_NO_THROW(diagram_context->SetTime(1.));
  DRAKE_EXPECT_NO_THROW(
      diagram_context->SetTimeAndContinuousState(2., init_state));
  auto diagram_context_clone = diagram_context->Clone();
  DRAKE_EXPECT_NO_THROW(
      diagram_context->SetTimeStateAndParametersFrom(*diagram_context_clone));
  DRAKE_EXPECT_NO_THROW(diagram_context->SetAccuracy(1e-6));

  // Time & accuracy changes NOT allowed at child (leaf) level.
  DRAKE_EXPECT_THROWS_MESSAGE(context0.SetTime(3.), std::logic_error,
                              ".*SetTime.*Time change allowed only.*root.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      context0.SetTimeAndContinuousState(4., new_x0), std::logic_error,
      ".*SetTimeAndContinuousState.*Time change allowed only.*root.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      context0.SetTimeStateAndParametersFrom(context1), std::logic_error,
      ".*SetTimeStateAndParametersFrom.*Time change allowed only.*root.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      context0.SetAccuracy(1e-7), std::logic_error,
      ".*SetAccuracy.*Accuracy change allowed only.*root.*");
}

// Tests that an exception is thrown if the systems in a Diagram do not have
// unique names.
GTEST_TEST(NonUniqueNamesTest, NonUniqueNames) {
  DiagramBuilder<double> builder;
  const int kInputs = 2;
  const int kSize = 1;
  auto adder0 = builder.AddSystem<Adder<double>>(kInputs, kSize);
  adder0->set_name("unoriginal");
  auto adder1 = builder.AddSystem<Adder<double>>(kInputs, kSize);
  adder1->set_name("unoriginal");
  EXPECT_THROW(builder.Build(), std::runtime_error);
}

// Tests that systems with unset names can be added to a Diagram.
GTEST_TEST(NonUniqueNamesTest, DefaultEmptyNames) {
  DiagramBuilder<double> builder;
  const int kInputs = 2;
  const int kSize = 1;
  builder.AddSystem<Adder<double>>(kInputs, kSize);
  builder.AddSystem<Adder<double>>(kInputs, kSize);
  DRAKE_EXPECT_NO_THROW(builder.Build());
}

// Tests that an exception is thrown if a system is reset to an empty name
// *after* being added to the diagram builder.
GTEST_TEST(NonUniqueNamesTest, ForcedEmptyNames) {
  DiagramBuilder<double> builder;
  const int kInputs = 2;
  const int kSize = 1;
  builder.AddSystem<Adder<double>>(kInputs, kSize);
  builder.AddSystem<Adder<double>>(kInputs, kSize)->set_name("");
  EXPECT_THROW(builder.Build(), std::runtime_error);
}

// A system for testing per step actions. A system can add per-step events for
// each event type as selected.
class PerStepActionTestSystem : public LeafSystem<double> {
 public:
  PerStepActionTestSystem() {
    DeclareDiscreteState(1);
    DeclareAbstractState(Value<std::string>(""));
  }

  // Methods for adding events post-construction.
  void AddPublishEvent() {
    DeclarePerStepPublishEvent(&PerStepActionTestSystem::PublishHandler);
  }

  void AddDiscreteUpdateEvent() {
    DeclarePerStepDiscreteUpdateEvent(
        &PerStepActionTestSystem::DiscreteHandler);
  }

  void AddUnrestrictedUpdateEvent() {
    DeclarePerStepUnrestrictedUpdateEvent(
        &PerStepActionTestSystem::UnrestrictedHandler);
  }

  using LeafSystem<double>::DeclarePerStepEvent;

  int publish_count() const { return publish_count_; }

 private:
  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {
    state->get_mutable_discrete_state()[0] = 0;
    state->get_mutable_abstract_state<std::string>(0) = "wow";
  }

  // TODO(15465) When per-step event declaration sugar allows for callbacks
  // whose result is "assumed to succeed", change these to void return types.
  EventStatus DiscreteHandler(const Context<double>& context,
                              DiscreteValues<double>* discrete_state) const {
    (*discrete_state)[0] = context.get_discrete_state(0)[0] + 1;
    return EventStatus::Succeeded();
  }

  EventStatus UnrestrictedHandler(const Context<double>& context,
                                  State<double>* state) const {
    // Depends on discrete state (therefore order of event evaluation).
    int int_num = static_cast<int>(context.get_discrete_state(0)[0]);
    state->get_mutable_abstract_state<std::string>(0) =
        "wow" + std::to_string(int_num);
    return EventStatus::Succeeded();
  }

  EventStatus PublishHandler(const Context<double>& context) const {
    ++publish_count_;
    return EventStatus::Succeeded();
  }

  // A hack to test publish calls easily as the Publish events have no access
  // to writable memory anywhere in the context. This is an anti-pattern outside
  // of unit tests.
  mutable int publish_count_{0};
};

// Test that the diagram successfully dispatches events into its component
// systems. To that end, we create a diagram with a nested diagram and sibling
// leaf systems. Each leaf system has a unique set of events (None, discrete and
// and unrestricted, and unrestricted and publish, respectively). By invoking
// the various forced event-generating methods (CalcUnrestrictedUpdate,
// CalcDiscreteVariableUpdates, and Publish), we can observe the results by
// observing the context for the system (and its for-the-unit-test, hacked
// internal state). The fact that these unit tests use events triggered by
// per-step events is wholly irrelevant -- at this tested level of the API, the
// trigger doesn't matter.
GTEST_TEST(DiagramEventEvaluation, Propagation) {
  std::unique_ptr<Diagram<double>> sub_diagram;
  PerStepActionTestSystem* sys0;
  PerStepActionTestSystem* sys1;
  PerStepActionTestSystem* sys2;

  // Sub diagram. Has sys0, and sys1.
  // sys0 does not have any per step actions.
  // sys1 has discrete and unrestricted updates.
  {
    DiagramBuilder<double> builder;
    sys0 = builder.AddSystem<PerStepActionTestSystem>();
    sys0->set_name("sys0");

    sys1 = builder.AddSystem<PerStepActionTestSystem>();
    sys1->set_name("sys1");
    sys1->AddDiscreteUpdateEvent();
    sys1->AddUnrestrictedUpdateEvent();

    sub_diagram = builder.Build();
    sub_diagram->set_name("sub_diagram");
  }

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(sub_diagram));
  sys2 = builder.AddSystem<PerStepActionTestSystem>();
  sys2->set_name("sys2");
  sys2->AddPublishEvent();
  sys2->AddUnrestrictedUpdateEvent();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->set_name("diagram");

  auto tmp_discrete_state = diagram->AllocateDiscreteVariables();
  std::unique_ptr<State<double>> tmp_state = context->CloneState();

  auto events = diagram->AllocateCompositeEventCollection();
  diagram->GetPerStepEvents(*context, events.get());
  ASSERT_TRUE(events->HasEvents());

  // Does unrestricted update first.
  diagram->CalcUnrestrictedUpdate(
      *context, events->get_unrestricted_update_events(), tmp_state.get());
  context->get_mutable_state().SetFrom(*tmp_state);

  // Does discrete updates second.
  diagram->CalcDiscreteVariableUpdates(*context,
                                       events->get_discrete_update_events(),
                                       tmp_discrete_state.get());
  context->get_mutable_discrete_state().SetFrom(*tmp_discrete_state);

  // Publishes last.
  diagram->Publish(*context, events->get_publish_events());

  // Only sys2 published once.
  EXPECT_EQ(sys0->publish_count(), 0);
  EXPECT_EQ(sys1->publish_count(), 0);
  EXPECT_EQ(sys2->publish_count(), 1);

  // sys0 doesn't have any updates; the state is in its initial state.
  auto& sys0_context = diagram->GetSubsystemContext(*sys0, *context);
  EXPECT_EQ(sys0_context.get_discrete_state(0)[0], 0);
  EXPECT_EQ(sys0_context.get_abstract_state<std::string>(0), "wow");
  EXPECT_EQ(sys0->publish_count(), 0);

  // sys1 should have an unrestricted update then a discrete update.
  auto& sys1_context = diagram->GetSubsystemContext(*sys1, *context);
  EXPECT_EQ(sys1_context.get_discrete_state(0)[0], 1);
  EXPECT_EQ(sys1_context.get_abstract_state<std::string>(0), "wow0");
  EXPECT_EQ(sys1->publish_count(), 0);

  // sys2 should have a unrestricted update then a publish.
  auto& sys2_context = diagram->GetSubsystemContext(*sys2, *context);
  EXPECT_EQ(sys2_context.get_discrete_state(0)[0], 0);
  EXPECT_EQ(sys2_context.get_abstract_state<std::string>(0), "wow0");
  EXPECT_EQ(sys2->publish_count(), 1);
}

class MyEventTestSystem : public LeafSystem<double> {
 public:
  // If p > 0, declares a periodic publish event with p. Otherwise, declares
  // a per step publish event.
  MyEventTestSystem(const std::string& name, double p) {
    if (p > 0) {
      DeclarePeriodicPublishEvent(p, 0.0, &MyEventTestSystem::PublishPeriodic);

      // Verify that no periodic discrete updates are registered.
      EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
    } else {
      DeclarePerStepPublishEvent(&MyEventTestSystem::PublishPerStep);
    }
    set_name(name);
  }

  int get_periodic_count() const { return periodic_count_; }

  int get_per_step_count() const { return per_step_count_; }

 private:
  void PublishPeriodic(const Context<double>&) const {
     ++periodic_count_;
  }

  // TODO(15465) When per-step event declaration sugar allows for callbacks
  // whose result is "assumed to succeed", change this to void return.
  EventStatus PublishPerStep(const Context<double>&) const {
    ++per_step_count_;
    return EventStatus::Succeeded();
  }

  mutable int periodic_count_{0};
  mutable int per_step_count_{0};
};

// Test the event functionality of the MyEventTestSystem before using it in the
// diagram.
GTEST_TEST(MyEventTest, MyEventTestLeaf) {
  // Test both construction modes: periodic and per-step events.
  for (double period : {0.2, 0.0}) {
    MyEventTestSystem dut("sys", period);
    auto events = dut.AllocateCompositeEventCollection();
    auto periodic_events = dut.AllocateCompositeEventCollection();
    auto per_step_events = dut.AllocateCompositeEventCollection();
    auto context = dut.CreateDefaultContext();
    EXPECT_EQ(events->get_system_id(), context->get_system_id());

    // If period is zero, we still need to evaluate the per step events.
    double time = dut.CalcNextUpdateTime(*context, periodic_events.get());
    dut.GetPerStepEvents(*context, per_step_events.get());
    events->AddToEnd(*periodic_events);
    events->AddToEnd(*per_step_events);
    context->SetTime(time);
    dut.Publish(*context, events->get_publish_events());

    EXPECT_EQ(dut.get_periodic_count(), period > 0 ? 1 : 0);
    EXPECT_EQ(dut.get_per_step_count(), period > 0 ? 0 : 1);
  }
}

// Builds a diagram with a sub diagram (has 3 MyEventTestSystem) and 2
// MyEventTestSystem. sys4 is configured to have a per-step event, and all
// the others have periodic publish events. Tests Diagram::CalcNextUpdateTime,
// and Diagram::GetPerStepEvents. The result should be sys1, sys2, sys3, sys4
// fired their proper callbacks.
GTEST_TEST(MyEventTest, MyEventTestDiagram) {
  std::unique_ptr<Diagram<double>> sub_diagram;
  std::vector<const MyEventTestSystem*> sys(5);

  {
    DiagramBuilder<double> builder;
    // sys0's scheduled time is after the rest, so its trigger should not fire.
    sys[0] = builder.AddSystem<MyEventTestSystem>("sys0", 0.2);
    sys[1] = builder.AddSystem<MyEventTestSystem>("sys1", 0.1);
    sys[2] = builder.AddSystem<MyEventTestSystem>("sys2", 0.1);

    sub_diagram = builder.Build();
    sub_diagram->set_name("sub_diagram");
  }
  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(sub_diagram));
  sys[3] = builder.AddSystem<MyEventTestSystem>("sys3", 0.1);
  sys[4] = builder.AddSystem<MyEventTestSystem>("sys4", 0.);

  auto dut = builder.Build();

  auto periodic_events = dut->AllocateCompositeEventCollection();
  auto per_step_events = dut->AllocateCompositeEventCollection();
  auto events = dut->AllocateCompositeEventCollection();

  auto context = dut->CreateDefaultContext();

  double time = dut->CalcNextUpdateTime(*context, periodic_events.get());
  dut->GetPerStepEvents(*context, per_step_events.get());

  events->AddToEnd(*periodic_events);
  events->AddToEnd(*per_step_events);

  context->SetTime(time);
  dut->Publish(*context, events->get_publish_events());

  // Sys0's period is larger, so it doesn't get evaluated.
  EXPECT_EQ(sys[0]->get_periodic_count(), 0);
  EXPECT_EQ(sys[0]->get_per_step_count(), 0);

  // Sys1, 2, & 3, all have periodic events at `time`.
  EXPECT_EQ(sys[1]->get_periodic_count(), 1);
  EXPECT_EQ(sys[1]->get_per_step_count(), 0);

  EXPECT_EQ(sys[2]->get_periodic_count(), 1);
  EXPECT_EQ(sys[2]->get_per_step_count(), 0);

  EXPECT_EQ(sys[3]->get_periodic_count(), 1);
  EXPECT_EQ(sys[3]->get_per_step_count(), 0);

  // Sys4 has only a per-step event, it gets triggered because we're taking a
  // step.
  EXPECT_EQ(sys[4]->get_periodic_count(), 0);
  EXPECT_EQ(sys[4]->get_per_step_count(), 1);
}

template <typename T>
class ConstraintTestSystem : public LeafSystem<T> {
 public:
  ConstraintTestSystem()
      : LeafSystem<T>(systems::SystemTypeTag<ConstraintTestSystem>{}) {
    this->DeclareContinuousState(2);
    this->DeclareEqualityConstraint(&ConstraintTestSystem::CalcState0Constraint,
                                    1, "x0");
    this->DeclareInequalityConstraint(
        &ConstraintTestSystem::CalcStateConstraint,
        { Vector2d::Zero(), std::nullopt }, "x");
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit ConstraintTestSystem(const ConstraintTestSystem<U>& system)
      : ConstraintTestSystem() {}

  // Expose some protected methods for testing.
  using LeafSystem<T>::DeclareInequalityConstraint;
  using LeafSystem<T>::DeclareEqualityConstraint;

  void CalcState0Constraint(const Context<T>& context,
                            VectorX<T>* value) const {
    *value = Vector1<T>(context.get_continuous_state_vector()[0]);
  }
  void CalcStateConstraint(const Context<T>& context, VectorX<T>* value) const {
    *value = context.get_continuous_state_vector().CopyToVector();
  }

 private:
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    // xdot = -x.
    derivatives->SetFromVector(-dynamic_cast<const BasicVector<T>&>(
                                    context.get_continuous_state_vector())
                                    .get_value());
  }
};

GTEST_TEST(DiagramConstraintTest, SystemConstraintsTest) {
  systems::DiagramBuilder<double> builder;
  auto sys1 = builder.AddSystem<ConstraintTestSystem<double>>();
  auto sys2 = builder.AddSystem<ConstraintTestSystem<double>>();

  auto diagram = builder.Build();
  EXPECT_EQ(diagram->num_constraints(), 4);  // two from each system

  auto context = diagram->CreateDefaultContext();

  // Set sys1 context.
  diagram->GetMutableSubsystemContext(*sys1, context.get())
      .get_mutable_continuous_state_vector()
      .SetFromVector(Vector2d(5.0, 7.0));

  // Set sys2 context.
  diagram->GetMutableSubsystemContext(*sys2, context.get())
      .get_mutable_continuous_state_vector()
      .SetFromVector(Vector2d(11.0, 12.0));

  VectorXd value;
  // Check system 1's x0 constraint.
  const SystemConstraint<double>& constraint0 =
      diagram->get_constraint(SystemConstraintIndex(0));
  constraint0.Calc(*context, &value);
  EXPECT_EQ(value.size(), 1);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_TRUE(constraint0.is_equality_constraint());
  std::string description = constraint0.description();
  // Constraint description should end in the original description.
  EXPECT_EQ(description.substr(description.size() - 2), "x0");

  // Check system 1's x constraint
  const SystemConstraint<double>& constraint1 =
      diagram->get_constraint(SystemConstraintIndex(1));
  constraint1.Calc(*context, &value);
  EXPECT_EQ(value.size(), 2);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_EQ(value[1], 7.0);
  EXPECT_FALSE(constraint1.is_equality_constraint());
  description = constraint1.description();
  EXPECT_EQ(description.substr(description.size() - 1), "x");

  // Check system 2's x0 constraint.
  const SystemConstraint<double>& constraint2 =
      diagram->get_constraint(SystemConstraintIndex(2));
  constraint2.Calc(*context, &value);
  EXPECT_EQ(value.size(), 1);
  EXPECT_EQ(value[0], 11.0);
  EXPECT_TRUE(constraint2.is_equality_constraint());
  description = constraint2.description();
  EXPECT_EQ(description.substr(description.size() - 2), "x0");

  // Check system 2's x constraint
  const SystemConstraint<double>& constraint3 =
      diagram->get_constraint(SystemConstraintIndex(3));
  constraint3.Calc(*context, &value);
  EXPECT_EQ(value.size(), 2);
  EXPECT_EQ(value[0], 11.0);
  EXPECT_EQ(value[1], 12.0);
  EXPECT_FALSE(constraint3.is_equality_constraint());
  description = constraint3.description();
  EXPECT_EQ(description.substr(description.size() - 1), "x");

  // Check that constraints survive ToAutoDiffXd.
  auto autodiff_diagram = diagram->ToAutoDiffXd();
  EXPECT_EQ(autodiff_diagram->num_constraints(), 4);
  auto autodiff_context = autodiff_diagram->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(*context);
  const SystemConstraint<AutoDiffXd>& autodiff_constraint =
      autodiff_diagram->get_constraint(SystemConstraintIndex(3));
  VectorX<AutoDiffXd> autodiff_value;
  autodiff_constraint.Calc(*autodiff_context, &autodiff_value);
  EXPECT_EQ(autodiff_value[0].value(), 11.0);

  // Check that constraints survive ToSymbolic.
  auto symbolic_diagram = diagram->ToSymbolic();
  EXPECT_EQ(symbolic_diagram->num_constraints(), 4);
  auto symbolic_context = symbolic_diagram->CreateDefaultContext();
  symbolic_context->SetTimeStateAndParametersFrom(*context);
  const SystemConstraint<symbolic::Expression>& symbolic_constraint =
      symbolic_diagram->get_constraint(SystemConstraintIndex(3));
  VectorX<symbolic::Expression> symbolic_value;
  symbolic_constraint.Calc(*symbolic_context, &symbolic_value);
  EXPECT_EQ(symbolic_value[0], 11.0);
}

GTEST_TEST(DiagramParametersTest, ParameterTest) {
  // Construct a diagram with multiple subsystems that have parameters.
  systems::DiagramBuilder<double> builder;
  auto pendulum1 =
      builder.AddSystem<examples::pendulum::PendulumPlant<double>>();
  auto pendulum2 =
      builder.AddSystem<examples::pendulum::PendulumPlant<double>>();
  auto constant_torque =
      builder.AddSystem<ConstantVectorSource<double>>(Vector1d(1.0));
  builder.Cascade(*constant_torque, *pendulum1);
  builder.Cascade(*constant_torque, *pendulum2);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  // Make sure the Diagram correctly reports its aggregate number of
  // parameters.
  EXPECT_EQ(diagram->num_numeric_parameter_groups(),
            context->num_numeric_parameter_groups());

  // Get pointers to the parameters.
  auto params1 = dynamic_cast<examples::pendulum::PendulumParams<double>*>(
      &diagram->GetMutableSubsystemContext(*pendulum1, context.get())
          .get_mutable_numeric_parameter(0));
  auto params2 = dynamic_cast<examples::pendulum::PendulumParams<double>*>(
      &diagram->GetMutableSubsystemContext(*pendulum2, context.get())
          .get_mutable_numeric_parameter(0));

  const double original_damping = params1->damping();
  const double new_damping = 5.0*original_damping;
  EXPECT_EQ(params2->damping(), original_damping);

  params1->set_damping(new_damping);
  // Check that I didn't change params2.
  EXPECT_EQ(params2->damping(), original_damping);

  diagram->SetDefaultContext(context.get());
  // Check that the original value is restored.
  EXPECT_EQ(params1->damping(), original_damping);

  params2->set_damping(new_damping);
  diagram->SetDefaultParameters(*context, &context->get_mutable_parameters());
  // Check that the original value is restored.
  EXPECT_EQ(params2->damping(), original_damping);
}

// Note: this class is duplicated from leaf_system_test.
class RandomContextTestSystem : public LeafSystem<double> {
 public:
  RandomContextTestSystem() {
    this->DeclareContinuousState(
        BasicVector<double>(Vector2d(-1.0, -2.0)));
    this->DeclareNumericParameter(
        BasicVector<double>(Vector3d(1.0, 2.0, 3.0)));
  }

  void SetRandomState(const Context<double>& context, State<double>* state,
                      RandomGenerator* generator) const override {
    std::normal_distribution<double> normal;
    for (int i = 0; i < context.get_continuous_state_vector().size(); i++) {
      state->get_mutable_continuous_state().get_mutable_vector().SetAtIndex(
          i, normal(*generator));
    }
  }
  void SetRandomParameters(const Context<double>& context,
                           Parameters<double>* params,
                           RandomGenerator* generator) const override {
    std::uniform_real_distribution<double> uniform;
    for (int i = 0; i < context.get_numeric_parameter(0).size(); i++) {
      params->get_mutable_numeric_parameter(0).SetAtIndex(
          i, uniform(*generator));
    }
  }
};

GTEST_TEST(RandomContextTest, SetRandomTest) {
  DiagramBuilder<double> builder;

  builder.AddSystem<RandomContextTestSystem>();
  builder.AddSystem<RandomContextTestSystem>();

  const auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  // Back-up the numeric context values.
  Vector4d state = context->get_continuous_state_vector().CopyToVector();
  Vector3d params0 = context->get_numeric_parameter(0).CopyToVector();
  Vector3d params1 = context->get_numeric_parameter(1).CopyToVector();

  // Should return the (same) original values.
  diagram->SetDefaultContext(context.get());
  EXPECT_TRUE((state.array() ==
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE((params0.array() ==
               context->get_numeric_parameter(0).get_value().array())
                  .all());
  EXPECT_TRUE((params1.array() ==
               context->get_numeric_parameter(1).get_value().array())
                  .all());

  RandomGenerator generator;

  // Should return different values.
  diagram->SetRandomContext(context.get(), &generator);
  EXPECT_TRUE((state.array() !=
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE((params0.array() !=
               context->get_numeric_parameter(0).get_value().array())
                  .all());
  EXPECT_TRUE((params1.array() !=
               context->get_numeric_parameter(1).get_value().array())
                  .all());

  // Update backup.
  state = context->get_continuous_state_vector().CopyToVector();
  params0 = context->get_numeric_parameter(0).CopyToVector();
  params1 = context->get_numeric_parameter(1).CopyToVector();

  // Should return different values (again).
  diagram->SetRandomContext(context.get(), &generator);
  EXPECT_TRUE((state.array() !=
               context->get_continuous_state_vector().CopyToVector().array())
                  .all());
  EXPECT_TRUE((params0.array() !=
               context->get_numeric_parameter(0).get_value().array())
                  .all());
  EXPECT_TRUE((params1.array() !=
               context->get_numeric_parameter(1).get_value().array())
                  .all());
}

// Tests initialization works properly for all subsystems.
GTEST_TEST(InitializationTest, InitializationTest) {
  // Note: this class is duplicated in leaf_system_test.
  class InitializationTestSystem : public LeafSystem<double> {
   public:
    InitializationTestSystem() {
      PublishEvent<double> pub_event(
          TriggerType::kInitialization,
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);

      DeclareInitializationEvent(DiscreteUpdateEvent<double>(
          TriggerType::kInitialization));
      DeclareInitializationEvent(UnrestrictedUpdateEvent<double>(
          TriggerType::kInitialization));
    }

    bool get_pub_init() const { return pub_init_; }
    bool get_dis_update_init() const { return dis_update_init_; }
    bool get_unres_update_init() const { return unres_update_init_; }

   private:
    void InitPublish(const Context<double>&,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(event.get_trigger_type(),
                TriggerType::kInitialization);
      pub_init_ = true;
    }

    void DoCalcDiscreteVariableUpdates(
        const Context<double>&,
        const std::vector<const DiscreteUpdateEvent<double>*>& events,
        DiscreteValues<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                TriggerType::kInitialization);
      dis_update_init_ = true;
    }

    void DoCalcUnrestrictedUpdate(
        const Context<double>&,
        const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
        State<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                TriggerType::kInitialization);
      unres_update_init_ = true;
    }

    mutable bool pub_init_{false};
    mutable bool dis_update_init_{false};
    mutable bool unres_update_init_{false};
  };

  DiagramBuilder<double> builder;

  auto sys0 = builder.AddSystem<InitializationTestSystem>();
  auto sys1 = builder.AddSystem<InitializationTestSystem>();

  auto dut = builder.Build();

  auto context = dut->CreateDefaultContext();
  auto discrete_updates = dut->AllocateDiscreteVariables();
  auto state = context->CloneState();
  auto init_events = dut->AllocateCompositeEventCollection();
  dut->GetInitializationEvents(*context, init_events.get());

  dut->Publish(*context, init_events->get_publish_events());
  dut->CalcDiscreteVariableUpdates(*context,
                                   init_events->get_discrete_update_events(),
                                   discrete_updates.get());
  dut->CalcUnrestrictedUpdate(
      *context, init_events->get_unrestricted_update_events(), state.get());

  EXPECT_TRUE(sys0->get_pub_init());
  EXPECT_TRUE(sys0->get_dis_update_init());
  EXPECT_TRUE(sys0->get_unres_update_init());
  EXPECT_TRUE(sys1->get_pub_init());
  EXPECT_TRUE(sys1->get_dis_update_init());
  EXPECT_TRUE(sys1->get_unres_update_init());
}

// TODO(siyuan) add direct tests for EventCollection

// A System that does not override the default implicit time derivatives
// implementation.
class DefaultExplicitSystem : public LeafSystem<double> {
 public:
  DefaultExplicitSystem() { DeclareContinuousState(3); }

  static Vector3d fixed_derivative() { return {1., 2., 3.}; }

 private:
  void DoCalcTimeDerivatives(const Context<double>& context,
                             ContinuousState<double>* derivatives) const final {
    derivatives->SetFromVector(fixed_derivative());
  }
};

// A System that _does_ override the default implicit time derivatives
// implementation, and also changes the residual size from its default.
class OverrideImplicitSystem : public DefaultExplicitSystem {
 public:
  OverrideImplicitSystem() {
    DeclareImplicitTimeDerivativesResidualSize(1);
  }

 private:
  void DoCalcImplicitTimeDerivativesResidual(
    const systems::Context<double>& context,
    const systems::ContinuousState<double>& proposed_derivatives,
    EigenPtr<VectorX<double>> residual) const final {
    EXPECT_EQ(residual->size(), 1);
    (*residual)[0] = proposed_derivatives.CopyToVector().sum();
  }
};

// A Diagram should concatenate the implicit time derivatives from its
// component subsystems, and tally up the size correctly.
GTEST_TEST(ImplicitTimeDerivatives, DiagramProcessing) {
  const Vector3d derivs = DefaultExplicitSystem::fixed_derivative();

  DiagramBuilder<double> builder;
  builder.AddSystem<DefaultExplicitSystem>();   // 3 residuals
  builder.AddSystem<OverrideImplicitSystem>();  // 1 residual
  builder.AddSystem<DefaultExplicitSystem>();   // 3 more residuals
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  EXPECT_EQ(diagram->implicit_time_derivatives_residual_size(), 7);
  VectorXd residual = diagram->AllocateImplicitTimeDerivativesResidual();
  EXPECT_EQ(residual.size(), 7);

  auto xdot = diagram->AllocateTimeDerivatives();
  EXPECT_EQ(xdot->size(), 9);  // 3 derivatives each subsystem
  const auto xdot_vector = VectorXd::LinSpaced(9, 11., 19.);
  xdot->SetFromVector(xdot_vector);
  VectorXd expected_result(7);
  expected_result.segment<3>(0) = xdot_vector.segment<3>(0) - derivs;
  expected_result[3] = xdot_vector.segment<3>(3).sum();
  expected_result.segment<3>(4) = xdot_vector.segment<3>(6) - derivs;

  diagram->CalcImplicitTimeDerivativesResidual(*context, *xdot, &residual);
  EXPECT_EQ(residual, expected_result);
}

}  // namespace
}  // namespace systems
}  // namespace drake
