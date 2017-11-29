#include "drake/systems/framework/diagram.h"

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/analysis/test_utilities/stateless_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
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
  }
};

template <typename T>
void CheckPeriodAndOffset(const typename Event<T>::PeriodicAttribute& attr) {
  EXPECT_EQ(attr.period_sec, 1.125);
  EXPECT_EQ(attr.offset_sec, 2.25);
}

// Tests whether the diagram exhibits the correct behavior for
// GetUniquePeriodicDiscreteUpdateAttribute().
GTEST_TEST(EmptySystemDiagramTest, CheckPeriodicTriggerDiscreteUpdateUnique) {
  // Check diagrams with no recursion.
  optional<Event<double>::PeriodicAttribute> periodic_attr;
  EmptySystemDiagram d_sys2upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys1, 0, true);
  EmptySystemDiagram d_sys1upd_zero(
      EmptySystemDiagram::kOneUpdatePerLevelSys2, 0, true);
  EmptySystemDiagram d_bothupd_zero(EmptySystemDiagram::kTwoUpdatesPerLevel, 0,
      true);
  ASSERT_TRUE(periodic_attr =
      d_sys2upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset<double>(periodic_attr.value());
  ASSERT_TRUE(periodic_attr =
      d_sys1upd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset<double>(periodic_attr.value());
  ASSERT_TRUE(periodic_attr =
      d_bothupd_zero.GetUniquePeriodicDiscreteUpdateAttribute());
  CheckPeriodAndOffset<double>(periodic_attr.value());

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
    ASSERT_TRUE(periodic_attr =
        d_sys1upd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
    ASSERT_TRUE(periodic_attr =
        d_sys2upd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
    ASSERT_TRUE(periodic_attr =
        d_bothupd.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
    ASSERT_TRUE(periodic_attr =
        d_both_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
    ASSERT_TRUE(periodic_attr =
        d_sys1_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
    ASSERT_TRUE(periodic_attr =
        d_sys2_last.GetUniquePeriodicDiscreteUpdateAttribute());
    CheckPeriodAndOffset<double>(periodic_attr.value());
  }
}

// Tests whether the diagram exhibits the correct behavior for
// GetUniquePeriodicDiscreteUpdateAttribute() with non-unique updates
GTEST_TEST(EmptySystemDiagramTest, CheckPeriodicTriggerDiscreteUpdate) {
  // Check diagrams with no recursion.
  Event<double>::PeriodicAttribute periodic_attr;
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

/// ExampleDiagram has the following structure:
/// adder0_: (input0_ + input1_) -> A
/// adder1_: (A + input2_)       -> B, output 0
/// adder2_: (A + B)             -> output 1
/// integrator1_: A              -> C
/// integrator2_: C              -> output 2
/// It also uses an StatelessSystem to verify Diagram's ability to retrieve
/// witness functions from its subsystems.
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

    if (use_abstract) {
      builder.AddSystem<ConstantValueSource<double>>(
          std::make_unique<Value<int>>(0));
    }
    if (use_double_only) {
      builder.AddSystem<DoubleOnlySystem>();
    }

    builder.BuildInto(this);
  }

  Adder<double>* adder0() { return adder0_; }
  Adder<double>* adder1() { return adder1_; }
  Adder<double>* adder2() { return adder2_; }
  Integrator<double>* integrator0() { return integrator0_; }
  Integrator<double>* integrator1() { return integrator1_; }
  analysis_test::StatelessSystem<double>* stateless() { return stateless_; }

 private:
  Adder<double>* adder0_ = nullptr;
  Adder<double>* adder1_ = nullptr;
  Adder<double>* adder2_ = nullptr;
  analysis_test::StatelessSystem<double>* stateless_ = nullptr;

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
    auto& integrator0_xc = GetMutableContinuousState(integrator0());
    integrator0_xc.get_mutable_vector().SetAtIndex(0, 3);
    integrator0_xc.get_mutable_vector().SetAtIndex(1, 9);
    integrator0_xc.get_mutable_vector().SetAtIndex(2, 27);

    auto& integrator1_xc = GetMutableContinuousState(integrator1());
    integrator1_xc.get_mutable_vector().SetAtIndex(0, 81);
    integrator1_xc.get_mutable_vector().SetAtIndex(1, 243);
    integrator1_xc.get_mutable_vector().SetAtIndex(2, 729);
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

// Tests that the diagram returns the correct number of witness functions and
// that the witness function can be called correctly.
TEST_F(DiagramTest, Witness) {
  std::vector<const WitnessFunction<double>*> wf;
  diagram_->GetWitnessFunctions(*context_, &wf);

  // Stateless function always returns the ClockWitness.
  ASSERT_EQ(wf.size(), 1);
  EXPECT_TRUE(is_dynamic_castable<const analysis_test::ClockWitness<double>>(
      wf.front()));

  EXPECT_LT(diagram_->EvaluateWitness(*context_, *wf.front()), 0);
}

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

TEST_F(DiagramTest, Path) {
  const std::string path = adder0()->GetPath();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);
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
      "_" + id + "_u1[color=blue, label=\"u1\"")) << dot;
  EXPECT_NE(std::string::npos, dot.find(
      "_" + id + "_y2[color=green, label=\"y2\"")) << dot;
  // Check that subsystem records appear.
  EXPECT_NE(std::string::npos, dot.find(
      "[shape=record, label=\"adder1|{{<u0>u0|<u1>u1} | {<y0>y0}}\"]")) << dot;
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
      diagram_->ToAutoDiffXd();
  std::unique_ptr<Context<AutoDiffXd>> context =
      ad_diagram->CreateDefaultContext();
  std::unique_ptr<SystemOutput<AutoDiffXd>> output =
      ad_diagram->AllocateOutput(*context);

  // The name was preserved.
  EXPECT_EQ(diagram_->get_name(), ad_diagram->get_name());

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

  // When the Diagram contains a System that does not support AutoDiffXd,
  // we cannot transmogrify the Diagram to AutoDiffXd.
  const bool use_abstract = false;
  const bool use_double_only = true;
  auto diagram_with_double_only = std::make_unique<ExampleDiagram>(
      kSize, use_abstract, use_double_only);
  EXPECT_THROW(diagram_with_double_only->ToAutoDiffXd(), std::exception);
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
    Context<double>& d0_context =
        diagram_->GetMutableSubsystemContext(*subdiagram0_, context_.get());
    Context<double>& d1_context =
        diagram_->GetMutableSubsystemContext(*subdiagram1_, context_.get());

    State<double>& integrator0_x = subdiagram0_->GetMutableSubsystemState(
        *subdiagram0_->integrator0(), &d0_context);
    integrator0_x.get_mutable_continuous_state()
        .get_mutable_vector().SetAtIndex(0, 3);

    State<double>& integrator1_x = subdiagram0_->GetMutableSubsystemState(
        *subdiagram0_->integrator1(), &d0_context);
    integrator1_x.get_mutable_continuous_state()
        .get_mutable_vector().SetAtIndex(0, 9);

    State<double>& integrator2_x = subdiagram1_->GetMutableSubsystemState(
        *subdiagram1_->integrator0(), &d1_context);
    integrator2_x.get_mutable_continuous_state()
        .get_mutable_vector().SetAtIndex(0, 27);

    State<double>& integrator3_x = subdiagram1_->GetMutableSubsystemState(
        *subdiagram1_->integrator1(), &d1_context);
    integrator3_x.get_mutable_continuous_state()
        .get_mutable_vector().SetAtIndex(0, 81);
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
}

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
  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override {
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
      : Diagram<T>(SystemTypeTag<systems::FeedbackDiagram>{}) {
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
  EXPECT_NO_THROW(context.reset());
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
    feedthrough_input_ = this->DeclareInputPort(kVectorValued, 1).get_index();
    sink_input_ = this->DeclareInputPort(kVectorValued, 1).get_index();
    this->DeclareVectorOutputPort(BasicVector<double>(1),
                                  &Reduce::CalcFeedthrough);
  }

  const systems::InputPortDescriptor<double>& get_sink_input() {
    return this->get_input_port(sink_input_);
  }

  const systems::InputPortDescriptor<double>& get_feedthrough_input() {
    return this->get_input_port(feedthrough_input_);
  }

  void CalcFeedthrough(const Context<double>& context,
                       BasicVector<double>* output) const {
    const BasicVector<double>* input_vector =
        this->EvalVectorInput(context, feedthrough_input_);
    output->get_mutable_value() = input_vector->get_value();
  }

  optional<bool> DoHasDirectFeedthrough(int input_port, int) const override {
    return input_port == feedthrough_input_;
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
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareInputPort(kVectorValued, 1,
                           RandomDistribution::kUniform);
    this->DeclareInputPort(kVectorValued, 1,
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
        &context->get_mutable_continuous_state_vector());
  }

 protected:
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
  EXPECT_EQ(qdot.GetAtIndex(0), 26);
  EXPECT_EQ(qdot.GetAtIndex(1), 34);

  // Now map the configuration derivatives back to v.
  BasicVector<double> vmutable(v.size());
  diagram.MapQDotToVelocity(*context, qdot, &vmutable);
  EXPECT_EQ(vmutable.GetAtIndex(0), 13);
  EXPECT_EQ(vmutable.GetAtIndex(1), 17);
}

// Test for GetSystems.
GTEST_TEST(GetSystemsTest, GetSystems) {
  auto diagram = std::make_unique<ExampleDiagram>(2);
  EXPECT_EQ((std::vector<const System<double>*>{
                diagram->adder0(), diagram->adder1(), diagram->adder2(),
                diagram->stateless(),
                diagram->integrator0(), diagram->integrator1()
            }),
            diagram->GetSystems());
}

const double kTestPublishPeriod = 19.0;

class TestPublishingSystem : public LeafSystem<double> {
 public:
  TestPublishingSystem() {
    this->DeclarePeriodicPublish(kTestPublishPeriod);

    // Verify that no periodic discrete updates are registered.
    EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
  }

  ~TestPublishingSystem() override {}

  bool published() { return published_; }

 protected:
  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
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
    hold1_->set_name("hold1");
    hold2_ = builder.template AddSystem<ZeroOrderHold<double>>(3.0, kSize);
    hold2_->set_name("hold2");
    publisher_ = builder.template AddSystem<TestPublishingSystem>();
    publisher_->set_name("publisher");
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
  context_->set_time(5.1);
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
  ctx1.get_mutable_discrete_state(0).SetAtIndex(0, 1001.0);
  Context<double>& ctx2 =
      diagram_.GetMutableSubsystemContext(*diagram_.hold2(), context_.get());
  ctx2.get_mutable_discrete_state(0).SetAtIndex(0, 1002.0);

  // Allocate the discrete variables.
  std::unique_ptr<DiscreteValues<double>> updates =
      diagram_.AllocateDiscreteVariables();

  // Set the time to 8.5, so only hold2 updates.
  context_->set_time(8.5);

  // Request the next update time.
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(9.0, time);
  EXPECT_TRUE(events->HasDiscreteUpdateEvents());

  // Fast forward to 9.0 sec and do the update.
  context_->set_time(9.0);
  diagram_.CalcDiscreteVariableUpdates(
      *context_, events->get_discrete_update_events(), updates.get());
  context_->get_mutable_discrete_state().SetFrom(*updates);
  EXPECT_EQ(1001.0, ctx1.get_discrete_state(0).GetAtIndex(0));
  EXPECT_EQ(23.0, ctx2.get_discrete_state(0).GetAtIndex(0));

  // Restore hold2 to its original value.
  ctx2.get_mutable_discrete_state(0).SetAtIndex(0, 1002.0);
  // Set the time to 11.5, so both hold1 and hold2 update.
  context_->set_time(11.5);
  time = diagram_.CalcNextUpdateTime(*context_, events.get());
  EXPECT_EQ(12.0, time);
  EXPECT_TRUE(events->HasDiscreteUpdateEvents());

  // Fast forward to 12.0 sec and do the update again.
  context_->set_time(12.0);
  diagram_.CalcDiscreteVariableUpdates(
      *context_, events->get_discrete_update_events(), updates.get());
  context_->get_mutable_discrete_state().SetFrom(*updates);
  EXPECT_EQ(17.0, ctx1.get_discrete_state(0).GetAtIndex(0));
  EXPECT_EQ(23.0, ctx2.get_discrete_state(0).GetAtIndex(0));
}

// Tests that a publish action is taken at 19 sec.
TEST_F(DiscreteStateTest, Publish) {
  context_->set_time(18.5);
  auto events = diagram_.AllocateCompositeEventCollection();
  double time = diagram_.CalcNextUpdateTime(*context_, events.get());

  EXPECT_EQ(19.0, time);
  EXPECT_TRUE(events->HasPublishEvents());

  // Fast forward to 19.0 sec and do the publish.
  EXPECT_EQ(false, diagram_.publisher()->published());
  context_->set_time(19.0);
  diagram_.Publish(*context_, events->get_publish_events());
  // Check that publication occurred.
  EXPECT_EQ(true, diagram_.publisher()->published());
}

class SystemWithAbstractState : public LeafSystem<double> {
 public:
  SystemWithAbstractState(int id, double update_period) : id_(id) {
    DeclarePeriodicUnrestrictedUpdate(update_period, 0);

    // Verify that no periodic discrete updates are registered.
    Event<double>::PeriodicAttribute attr;
    EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
  }

  ~SystemWithAbstractState() override {}

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    std::vector<std::unique_ptr<AbstractValue>> values;
    values.push_back({PackValue<double>(id_)});
    return std::make_unique<AbstractValues>(std::move(values));
  }

  // Abstract state is set to time + id.
  void DoCalcUnrestrictedUpdate(
      const Context<double>& context,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const override {
    double& state_num = state->get_mutable_abstract_state()
                            .get_mutable_value(0)
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
    sys0_->set_name("sys0");
    sys1_ = builder.template AddSystem<SystemWithAbstractState>(1, 3.);
    sys1_->set_name("sys1");
    builder.BuildInto(this);
  }

  const SystemWithAbstractState& get_sys(int i) const {
    if (i == 0)
      return *sys0_;
    return *sys1_;
  }

  SystemWithAbstractState* get_mutable_sys0() { return sys0_; }
  SystemWithAbstractState* get_mutable_sys1() { return sys1_; }

 private:
  SystemWithAbstractState* sys0_{nullptr};
  SystemWithAbstractState* sys1_{nullptr};
};

class AbstractStateDiagramTest : public ::testing::Test {
 protected:
  void SetUp() override { context_ = diagram_.CreateDefaultContext(); }

  double get_sys0_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*diagram_.get_mutable_sys0(), *context_);
    return sys_context.get_abstract_state<double>(0);
  }

  double get_sys1_abstract_data_as_double() {
    const Context<double>& sys_context =
        diagram_.GetSubsystemContext(*diagram_.get_mutable_sys1(), *context_);
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
  auto events = diagram_.AllocateCompositeEventCollection();
  EXPECT_EQ(diagram_.CalcNextUpdateTime(*context_, events.get()), 2.);
  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(0), *events);
    EXPECT_TRUE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }
  {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(1), *events);
    EXPECT_FALSE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }

  // Creates a temp state and does unrestricted updates.
  std::unique_ptr<State<double>> x_buf = context_->CloneState();
  diagram_.CalcUnrestrictedUpdate(
      *context_, events->get_unrestricted_update_events(), x_buf.get());

  // The abstract data in the current context should be the same as before.
  EXPECT_EQ(get_sys0_abstract_data_as_double(), 0);
  EXPECT_EQ(get_sys1_abstract_data_as_double(), 1);

  // Swaps in the new state, and the abstract data for sys0 should be updated.
  context_->get_mutable_state().CopyFrom(*x_buf);
  EXPECT_EQ(get_sys0_abstract_data_as_double(), (time + 0));
  EXPECT_EQ(get_sys1_abstract_data_as_double(), 1);

  // Sets time to 5.5, both system should be updating at 6 sec.
  time = 5.5;
  context_->set_time(time);
  EXPECT_EQ(diagram_.CalcNextUpdateTime(*context_, events.get()), 6.);
  for (int i = 0; i < 2; i++) {
    const auto& subevent_collection =
        diagram_.GetSubsystemCompositeEventCollection(
            diagram_.get_sys(i), *events);
    EXPECT_TRUE(
        subevent_collection.get_unrestricted_update_events().HasEvents());
  }

  diagram_.CalcUnrestrictedUpdate(
      *context_, events->get_unrestricted_update_events(), x_buf.get());
  // Both sys0 and sys1's abstract data should be updated.
  context_->get_mutable_state().CopyFrom(*x_buf);
  EXPECT_EQ(get_sys0_abstract_data_as_double(), (time + 0));
  EXPECT_EQ(get_sys1_abstract_data_as_double(), (time + 1));
}

// Test diagram. Top level diagram (big_diagram) has 3 components:
// diagram0, int2, diagram1, where diagram0 has int0 and int1, and
// diagram1 has int3.
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
    big_output_ = big_diagram_->AllocateOutput(*big_context_);
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

// Sets the continuous state of all the integrators through
// GetMutableSubsystemContext(), and check that they are correctly set.
TEST_F(NestedDiagramContextTest, GetSubsystemContext) {
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 0);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 0);

  big_diagram_->GetMutableSubsystemContext(*integrator0_, big_context_.get())
      .get_mutable_continuous_state_vector()
      .SetAtIndex(0, 1);
  big_diagram_->GetMutableSubsystemContext(*integrator1_, big_context_.get())
      .get_mutable_continuous_state_vector()
      .SetAtIndex(0, 2);
  big_diagram_->GetMutableSubsystemContext(*integrator2_, big_context_.get())
      .get_mutable_continuous_state_vector()
      .SetAtIndex(0, 3);
  big_diagram_->GetMutableSubsystemContext(*integrator3_, big_context_.get())
      .get_mutable_continuous_state_vector()
      .SetAtIndex(0, 4);

  // Checks states.
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator0_, *big_context_)
                .get_continuous_state_vector()
                .GetAtIndex(0),
            1);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator1_, *big_context_)
                .get_continuous_state_vector()
                .GetAtIndex(0),
            2);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator2_, *big_context_)
                .get_continuous_state_vector()
                .GetAtIndex(0),
            3);
  EXPECT_EQ(big_diagram_->GetSubsystemContext(*integrator3_, *big_context_)
                .get_continuous_state_vector()
                .GetAtIndex(0),
            4);

  // Checks output.
  big_diagram_->CalcOutput(*big_context_, big_output_.get());

  EXPECT_EQ(big_output_->get_vector_data(0)->GetAtIndex(0), 1);
  EXPECT_EQ(big_output_->get_vector_data(1)->GetAtIndex(0), 2);
  EXPECT_EQ(big_output_->get_vector_data(2)->GetAtIndex(0), 3);
  EXPECT_EQ(big_output_->get_vector_data(3)->GetAtIndex(0), 4);
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
      .get_mutable_vector()
      .SetAtIndex(0, 1);
  big_diagram_
      ->GetMutableSubsystemState(*integrator1_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()
      .SetAtIndex(0, 2);
  big_diagram_
      ->GetMutableSubsystemState(*integrator2_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()
      .SetAtIndex(0, 3);
  big_diagram_
      ->GetMutableSubsystemState(*integrator3_, &big_state)
      .get_mutable_continuous_state()
      .get_mutable_vector()
      .SetAtIndex(0, 4);

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
  EXPECT_NO_THROW(builder.Build());
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

// A system for testing per step actions.
class PerStepActionTestSystem : public LeafSystem<double> {
 public:
  PerStepActionTestSystem() {
    DeclareDiscreteState(1);
    DeclareAbstractState(AbstractValue::Make<std::string>(""));
  }

  template <typename EventType>
  void AddPerStepEvent() {
    EventType event(Event<double>::TriggerType::kPerStep);
    this->DeclarePerStepEvent(event);
  }

  int get_publish_ctr() const { return publish_ctr_; }

 private:
  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {
    state->get_mutable_discrete_state()[0] = 0;
    state->get_mutable_abstract_state<std::string>(0) = "wow";
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>* discrete_state) const override {
    (*discrete_state)[0] = context.get_discrete_state(0).GetAtIndex(0) + 1;
  }

  void DoCalcUnrestrictedUpdate(
      const Context<double>& context,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const override {
    int int_num =
        static_cast<int>(context.get_discrete_state(0).GetAtIndex(0));
    state->get_mutable_abstract_state<std::string>(0) =
        "wow" + std::to_string(int_num);
  }

  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
    publish_ctr_++;
  }

  // A hack to test publish calls easily.
  mutable int publish_ctr_{0};
};

// Builds a nested diagram and tests per step publish, discrete and
// unrestricted updates.
GTEST_TEST(DiagramPerStepActionTest, TestEverything) {
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

    sys1->AddPerStepEvent<DiscreteUpdateEvent<double>>();
    sys1->AddPerStepEvent<UnrestrictedUpdateEvent<double>>();

    sub_diagram = builder.Build();
    sub_diagram->set_name("sub_diagram");
  }

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(sub_diagram));
  sys2 = builder.AddSystem<PerStepActionTestSystem>();
  sys2->set_name("sys2");

  // sys2 has publish and unrestricted updates.
  sys2->AddPerStepEvent<PublishEvent<double>>();
  sys2->AddPerStepEvent<UnrestrictedUpdateEvent<double>>();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->set_name("diagram");

  auto tmp_discrete_state = diagram->AllocateDiscreteVariables();
  std::unique_ptr<State<double>> tmp_state = context->CloneState();

  auto events = diagram->AllocateCompositeEventCollection();
  diagram->GetPerStepEvents(*context, events.get());

  // Does unrestricted update first.
  diagram->CalcUnrestrictedUpdate(
      *context, events->get_unrestricted_update_events(), tmp_state.get());
  context->get_mutable_state().CopyFrom(*tmp_state);

  // Does discrete updates second.
  diagram->CalcDiscreteVariableUpdates(*context,
                                       events->get_discrete_update_events(),
                                       tmp_discrete_state.get());
  context->get_mutable_discrete_state().SetFrom(*tmp_discrete_state);

  // Publishes last.
  diagram->Publish(*context, events->get_publish_events());

  // Only sys2 published once.
  EXPECT_EQ(sys0->get_publish_ctr(), 0);
  EXPECT_EQ(sys1->get_publish_ctr(), 0);
  EXPECT_EQ(sys2->get_publish_ctr(), 1);

  // sys0 doesn't have any updates.
  auto& sys0_context = diagram->GetSubsystemContext(*sys0, *context);
  EXPECT_EQ(sys0_context.get_discrete_state(0).GetAtIndex(0), 0);
  EXPECT_EQ(sys0_context.get_abstract_state<std::string>(0), "wow");

  // sys1 should have an unrestricted update then a discrete update.
  auto& sys1_context = diagram->GetSubsystemContext(*sys1, *context);
  EXPECT_EQ(sys1_context.get_discrete_state(0).GetAtIndex(0), 1);
  EXPECT_EQ(sys1_context.get_abstract_state<std::string>(0), "wow0");

  // sys2 should have a unrestricted update then a publish.
  auto& sys2_context = diagram->GetSubsystemContext(*sys2, *context);
  EXPECT_EQ(sys2_context.get_discrete_state(0).GetAtIndex(0), 0);
  EXPECT_EQ(sys2_context.get_abstract_state<std::string>(0), "wow0");
}

class MyEventTestSystem : public LeafSystem<double> {
 public:
  // If p > 0, declares a periodic publish event with p. Otherwise, declares
  // a per step publish event.
  MyEventTestSystem(const std::string& name, double p) {
    if (p > 0) {
      DeclarePeriodicPublish(p);

      // Verify that no periodic discrete updates are registered.
      Event<double>::PeriodicAttribute attr;
      EXPECT_FALSE(this->GetUniquePeriodicDiscreteUpdateAttribute());
    } else {
      DeclarePerStepEvent<PublishEvent<double>>(
          PublishEvent<double>(Event<double>::TriggerType::kPerStep));
    }
    set_name(name);
  }

  int get_periodic_count() const { return periodic_count_; }

  int get_per_step_count() const { return per_step_count_; }

 private:
  void DoPublish(
      const Context<double>& context,
      const std::vector<const PublishEvent<double>*>& events) const override {
    for (const PublishEvent<double>* event : events) {
      if (event->get_trigger_type() ==
          Event<double>::TriggerType::kPeriodic) {
        periodic_count_++;
      } else if (event->get_trigger_type() ==
          Event<double>::TriggerType::kPerStep) {
        per_step_count_++;
      } else {
        DRAKE_ABORT();
      }
    }
  }

  mutable int periodic_count_{0};
  mutable int per_step_count_{0};
};

GTEST_TEST(MyEventTest, MyEventTestLeaf) {
  MyEventTestSystem dut("sys", 0.2);
  auto events = dut.AllocateCompositeEventCollection();
  auto context = dut.CreateDefaultContext();

  double time = dut.CalcNextUpdateTime(*context, events.get());
  context->set_time(time);
  dut.Publish(*context, events->get_publish_events());

  EXPECT_EQ(dut.get_periodic_count(), 1);
  EXPECT_EQ(dut.get_per_step_count(), 0);
}

// Builds a diagram with a sub diagram (has 3 MyEventTestSystem) and 2
// MyEventTestSystem. sys4 is configured to have per step events, and all
// the others should have periodic publish events. Tests
// Diagram::CalcNextUpdateTime, Diagram::GetPerStepEvents, and
// CompositeEventCollection::Merge. The result should be sys1, sys2, sys3, sys4
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
  auto perstep_events = dut->AllocateCompositeEventCollection();
  auto events = dut->AllocateCompositeEventCollection();

  auto context = dut->CreateDefaultContext();

  double time = dut->CalcNextUpdateTime(*context, periodic_events.get());
  dut->GetPerStepEvents(*context, perstep_events.get());

  events->Merge(*periodic_events);
  events->Merge(*perstep_events);

  context->set_time(time);
  dut->Publish(*context, events->get_publish_events());

  EXPECT_EQ(sys[0]->get_periodic_count(), 0);
  EXPECT_EQ(sys[0]->get_per_step_count(), 0);

  EXPECT_EQ(sys[1]->get_periodic_count(), 1);
  EXPECT_EQ(sys[1]->get_per_step_count(), 0);

  EXPECT_EQ(sys[2]->get_periodic_count(), 1);
  EXPECT_EQ(sys[2]->get_per_step_count(), 0);

  EXPECT_EQ(sys[3]->get_periodic_count(), 1);
  EXPECT_EQ(sys[3]->get_per_step_count(), 0);

  EXPECT_EQ(sys[4]->get_periodic_count(), 0);
  EXPECT_EQ(sys[4]->get_per_step_count(), 1);
}

template <typename T>
class ConstraintTestSystem : public LeafSystem<T> {
 public:
  ConstraintTestSystem()
      : LeafSystem<T>(systems::SystemTypeTag<systems::ConstraintTestSystem>{}) {
    this->DeclareContinuousState(2);
    this->DeclareEqualityConstraint(&ConstraintTestSystem::CalcState0Constraint,
                                    1, "x0");
    this->DeclareInequalityConstraint(
        &ConstraintTestSystem::CalcStateConstraint, 2, "x");
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
    *value = Vector1<T>(context.get_continuous_state_vector().GetAtIndex(0));
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
  EXPECT_EQ(diagram->get_num_constraints(), 4);  // two from each system

  auto context = diagram->CreateDefaultContext();

  // Set sys1 context.
  diagram->GetMutableSubsystemContext(*sys1, context.get())
      .get_mutable_continuous_state_vector()
      .SetFromVector(Eigen::Vector2d(5.0, 7.0));

  // Set sys2 context.
  diagram->GetMutableSubsystemContext(*sys2, context.get())
      .get_mutable_continuous_state_vector()
      .SetFromVector(Eigen::Vector2d(11.0, 12.0));

  Eigen::VectorXd value;
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
  EXPECT_EQ(autodiff_diagram->get_num_constraints(), 4);
  auto autodiff_context = autodiff_diagram->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(*context);
  const SystemConstraint<AutoDiffXd>& autodiff_constraint =
      autodiff_diagram->get_constraint(SystemConstraintIndex(3));
  VectorX<AutoDiffXd> autodiff_value;
  autodiff_constraint.Calc(*autodiff_context, &autodiff_value);
  EXPECT_EQ(autodiff_value[0].value(), 11.0);

  // Check that constraints survive ToSymbolic.
  auto symbolic_diagram = diagram->ToSymbolic();
  EXPECT_EQ(symbolic_diagram->get_num_constraints(), 4);
  auto symbolic_context = symbolic_diagram->CreateDefaultContext();
  symbolic_context->SetTimeStateAndParametersFrom(*context);
  const SystemConstraint<symbolic::Expression>& symbolic_constraint =
      symbolic_diagram->get_constraint(SystemConstraintIndex(3));
  VectorX<symbolic::Expression> symbolic_value;
  symbolic_constraint.Calc(*symbolic_context, &symbolic_value);
  EXPECT_EQ(symbolic_value[0], 11.0);
}

GTEST_TEST(DiagramParametersTest, ParameterTest) {
  // Construct a diagram with multiple subsytems that have parameters.
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
        BasicVector<double>(Eigen::Vector2d(-1.0, -2.0)));
    this->DeclareNumericParameter(
        BasicVector<double>(Eigen::Vector3d(1.0, 2.0, 3.0)));
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
      params->get_mutable_numeric_parameter(0).SetAtIndex(i,
                                                          uniform(*generator));
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
  Eigen::Vector4d state = context->get_continuous_state_vector().CopyToVector();
  Eigen::Vector3d params0 = context->get_numeric_parameter(0).CopyToVector();
  Eigen::Vector3d params1 = context->get_numeric_parameter(1).CopyToVector();

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
          Event<double>::TriggerType::kInitialization,
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);

      DeclareInitializationEvent(DiscreteUpdateEvent<double>(
          Event<double>::TriggerType::kInitialization));
      DeclareInitializationEvent(UnrestrictedUpdateEvent<double>(
          Event<double>::TriggerType::kInitialization));
    }

    bool get_pub_init() const { return pub_init_; }
    bool get_dis_update_init() const { return dis_update_init_; }
    bool get_unres_update_init() const { return unres_update_init_; }

   private:
    void InitPublish(const Context<double>&,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(event.get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
      pub_init_ = true;
    }

    void DoCalcDiscreteVariableUpdates(
        const Context<double>&,
        const std::vector<const DiscreteUpdateEvent<double>*>& events,
        DiscreteValues<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
      dis_update_init_ = true;
    }

    void DoCalcUnrestrictedUpdate(
        const Context<double>&,
        const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
        State<double>*) const final {
      EXPECT_EQ(events.size(), 1);
      EXPECT_EQ(events.front()->get_trigger_type(),
                Event<double>::TriggerType::kInitialization);
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

}  // namespace
}  // namespace systems
}  // namespace drake
