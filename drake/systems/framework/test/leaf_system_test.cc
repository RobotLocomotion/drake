#include "drake/systems/framework/leaf_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {
namespace {

// A shell System to test the default implementations.
template<typename T>
class TestSystem : public LeafSystem<T> {
 public:
  TestSystem() {
    this->set_name("TestSystem");
    this->DeclareOutputPort(kVectorValued, 17);
    this->DeclareAbstractOutputPort();
  }
  ~TestSystem() override {}

  void AddPeriodicUpdate() {
    const double period = 10.0;
    const double offset = 5.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }

  void AddPeriodicUpdate(double period) {
    const double offset = 0.0;
    this->DeclarePeriodicDiscreteUpdate(period, offset);
  }

  void AddPeriodicUnrestrictedUpdate(double period, double offset) {
    this->DeclarePeriodicUnrestrictedUpdate(period, offset);
  }

  void AddPublish(double period) {
    this->DeclarePublishPeriodSec(period);
  }

  void AddContinuousState() {
    this->DeclareContinuousState(4, 3, 2);
  }

  void AddContinuousState(std::unique_ptr<BasicVector<T>> vec) {
    this->DeclareContinuousState(std::move(vec), 4, 3, 2);
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {}

  void DoCalcTimeDerivatives(
      const Context<T>& context,
      ContinuousState<T>* derivatives) const override {}

  std::unique_ptr<Parameters<T>> AllocateParameters() const override {
    return std::make_unique<Parameters<T>>(
        std::make_unique<BasicVector<T>>(2));
  }

  std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const OutputPortDescriptor<T>& descriptor) const override {
    return std::make_unique<BasicVector<T>>(17);
  }

  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<T>& descriptor) const override {
    return AbstractValue::Make<int>(42);
  }

  void SetDefaultParameters(const LeafContext<T>& context,
                            Parameters<T>* params) const override {
    BasicVector<T>* param = params->get_mutable_numeric_parameter(0);
    Vector2<T> p0;
    p0 << 13.0, 7.0;
    param->SetFromVector(p0);
  }

  const BasicVector<T>& GetVanillaNumericParameters(
      const Context<T>& context) const {
    return this->GetNumericParameter(context, 0 /* index */);
  }
};

class LeafSystemTest : public ::testing::Test {
 protected:
  TestSystem<double> system_;
  LeafContext<double> context_;
};

// Tests that if no update events are configured, none are reported.
TEST_F(LeafSystemTest, NoUpdateEvents) {
  context_.set_time(25.0);
  UpdateActions<double> actions;
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), actions.time);
  EXPECT_EQ(0u, actions.events.size());
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset.
TEST_F(LeafSystemTest, OffsetHasNotArrivedYet) {
  context_.set_time(2.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(5.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is smaller than the offset, the next
// update time is the offset, DiscreteUpdate and UnrestrictedUpdate happen
// at the same time.
TEST_F(LeafSystemTest, EventsAtTheSameTime) {
  context_.set_time(2.0);
  UpdateActions<double> actions;
  // Both actions happen at t = 5.
  system_.AddPeriodicUpdate();
  system_.AddPeriodicUnrestrictedUpdate(3, 5);
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(5.0, actions.time);
  ASSERT_EQ(2u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
  EXPECT_EQ(DiscreteEvent<double>::kUnrestrictedUpdateAction,
            actions.events[1].action);
}

// Tests that if the current time is exactly the offset, the next
// update time is in the future.
TEST_F(LeafSystemTest, ExactlyAtOffset) {
  context_.set_time(5.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(15.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is larger than the offset, the next
// update time is determined by the period.
TEST_F(LeafSystemTest, OffsetIsInThePast) {
  context_.set_time(23.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(25.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the current time is exactly an update time, the next update
// time is in the future.
TEST_F(LeafSystemTest, ExactlyOnUpdateTime) {
  context_.set_time(25.0);
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate();
  system_.CalcNextUpdateTime(context_, &actions);

  EXPECT_EQ(35.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if a LeafSystem has both a discrete update and a periodic Publish,
// the update actions are computed appropriately.
TEST_F(LeafSystemTest, UpdateAndPublish) {
  system_.AddPeriodicUpdate(15.0);
  system_.AddPublish(12.0);

  UpdateActions<double> actions;

  // The publish event fires at 12sec.
  context_.set_time(9.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(12.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kPublishAction, actions.events[0].action);

  // The update event fires at 15sec.
  context_.set_time(14.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(15.0, actions.time);
  ASSERT_EQ(1u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);

  // Both events fire at 60sec.
  context_.set_time(59.0);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_EQ(60.0, actions.time);
  ASSERT_EQ(2u, actions.events.size());
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction,
            actions.events[0].action);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroOneFive) {
  context_.set_time(0.015 * 11);  // Slightly less than 0.165.
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate(0.015);
  system_.CalcNextUpdateTime(context_, &actions);
  // 0.015 * 12 = 0.18.
  EXPECT_NEAR(0.18, actions.time, 1e-8);
}

// Tests that if the integrator has stopped on the k-th sample, and the current
// time for that sample is slightly less than k * period due to floating point
// rounding, the next sample time is (k + 1) * period.
TEST_F(LeafSystemTest, FloatingPointRoundingZeroPointZeroZeroTwoFive) {
  context_.set_time(0.0025 * 977);  // Slightly less than 2.4425
  UpdateActions<double> actions;
  system_.AddPeriodicUpdate(0.0025);
  system_.CalcNextUpdateTime(context_, &actions);
  EXPECT_NEAR(2.445, actions.time, 1e-8);
}

// Tests that the leaf system reserved the declared Parameters with default
// values.
TEST_F(LeafSystemTest, Parameters) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const BasicVector<double>& vec =
      system_.GetVanillaNumericParameters(*context);
  EXPECT_EQ(13.0, vec[0]);
  EXPECT_EQ(7.0, vec[1]);
}

// Tests that the leaf system reserved the declared continuous state, of
// vanilla type.
TEST_F(LeafSystemTest, DeclareVanillaContinuousState) {
  system_.AddContinuousState();
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  EXPECT_EQ(4 + 3 + 2, xc->size());
  EXPECT_EQ(4, xc->get_generalized_position().size());
  EXPECT_EQ(3, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the leaf system reserved the declared continuous state, of
// interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedContinuousState) {
  using MyVector9d = MyVector<4 + 3 + 2, double>;

  system_.AddContinuousState(std::make_unique<MyVector9d>());
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<MyVector9d*>(
                         context->get_mutable_continuous_state_vector()));
  // Check that dimensions were preserved.
  EXPECT_EQ(4 + 3 + 2, xc->size());
  EXPECT_EQ(4, xc->get_generalized_position().size());
  EXPECT_EQ(3, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
}

// Tests that the vector-valued output has been allocated with the correct
// dimensions.
TEST_F(LeafSystemTest, DeclareVectorOutput) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  auto output = system_.AllocateOutput(*context);
  EXPECT_EQ(17, output->get_vector_data(0)->size());
}

// Tests that the abstract-valued output has been allocated with the correct
// type.
TEST_F(LeafSystemTest, DeclareAbstractOutput) {
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  auto output = system_.AllocateOutput(*context);
  EXPECT_EQ(42, UnpackIntValue(output->get_data(1)));
}

// Tests both that an unrestricted update callback is called and that
// modifications to state dimension are caught.
TEST_F(LeafSystemTest, CallbackAndInvalidUpdates) {
  // Create 9, 1, and 3 dimensional continuous, discrete, and abstract state
  // vectors.
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  context->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));
  context->set_discrete_state(
    std::make_unique<DiscreteState<double>>(
      std::make_unique<BasicVector<double>>(1)));
  std::vector<std::unique_ptr<AbstractValue>> abstract_data;
  abstract_data.push_back(PackValue(3));
  abstract_data.push_back(PackValue(5));
  abstract_data.push_back(PackValue(7));
  context->set_abstract_state(
        std::make_unique<AbstractState>(std::move(abstract_data)));

  // Copy the state.
  std::unique_ptr<State<double>> x = context->CloneState();

  // Create an unrestricted update callback that just copies the state.
  DiscreteEvent<double> event;
  event.action = DiscreteEvent<double>::kUnrestrictedUpdateAction;
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
  };

  // Verify no exception is thrown.
  EXPECT_NO_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()));

  // Change the function to change the continuous state dimension.
  // Call the unrestricted update function again, now verifying that an
  // exception is thrown.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
    s->set_continuous_state(
      std::make_unique<ContinuousState<double>>(
        std::make_unique<BasicVector<double>>(4), 4, 0, 0));
  };

  // Call the unrestricted update function, verifying that an exception
  // is thrown
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);

  // Restore the continuous state (size).
  x->set_continuous_state(
    std::make_unique<ContinuousState<double>>(
      std::make_unique<BasicVector<double>>(9), 3, 3, 3));

  // Change the event to indicate to change the discrete state dimension.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    std::vector<std::unique_ptr<BasicVector<double>>> disc_data;
    s->CopyFrom(*c.CloneState());
    disc_data.push_back(std::make_unique<BasicVector<double>>(1));
    disc_data.push_back(std::make_unique<BasicVector<double>>(1));
    s->set_discrete_state(
         std::make_unique<DiscreteState<double>>(std::move(disc_data)));
  };

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);

  // Restore the discrete state (size).
  x->set_discrete_state(
    std::make_unique<DiscreteState<double>>(
      std::make_unique<BasicVector<double>>(1)));

  // Change the event to indicate to change the abstract state dimension.
  event.do_unrestricted_update = [](const Context<double>& c,
                                  State<double>* s) {
    s->CopyFrom(*c.CloneState());
    s->set_abstract_state(std::make_unique<AbstractState>());
  };

  // Call the unrestricted update function again, again verifying that an
  // exception is thrown.
  EXPECT_THROW(system_.CalcUnrestrictedUpdate(*context, event, x.get()),
               std::logic_error);
}

// Tests that the next update time is computed correctly for LeafSystems
// templated on AutoDiffXd. Protects against regression on #4431.
GTEST_TEST(AutodiffLeafSystemTest, NextUpdateTimeAutodiff) {
  TestSystem<AutoDiffXd> system;
  LeafContext<AutoDiffXd> context;

  context.set_time(21.0);
  UpdateActions<AutoDiffXd> actions;
  system.AddPeriodicUpdate();
  system.CalcNextUpdateTime(context, &actions);

  EXPECT_EQ(25.0, actions.time);
}

// A LeafSystem that uses the default, conservative direct-feedthrough
// implementation, informed by neither symbolic sparsity analysis nor manual
// sparsity declarations.
class DefaultFeedthroughSystem : public LeafSystem<double> {
 public:
  DefaultFeedthroughSystem() {}

  ~DefaultFeedthroughSystem() override {}

  void AddAbstractInputPort() {
    this->DeclareAbstractInputPort();
  }

  void AddAbstractOutputPort() {
    this->DeclareAbstractOutputPort();
  }

 protected:
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}
};


GTEST_TEST(FeedthroughTest, DefaultWithNoInputsOrOutputs) {
  DefaultFeedthroughSystem system;
  EXPECT_FALSE(system.HasAnyDirectFeedthrough());
}

GTEST_TEST(FeedthroughTest, DefaultWithBothInputsAndOutputs) {
  DefaultFeedthroughSystem system;
  system.AddAbstractInputPort();
  system.AddAbstractOutputPort();
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 0));
}

GTEST_TEST(FeedthroughTest, DefaultWithInputsOnly) {
  DefaultFeedthroughSystem input_only;
  input_only.AddAbstractInputPort();
  EXPECT_FALSE(input_only.HasAnyDirectFeedthrough());
}

GTEST_TEST(FeedthroughTest, DefaultWithOutputsOnly) {
  DefaultFeedthroughSystem output_only;
  output_only.AddAbstractOutputPort();
  EXPECT_FALSE(output_only.HasAnyDirectFeedthrough());
  EXPECT_FALSE(output_only.HasDirectFeedthrough(0));
}

// A MIMO system with manually-configured direct feedthrough properties: input
// 0 affects only output 1, and input 1 affects only output 0.
class ManualSparsitySystem : public DefaultFeedthroughSystem {
 public:
  ManualSparsitySystem() {
    this->AddAbstractInputPort();
    this->AddAbstractInputPort();
    this->AddAbstractOutputPort();
    this->AddAbstractOutputPort();
  }

 protected:
  bool DoHasDirectFeedthrough(const SparsityMatrix* sparsity,
                              int input_port,
                              int output_port) const override {
    if (input_port == 0 && output_port == 1) {
      return true;
    }
    if (input_port == 1 && output_port == 0) {
      return true;
    }
    return false;
  }
};

GTEST_TEST(FeedthroughTest, ManualSparsity) {
  ManualSparsitySystem system;
  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
}

// SymbolicSparsitySystem has the same sparsity matrix as ManualSparsitySystem,
// but the matrix can be inferred from the symbolic form.
template <typename T>
class SymbolicSparsitySystem : public LeafSystem<T> {
 public:
  SymbolicSparsitySystem() {
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    auto& y0 = *(output->GetMutableVectorData(0));
    auto& y1 = *(output->GetMutableVectorData(1));

    y0.set_value(u1.get_value());
    y1.set_value(u0.get_value());
  }

 protected:
  SymbolicSparsitySystem<symbolic::Expression>* DoToSymbolic() const override {
    return new SymbolicSparsitySystem<symbolic::Expression>();
  }

  const int kSize = 1;
};

GTEST_TEST(FeedthroughTest, SymbolicSparsity) {
  SymbolicSparsitySystem<double> system;
  // Both the output ports have direct feedthrough from some input.
  EXPECT_TRUE(system.HasAnyDirectFeedthrough());
  EXPECT_TRUE(system.HasDirectFeedthrough(0));
  EXPECT_TRUE(system.HasDirectFeedthrough(1));
  // Check the entire matrix.
  EXPECT_FALSE(system.HasDirectFeedthrough(0, 0));
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 1));
  EXPECT_TRUE(system.HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(system.HasDirectFeedthrough(1, 1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
