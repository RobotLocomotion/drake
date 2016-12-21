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
  }
  ~TestSystem() override {}

  void AddPeriodicUpdate() {
    const double period = 10.0;
    const double offset = 5.0;
    this->DeclarePeriodicUpdate(period, offset);
  }

  void AddPeriodicUpdate(double period) {
    const double offset = 0.0;
    this->DeclarePeriodicUpdate(period, offset);
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

class SizeNineBasicVector : public BasicVector<double> {
 public:
  SizeNineBasicVector() : BasicVector<double>(4 + 3 + 2) {}
  ~SizeNineBasicVector() override {}

 protected:
  SizeNineBasicVector* DoClone() const override {
    auto clone = new SizeNineBasicVector;
    clone->set_value(this->get_value());
    return clone;
  }
};

// Tests that the leaf system reserved the declared continuous state, of
// interesting custom type.
TEST_F(LeafSystemTest, DeclareTypedContinuousState) {
  system_.AddContinuousState(std::make_unique<SizeNineBasicVector>());
  std::unique_ptr<Context<double>> context = system_.CreateDefaultContext();
  const ContinuousState<double>* xc = context->get_continuous_state();
  // Check that type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<SizeNineBasicVector*>(
                         context->get_mutable_continuous_state_vector()));
  // Check that dimensions were preserved.
  EXPECT_EQ(4 + 3 + 2, xc->size());
  EXPECT_EQ(4, xc->get_generalized_position().size());
  EXPECT_EQ(3, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());
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

}  // namespace
}  // namespace systems
}  // namespace drake
