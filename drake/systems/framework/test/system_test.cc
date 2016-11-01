#include "drake/systems/framework/system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;
const int kNumberToPublish = 1776;
const int kNumberToUpdate = 2001;

// A shell System to test the default implementations.
class TestSystem : public System<double> {
 public:
  TestSystem() {}
  ~TestSystem() override {}

  std::string get_name() const override { return "TestSystem"; }

  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<Context<double>> CreateDefaultContext() const override {
    return nullptr;
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    return nullptr;
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

  void EvalTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {}

  int get_publish_count() const { return publish_count_; }
  int get_update_count() const { return update_count_; }
  const std::vector<int>& get_published_numbers() const {
    return published_numbers_;
  }
  const std::vector<int>& get_updated_numbers() const {
    return updated_numbers_;
  }

 protected:
  // Sets up an arbitrary mapping from the current time to the next discrete
  // action, to exercise several different forms of discrete action.
  void DoCalcNextUpdateTime(const Context<double>& context,
                            UpdateActions<double>* actions) const override {
    actions->time = context.get_time() + 1;
    actions->events.emplace_back();
    DiscreteEvent<double>& event = actions->events.back();
    event.recipient = this;
    if (context.get_time() < 10.0) {
      // Use the default publish action.
      event.action = DiscreteEvent<double>::kPublishAction;
    } else if (context.get_time() < 20.0) {
      // Use the default update action.
      event.action = DiscreteEvent<double>::kUpdateAction;
    } else if (context.get_time() < 30.0) {
      // Use a custom publish action.
      event.action = DiscreteEvent<double>::kPublishAction;
      event.do_publish = std::bind(&TestSystem::DoPublishNumber, this,
                                   std::placeholders::_1 /* context */);
    } else {
      // Use a custom update action.
      event.action = DiscreteEvent<double>::kUpdateAction;
      event.do_update = std::bind(&TestSystem::DoEvalDifferenceUpdatesNumber,
                                  this, std::placeholders::_1 /* context */,
                                  std::placeholders::_2 /* difference state */,
                                  kNumberToUpdate);
    }
  }

  // The default publish function.
  void DoPublish(const Context<double>& context) const override {
    ++publish_count_;
  }

  // The default update function.
  void DoEvalDifferenceUpdates(
      const Context<double>& context,
      DifferenceState<double>* difference_state) const override {
    ++update_count_;
  }

 private:
  // A custom publish function with no additional arguments.
  void DoPublishNumber(const Context<double>& context) const {
    published_numbers_.push_back(kNumberToPublish);
  }

  // A custom update function with additional argument @p num, which may be
  // bound in DoCalcNextUpdateTime.
  void DoEvalDifferenceUpdatesNumber(const Context<double>& context,
                                     DifferenceState<double>* difference_state,
                                     int num) const {
    updated_numbers_.push_back(num);
  }

 private:
  mutable int publish_count_ = 0;
  mutable int update_count_ = 0;
  mutable std::vector<int> published_numbers_;
  mutable std::vector<int> updated_numbers_;
};

class SystemTest : public ::testing::Test {
 protected:
  TestSystem system_;
  LeafContext<double> context_;
};

TEST_F(SystemTest, MapVelocityToConfigurationDerivatives) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize);

  system_.MapVelocityToConfigurationDerivatives(context_, *state_vec1,
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));

  // Test Eigen specialized function specially.
  system_.MapVelocityToConfigurationDerivatives(context_,
                                                state_vec1->CopyToVector(),
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, MapConfigurationDerivativesToVelocity) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize);

  system_.MapConfigurationDerivativesToVelocity(context_, *state_vec1,
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));

  // Test Eigen specialized function specially.
  system_.MapConfigurationDerivativesToVelocity(context_,
                                                state_vec1->CopyToVector(),
                                                &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, ConfigurationDerivativeVelocitySizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapConfigurationDerivativesToVelocity(
      context_, *state_vec1, &state_vec2),
               std::runtime_error);
}

TEST_F(SystemTest, VelocityConfigurationDerivativeSizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapVelocityToConfigurationDerivatives(
                   context_, *state_vec1, &state_vec2),
               std::runtime_error);
}

// Tests that the default DoPublish is invoked when no other handler is
// registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscretePublish) {
  context_.set_time(5.0);
  UpdateActions<double> actions;

  system_.CalcNextUpdateTime(context_, &actions);
  ASSERT_EQ(1u, actions.events.size());

  system_.Publish(context_, actions.events[0]);
  EXPECT_EQ(1, system_.get_publish_count());
}

// Tests that the default DoEvalDifferenceUpdates is invoked when no other
// handler is
// registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscreteUpdate) {
  context_.set_time(15.0);
  UpdateActions<double> actions;

  system_.CalcNextUpdateTime(context_, &actions);
  ASSERT_EQ(1u, actions.events.size());

  std::unique_ptr<DifferenceState<double>> update =
      system_.AllocateDifferenceVariables();
  system_.EvalDifferenceUpdates(context_, actions.events[0], update.get());
  EXPECT_EQ(1, system_.get_update_count());
}

// Tests that custom do_publish handlers registered in DoCalcNextUpdateTime
// are invoked.
TEST_F(SystemTest, CustomDiscretePublish) {
  context_.set_time(25.0);
  UpdateActions<double> actions;

  system_.CalcNextUpdateTime(context_, &actions);
  ASSERT_EQ(1u, actions.events.size());

  system_.Publish(context_, actions.events[0]);
  ASSERT_EQ(1u, system_.get_published_numbers().size());
  EXPECT_EQ(kNumberToPublish, system_.get_published_numbers()[0]);
}

// Tests that custom do_update handlers registered in DoCalcNextUpdateTime
// are invoked.
TEST_F(SystemTest, CustomDiscreteUpdate) {
  context_.set_time(35.0);
  UpdateActions<double> actions;

  system_.CalcNextUpdateTime(context_, &actions);
  ASSERT_EQ(1u, actions.events.size());

  std::unique_ptr<DifferenceState<double>> update =
      system_.AllocateDifferenceVariables();
  system_.EvalDifferenceUpdates(context_, actions.events[0], update.get());
  ASSERT_EQ(1u, system_.get_updated_numbers().size());
  EXPECT_EQ(kNumberToUpdate, system_.get_updated_numbers()[0]);
}

// A shell System for AbstractValue IO test
class ValueIOTestSystem : public System<double> {
 public:
  // Has 2 input and 2 output ports.
  // The first input / output pair are abstractu type, but assumed to be
  // std::string.
  // The second input / output pair are vector type with length 1.
  ValueIOTestSystem() {
    DeclareAbstractInputPort(kInheritedSampling);
    DeclareAbstractOutputPort(kInheritedSampling);

    DeclareInputPort(kVectorValued, 1, kInheritedSampling);
    DeclareOutputPort(kVectorValued, 1, kInheritedSampling);
  }

  ~ValueIOTestSystem() override {}

  std::string get_name() const override { return "ValueIOTestSystem"; }

  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<Context<double>> CreateDefaultContext() const override {
    std::unique_ptr<LeafContext<double>> context(new LeafContext<double>);
    context->SetNumInputPorts(this->get_num_input_ports());
    return std::unique_ptr<Context<double>>(context.release());
  }

  // Eval append "output" to input(0), and sets output(1) = 2 * input(1).
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    const std::string* str_in = EvalInputValue<std::string>(context, 0);

    std::string& str_out =
        output->GetMutableData(0)->GetMutableValue<std::string>();
    str_out = *str_in + "output";

    const BasicVector<double>* vec_in = EvalVectorInput(context, 1);
    BasicVector<double>* vec_out = output->GetMutableVectorData(1);

    vec_out->get_mutable_value() = 2 * vec_in->get_value();
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<std::string>("output")));

    std::unique_ptr<OutputPort> vec_out_port(
        new OutputPort(std::make_unique<BasicVector<double>>(1)));
    output->get_mutable_ports()->push_back(std::move(vec_out_port));

    return std::unique_ptr<SystemOutput<double>>(output.release());
  }
};

GTEST_TEST(SystemIOTest, SystemValueIOTest) {
  ValueIOTestSystem test_sys;

  std::unique_ptr<Context<double>> context = test_sys.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output =
      test_sys.AllocateOutput(*context);

  // make string input
  std::unique_ptr<Value<std::string>> str_input =
      std::make_unique<Value<std::string>>("input");
  context->SetInputPort(
      0, std::make_unique<FreestandingInputPort>(std::move(str_input)));

  // make vector input
  std::unique_ptr<BasicVector<double>> vec_input =
      std::make_unique<BasicVector<double>>(1);
  vec_input->SetAtIndex(0, 2);
  context->SetInputPort(
      1, std::make_unique<FreestandingInputPort>(std::move(vec_input)));

  test_sys.EvalOutput(*context, output.get());

  EXPECT_EQ(context->get_num_input_ports(), 2);
  EXPECT_EQ(output->get_num_ports(), 2);

  EXPECT_EQ(output->get_data(0)->GetValue<std::string>(),
            std::string("inputoutput"));
  EXPECT_EQ(output->get_vector_data(1)->get_value()(0), 4);
}

}  // namespace
}  // namespace systems
}  // namespace drake
