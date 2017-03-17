#include "drake/systems/framework/system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;
const int kNumberToPublish = 1776;
const int kNumberToUpdate = 2001;

// A shell System to test the default implementations.
class TestSystem : public System<double> {
 public:
  TestSystem() {
    this->set_name("TestSystem");
  }
  ~TestSystem() override {}

  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<Context<double>> AllocateContext() const override {
    return nullptr;
  }

  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {}

  void SetDefaults(Context<double>* context) const override {}

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    return nullptr;
  }

  const InputPortDescriptor<double>& AddAbstractInputPort() {
    return this->DeclareAbstractInputPort();
  }


  const OutputPortDescriptor<double>& AddAbstractOutputPort() {
    return this->DeclareAbstractOutputPort();
  }

  bool HasAnyDirectFeedthrough() const override {
    return true;
  }

  bool HasDirectFeedthrough(int output_port) const override {
    return true;
  }

  bool HasDirectFeedthrough(int input_port, int output_port) const override {
    return true;
  }

  int get_publish_count() const { return publish_count_; }
  int get_update_count() const { return update_count_; }
  const std::vector<int>& get_published_numbers() const {
    return published_numbers_;
  }
  const std::vector<int>& get_updated_numbers() const {
    return updated_numbers_;
  }

 protected:
  BasicVector<double>* DoAllocateInputVector(
      const InputPortDescriptor<double>& descriptor) const override {
    return nullptr;
  }

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<double>& descriptor) const override {
    return nullptr;
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {}

  // Sets up an arbitrary mapping from the current time to the next discrete
  // action, to exercise several different forms of discrete action.
  void DoCalcNextUpdateTime(const Context<double>& context,
                            UpdateActions<double>* actions) const override {
    actions->time = context.get_time() + 1;
    actions->events.emplace_back();
    DiscreteEvent<double>& event = actions->events.back();
    if (context.get_time() < 10.0) {
      // Use the default publish action.
      event.action = DiscreteEvent<double>::kPublishAction;
    } else if (context.get_time() < 20.0) {
      // Use the default update action.
      event.action = DiscreteEvent<double>::kDiscreteUpdateAction;
    } else if (context.get_time() < 30.0) {
      // Use a custom publish action.
      event.action = DiscreteEvent<double>::kPublishAction;
      event.do_publish = std::bind(&TestSystem::DoPublishNumber, this,
                                   std::placeholders::_1 /* context */);
    } else {
      // Use a custom update action.
      event.action = DiscreteEvent<double>::kDiscreteUpdateAction;
      event.do_calc_discrete_variable_update = std::bind(
                                  &TestSystem::DoCalcDiscreteUpdatesNumber,
                                  this, std::placeholders::_1 /* context */,
                                  std::placeholders::_2 /* discrete state */,
                                  kNumberToUpdate);
    }
  }

  // The default publish function.
  void DoPublish(const Context<double>& context) const override {
    ++publish_count_;
  }

  // The default update function.
  void DoCalcDiscreteVariableUpdates(
      const Context<double> &context,
      DiscreteState<double> *discrete_state) const override {
    ++update_count_;
  }

 private:
  // A custom publish function with no additional arguments.
  void DoPublishNumber(const Context<double>& context) const {
    published_numbers_.push_back(kNumberToPublish);
  }

  // A custom update function with additional argument @p num, which may be
  // bound in DoCalcNextUpdateTime.
  void DoCalcDiscreteUpdatesNumber(const Context<double> &context,
                                   DiscreteState<double> *discrete_state,
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

  system_.MapVelocityToQDot(context_, *state_vec1, &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));

  // Test Eigen specialized function specially.
  system_.MapVelocityToQDot(context_, state_vec1->CopyToVector(), &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, MapConfigurationDerivativesToVelocity) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize);

  system_.MapQDotToVelocity(context_, *state_vec1, &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));

  // Test Eigen specialized function specially.
  system_.MapQDotToVelocity(context_, state_vec1->CopyToVector(), &state_vec2);
  EXPECT_EQ(1.0, state_vec2.GetAtIndex(0));
  EXPECT_EQ(2.0, state_vec2.GetAtIndex(1));
  EXPECT_EQ(3.0, state_vec2.GetAtIndex(2));
}

TEST_F(SystemTest, ConfigurationDerivativeVelocitySizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapQDotToVelocity(context_, *state_vec1, &state_vec2),
               std::runtime_error);
}

TEST_F(SystemTest, VelocityConfigurationDerivativeSizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapVelocityToQDot(context_, *state_vec1, &state_vec2),
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

// Tests that the default DoEvalDiscreteVariableUpdates is invoked when no other
// handler is
// registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscreteUpdate) {
  context_.set_time(15.0);
  UpdateActions<double> actions;

  system_.CalcNextUpdateTime(context_, &actions);
  ASSERT_EQ(1u, actions.events.size());

  std::unique_ptr<DiscreteState<double>> update =
      system_.AllocateDiscreteVariables();
  system_.CalcDiscreteVariableUpdates(context_, actions.events[0],
                                      update.get());
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

  std::unique_ptr<DiscreteState<double>> update =
      system_.AllocateDiscreteVariables();
  system_.CalcDiscreteVariableUpdates(context_, actions.events[0],
                                      update.get());
  ASSERT_EQ(1u, system_.get_updated_numbers().size());
  EXPECT_EQ(kNumberToUpdate, system_.get_updated_numbers()[0]);
}

// Tests that descriptor references remain valid even if lots of other
// descriptors are added to the system, forcing a vector resize.
TEST_F(SystemTest, PortDescriptorsAreStable) {
  const auto& first_input = system_.AddAbstractInputPort();
  const auto& first_output = system_.AddAbstractOutputPort();
  for (int i = 0; i < 1000; i++) {
    system_.AddAbstractInputPort();
    system_.AddAbstractOutputPort();
  }
  EXPECT_EQ(1001, system_.get_num_input_ports());
  EXPECT_EQ(1001, system_.get_num_output_ports());

  // Check for address equality.
  EXPECT_EQ(&first_input, &system_.get_input_port(0));
  EXPECT_EQ(&first_output, &system_.get_output_port(0));

  // Check for valid content.
  EXPECT_EQ(kAbstractValued, first_input.get_data_type());
  EXPECT_EQ(kAbstractValued, first_output.get_data_type());
}

template <typename T>
using TestTypedVector = MyVector<1, T>;

// A shell System for AbstractValue IO test.
template <typename T>
class ValueIOTestSystem : public System<T> {
 public:
  // Has 2 input and 2 output ports.
  // The first input / output pair are abstract type, but assumed to be
  // std::string.
  // The second input / output pair are vector type with length 1.
  ValueIOTestSystem() {
    this->DeclareAbstractInputPort();
    this->DeclareAbstractOutputPort();

    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareOutputPort(kVectorValued, 1);

    this->set_name("ValueIOTestSystem");
  }

  ~ValueIOTestSystem() override {}

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    // Should only get called for the first input.
    EXPECT_EQ(descriptor.get_index(), 0);
    return AbstractValue::Make<std::string>("").release();
  }

  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    // Should only get called for the second input.
    EXPECT_EQ(descriptor.get_index(), 1);
    return new TestTypedVector<T>();
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<Context<T>> AllocateContext() const override {
    std::unique_ptr<LeafContext<T>> context(new LeafContext<T>);
    context->SetNumInputPorts(this->get_num_input_ports());
    return std::unique_ptr<Context<T>>(context.release());
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {}

  void SetDefaults(Context<T>* context) const override {}

  bool HasAnyDirectFeedthrough() const override {
    return true;
  }

  bool HasDirectFeedthrough(int output_port) const override {
    return true;
  }

  bool HasDirectFeedthrough(int input_port, int output_port) const override {
    return true;
  }

  // Append "output" to input(0), and sets output(1) = 2 * input(1).
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
    const std::string* str_in =
        this->template EvalInputValue<std::string>(context, 0);

    std::string& str_out =
        output->GetMutableData(0)->template GetMutableValue<std::string>();
    str_out = *str_in + "output";

    const BasicVector<T>* vec_in = this->EvalVectorInput(context, 1);
    BasicVector<T>* vec_out = output->GetMutableVectorData(1);

    vec_out->get_mutable_value() = 2 * vec_in->get_value();
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    std::unique_ptr<LeafSystemOutput<T>> output(
        new LeafSystemOutput<T>);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<std::string>("output")));

    output->add_port(std::make_unique<OutputPortValue>(
        std::make_unique<BasicVector<T>>(1)));

    return std::unique_ptr<SystemOutput<T>>(output.release());
  }
};

class SystemIOTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = test_sys_.CreateDefaultContext();
    output_ = test_sys_.AllocateOutput(*context_);

    // make string input
    std::unique_ptr<Value<std::string>> str_input =
        std::make_unique<Value<std::string>>("input");
    context_->SetInputPortValue(
        0, std::make_unique<FreestandingInputPortValue>(std::move(str_input)));

    // make vector input
    std::unique_ptr<BasicVector<double>> vec_input =
        std::make_unique<BasicVector<double>>(1);
    vec_input->SetAtIndex(0, 2);
    context_->SetInputPortValue(
        1, std::make_unique<FreestandingInputPortValue>(std::move(vec_input)));
  }

  ValueIOTestSystem<double> test_sys_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(SystemIOTest, SystemValueIOTest) {
  test_sys_.CalcOutput(*context_, output_.get());

  EXPECT_EQ(context_->get_num_input_ports(), 2);
  EXPECT_EQ(output_->get_num_ports(), 2);

  EXPECT_EQ(output_->get_data(0)->GetValue<std::string>(),
            std::string("inputoutput"));
  EXPECT_EQ(output_->get_vector_data(1)->get_value()(0), 4);

  // Test AllocateInput*
  // Second input is not (yet) a TestTypedVector, since I haven't called the
  // Allocate methods directly yet.
  EXPECT_EQ(dynamic_cast<const TestTypedVector<double>*>(
                test_sys_.EvalVectorInput(*context_, 1)),
            nullptr);
  // Now allocate.
  test_sys_.AllocateFreestandingInputs(context_.get());
  // First input should have been re-allocated to the empty string.
  EXPECT_EQ(test_sys_.EvalAbstractInput(*context_, 0)->GetValue<std::string>(),
            "");
  // Second input should now be of type TestTypedVector.
  EXPECT_NE(dynamic_cast<const TestTypedVector<double>*>(
                test_sys_.EvalVectorInput(*context_, 1)),
            nullptr);
}

// Tests that FixInputPortsFrom allocates ports of the same dimension as the
// source context, with the values computed by the source system, and that
// double values in vector-valued ports are explicitly converted to AutoDiffXd.
TEST_F(SystemIOTest, TransmogrifyAndFix) {
  ValueIOTestSystem<AutoDiffXd> dest_system;
  auto dest_context = dest_system.AllocateContext();
  dest_system.FixInputPortsFrom(test_sys_, *context_, dest_context.get());

  EXPECT_EQ(
      dest_system.EvalAbstractInput(*dest_context, 0)->GetValue<std::string>(),
      "input");

  const TestTypedVector<AutoDiffXd>* fixed_vec =
      dest_system.EvalVectorInput<TestTypedVector>(*dest_context, 1);
  EXPECT_NE(fixed_vec, nullptr);
  EXPECT_EQ(2, fixed_vec->GetAtIndex(0).value());
  EXPECT_EQ(0, fixed_vec->GetAtIndex(0).derivatives().size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
