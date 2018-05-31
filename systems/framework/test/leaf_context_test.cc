#include "drake/systems/framework/leaf_context.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {

constexpr int kNumInputPorts = 2;
constexpr int kInputSize[kNumInputPorts] = {1, 2};
constexpr int kContinuousStateSize = 5;
constexpr int kGeneralizedPositionSize = 2;
constexpr int kGeneralizedVelocitySize = 2;
constexpr int kMiscContinuousStateSize = 1;

constexpr double kTime = 12.0;

// Defines a simple class for evaluating abstract types.
class TestAbstractType {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestAbstractType)
  TestAbstractType() = default;
};

class LeafContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_.set_time(kTime);

    // Input
    SetNumInputPorts(kNumInputPorts, &context_);

    // Fixed input values get new tickets -- manually update the ticket
    // counter here so this test can add more ticketed things later.
    // (That's not allowed in user code.)
    for (int i = 0; i < kNumInputPorts; ++i) {
      context_.FixInputPort(i, BasicVector<double>(kInputSize[i]));
      ++next_ticket_;
    }

    // Reserve a continuous state with five elements.
    context_.set_continuous_state(std::make_unique<ContinuousState<double>>(
        BasicVector<double>::Make({1.0, 2.0, 3.0, 5.0, 8.0}),
        kGeneralizedPositionSize, kGeneralizedVelocitySize,
        kMiscContinuousStateSize));

    // Reserve a discrete state with a single element of size 1, and verify
    // that we can change it using get_mutable_discrete_state_vector().
    std::vector<std::unique_ptr<BasicVector<double>>> xd_single;
    xd_single.push_back(BasicVector<double>::Make({128.0}));
    context_.set_discrete_state(
        std::make_unique<DiscreteValues<double>>(std::move(xd_single)));
    context_.get_mutable_discrete_state_vector()[0] = 192.0;
    EXPECT_EQ(context_.get_discrete_state().get_vector()[0], 192.0);

    // Reserve a discrete state with two elements, of size 1 and size 2.
    std::vector<std::unique_ptr<BasicVector<double>>> xd;
    xd.push_back(BasicVector<double>::Make({128.0}));
    xd.push_back(BasicVector<double>::Make({256.0, 512.0}));
    context_.set_discrete_state(
        std::make_unique<DiscreteValues<double>>(std::move(xd)));

    // Reserve an abstract state with one element, which is not owned.
    abstract_state_ = PackValue(42);
    std::vector<AbstractValue*> xa;
    xa.push_back(abstract_state_.get());
    context_.set_abstract_state(
        std::make_unique<AbstractValues>(std::move(xa)));

    // Reserve two numeric parameters of size 3 and size 4, and one abstract
    // valued parameter of type TestAbstractType.
    std::vector<std::unique_ptr<BasicVector<double>>> vector_params;
    vector_params.push_back(BasicVector<double>::Make({1.0, 2.0, 4.0}));
    vector_params.push_back(BasicVector<double>::Make({8.0, 16.0, 32.0, 64.0}));
    std::vector<std::unique_ptr<AbstractValue>> abstract_params;
    abstract_params.push_back(std::make_unique<Value<TestAbstractType>>());
    context_.set_parameters(std::make_unique<Parameters<double>>(
        std::move(vector_params),
        std::move(abstract_params)));
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  // Returns nullptr if the port is not connected.
  const BasicVector<double>* ReadVectorInputPort(const Context<double>& context,
                                                 int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_vector_value<double>() : nullptr;
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  const std::string* ReadStringInputPort(const Context<double>& context,
                                         int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_value().GetValue<std::string>()
                      : nullptr;
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  const AbstractValue* ReadAbstractInputPort(const Context<double>& context,
                                             int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_value() : nullptr;
  }

  // Mocks up some input ports sufficient to allow us to give them fixed values.
  // This code mimics SystemBase::CreateSourceTrackers.
  template <typename T>
  void SetNumInputPorts(int n, Context<T>* context) {
    for (InputPortIndex i(0); i < n; ++i) {
      context->AddInputPort(i, next_ticket_++);
    }
  }

  DependencyTicket next_ticket_{internal::kNextAvailableTicket};
  LeafContext<double> context_;
  std::unique_ptr<AbstractValue> abstract_state_;
};

// Verifies that @p state is a clone of the state constructed in
// LeafContextTest::SetUp.
void VerifyClonedState(const State<double>& clone) {
  // Verify that the state was copied.
  const ContinuousState<double>& xc = clone.get_continuous_state();
  {
    VectorX<double> contents = xc.CopyToVector();
    VectorX<double> expected(kContinuousStateSize);
    expected << 1.0, 2.0, 3.0, 5.0, 8.0;
    EXPECT_EQ(expected, contents);
  }

  EXPECT_EQ(2, clone.get_discrete_state().num_groups());
  const BasicVector<double>& xd0 = clone.get_discrete_state().get_vector(0);
  const BasicVector<double>& xd1 = clone.get_discrete_state().get_vector(1);

  // Check that sugar methods work too.
  EXPECT_EQ(&clone.get_discrete_state(0), &xd0);
  EXPECT_EQ(&clone.get_discrete_state(1), &xd1);

  {
    VectorX<double> contents = xd0.CopyToVector();
    VectorX<double> expected(1);
    expected << 128.0;
    EXPECT_EQ(expected, contents);
  }

  {
    VectorX<double> contents = xd1.CopyToVector();
    VectorX<double> expected(2);
    expected << 256.0, 512.0;
    EXPECT_EQ(expected, contents);
  }

  EXPECT_EQ(1, clone.get_abstract_state().size());
  EXPECT_EQ(42, clone.get_abstract_state().get_value(0).GetValue<int>());
  EXPECT_EQ(42, clone.get_abstract_state<int>(0));

  // Verify that the state type was preserved.
  const BasicVector<double>* xc_data =
      dynamic_cast<const BasicVector<double>*>(&xc.get_vector());
  ASSERT_NE(nullptr, xc_data);
  EXPECT_EQ(kContinuousStateSize, xc_data->size());

  // Verify that the second-order structure was preserved.
  EXPECT_EQ(kGeneralizedPositionSize, xc.get_generalized_position().size());
  EXPECT_EQ(1.0, xc.get_generalized_position().GetAtIndex(0));
  EXPECT_EQ(2.0, xc.get_generalized_position().GetAtIndex(1));

  EXPECT_EQ(kGeneralizedVelocitySize, xc.get_generalized_velocity().size());
  EXPECT_EQ(3.0, xc.get_generalized_velocity().GetAtIndex(0));
  EXPECT_EQ(5.0, xc.get_generalized_velocity().GetAtIndex(1));

  EXPECT_EQ(kMiscContinuousStateSize, xc.get_misc_continuous_state().size());
  EXPECT_EQ(8.0, xc.get_misc_continuous_state().GetAtIndex(0));
}

TEST_F(LeafContextTest, GetNumInputPorts) {
  ASSERT_EQ(kNumInputPorts, context_.get_num_input_ports());
}

TEST_F(LeafContextTest, GetNumDiscreteStateGroups) {
  EXPECT_EQ(2, context_.get_num_discrete_state_groups());
}

TEST_F(LeafContextTest, GetNumAbstractStates) {
  EXPECT_EQ(1, context_.get_num_abstract_states());
}

TEST_F(LeafContextTest, IsStateless) {
  EXPECT_FALSE(context_.is_stateless());
  LeafContext<double> empty_context;
  EXPECT_TRUE(empty_context.is_stateless());
}

TEST_F(LeafContextTest, HasOnlyContinuousState) {
  EXPECT_FALSE(context_.has_only_continuous_state());
  context_.set_discrete_state(std::make_unique<DiscreteValues<double>>());
  context_.set_abstract_state(std::make_unique<AbstractValues>());
  EXPECT_TRUE(context_.has_only_continuous_state());
}

TEST_F(LeafContextTest, HasOnlyDiscreteState) {
  EXPECT_FALSE(context_.has_only_discrete_state());
  context_.set_continuous_state(std::make_unique<ContinuousState<double>>());
  context_.set_abstract_state(std::make_unique<AbstractValues>());
  EXPECT_TRUE(context_.has_only_discrete_state());
}

TEST_F(LeafContextTest, GetNumStates) {
  LeafContext<double> context;
  EXPECT_EQ(context.get_num_total_states(), 0);

  // Reserve a continuous state with five elements.
  context.set_continuous_state(std::make_unique<ContinuousState<double>>(
      BasicVector<double>::Make({1.0, 2.0, 3.0, 5.0, 8.0})));
  EXPECT_EQ(context.get_num_total_states(), 5);

  // Reserve a discrete state with two elements, of size 1 and size 2.
  std::vector<std::unique_ptr<BasicVector<double>>> xd;
  xd.push_back(BasicVector<double>::Make({128.0}));
  xd.push_back(BasicVector<double>::Make({256.0, 512.0}));
  context.set_discrete_state(
      std::make_unique<DiscreteValues<double>>(std::move(xd)));
  EXPECT_EQ(context.get_num_total_states(), 8);

  // Reserve an abstract state with one element, which is not owned.
  std::unique_ptr<AbstractValue> abstract_state = PackValue(42);
  std::vector<AbstractValue*> xa;
  xa.push_back(abstract_state.get());
  context.set_abstract_state(std::make_unique<AbstractValues>(std::move(xa)));
  EXPECT_THROW(context.get_num_total_states(), std::runtime_error);
}

TEST_F(LeafContextTest, GetVectorInput) {
  LeafContext<double> context;
  SetNumInputPorts(2, &context);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  context.FixInputPort(0, {5.0, 6.0});

  // Test that port 0 is retrievable.
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, ReadVectorInputPort(context, 0)->get_value());

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadVectorInputPort(context, 1));
}

TEST_F(LeafContextTest, GetAbstractInput) {
  LeafContext<double> context;
  SetNumInputPorts(2, &context);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  context.FixInputPort(0, Value<std::string>("foo"));

  // Test that port 0 is retrievable.
  EXPECT_EQ("foo", *ReadStringInputPort(context, 0));

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadAbstractInputPort(context, 1));
}

TEST_F(LeafContextTest, FixInputPort) {
  const InputPortIndex index{0};
  const int size = kInputSize[index];

  // Test the unique_ptr<BasicVector> overload.
  {
    FixedInputPortValue& value = context_.FixInputPort(
        index, VectorXd::Constant(size, 3.0));
    EXPECT_EQ(context_.MaybeGetFixedInputPortValue(index), &value);
    EXPECT_EQ(value.get_vector_value<double>()[0], 3.0);
  }

  // Test the const BasicVector& overload.
  {
    FixedInputPortValue& value = context_.FixInputPort(
        index, BasicVector<double>(VectorXd::Constant(size, 2.0)));
    EXPECT_EQ(context_.MaybeGetFixedInputPortValue(index), &value);
    EXPECT_EQ(value.get_vector_value<double>()[0], 2.0);
  }

  // Test the Eigen::Ref overload.
  {
    FixedInputPortValue& value = context_.FixInputPort(
        index, VectorXd::Constant(size, 1.0));
    EXPECT_EQ(context_.MaybeGetFixedInputPortValue(index), &value);
    EXPECT_EQ(value.get_vector_value<double>()[0], 1.0);
  }

  // N.B. The GetAbstractInput test case above already covers the FixInputPort
  // overloads for AbstractValue.
}

TEST_F(LeafContextTest, Clone) {
  std::unique_ptr<Context<double>> clone = context_.Clone();
  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(kNumInputPorts, clone->get_num_input_ports());
  for (int i = 0; i < kNumInputPorts; ++i) {
    const BasicVector<double>* context_port = ReadVectorInputPort(context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(context_port, clone_port);
    EXPECT_TRUE(CompareMatrices(context_port->get_value(),
                                clone_port->get_value(), 1e-8,
                                MatrixCompareType::absolute));
  }

  // Verify that the state was copied.
  VerifyClonedState(clone->get_state());

  // Verify that changes to the cloned state do not affect the original state.
  // -- Continuous
  ContinuousState<double>& xc = clone->get_mutable_continuous_state();
  xc.get_mutable_generalized_velocity().SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, xc[3]);
  EXPECT_EQ(5.0, context_.get_continuous_state_vector().GetAtIndex(3));

  // -- Discrete
  BasicVector<double>& xd1 = clone->get_mutable_discrete_state(1);
  xd1.SetAtIndex(0, 1024.0);
  EXPECT_EQ(1024.0, clone->get_discrete_state(1).GetAtIndex(0));
  EXPECT_EQ(256.0, context_.get_discrete_state(1).GetAtIndex(0));

  // Check State indexed discrete methods too.
  State<double>& state = clone->get_mutable_state();
  EXPECT_EQ(1024.0, state.get_discrete_state(1).GetAtIndex(0));
  EXPECT_EQ(1024.0, state.get_mutable_discrete_state(1).GetAtIndex(0));

  // -- Abstract (even though it's not owned in context_)
  clone->get_mutable_abstract_state<int>(0) = 2048;
  EXPECT_EQ(42, context_.get_abstract_state<int>(0));
  EXPECT_EQ(42, context_.get_abstract_state().get_value(0).GetValue<int>());
  EXPECT_EQ(2048, clone->get_abstract_state<int>(0));

  // Verify that the parameters were copied.
  LeafContext<double>* leaf_clone =
      dynamic_cast<LeafContext<double>*>(clone.get());
  EXPECT_EQ(2, leaf_clone->num_numeric_parameters());
  const BasicVector<double>& param0 = leaf_clone->get_numeric_parameter(0);
  EXPECT_EQ(1.0, param0[0]);
  EXPECT_EQ(2.0, param0[1]);
  EXPECT_EQ(4.0, param0[2]);
  const BasicVector<double>& param1 = leaf_clone->get_numeric_parameter(1);
  EXPECT_EQ(8.0, param1[0]);
  EXPECT_EQ(16.0, param1[1]);
  EXPECT_EQ(32.0, param1[2]);
  EXPECT_EQ(64.0, param1[3]);
  ASSERT_EQ(1, leaf_clone->num_abstract_parameters());
  EXPECT_TRUE(is_dynamic_castable<const TestAbstractType>(
      &leaf_clone->get_abstract_parameter(0).GetValue<TestAbstractType>()));

  // Verify that changes to the cloned parameters do not affect the originals.
  leaf_clone->get_mutable_numeric_parameter(0)[0] = 76.0;
  EXPECT_EQ(1.0, context_.get_numeric_parameter(0).GetAtIndex(0));
}

// Tests that a LeafContext can provide a clone of its State.
TEST_F(LeafContextTest, CloneState) {
  std::unique_ptr<State<double>> clone = context_.CloneState();
  VerifyClonedState(*clone);
}

// Tests that the State can be copied from another State.
TEST_F(LeafContextTest, CopyStateFrom) {
  std::unique_ptr<Context<double>> clone = context_.Clone();
  clone->get_mutable_continuous_state()[0] = 81.0;
  clone->get_mutable_discrete_state(0)[0] = 243.0;
  clone->get_mutable_abstract_state<int>(0) = 729;

  context_.get_mutable_state().CopyFrom(clone->get_state());

  EXPECT_EQ(81.0, context_.get_continuous_state()[0]);
  EXPECT_EQ(243.0, context_.get_discrete_state(0)[0]);
  EXPECT_EQ(729, context_.get_abstract_state<int>(0));
}

// Tests that a LeafContext<AutoDiffXd> can be initialized from a
// LeafContext<double>.
TEST_F(LeafContextTest, SetTimeStateAndParametersFrom) {
  // Set up a target with the same geometry as the source, and no
  // interesting values.
  // In actual applications, System<T>::CreateDefaultContext does this.
  LeafContext<AutoDiffXd> target;
  target.set_continuous_state(std::make_unique<ContinuousState<AutoDiffXd>>(
      std::make_unique<BasicVector<AutoDiffXd>>(5),
      kGeneralizedPositionSize, kGeneralizedVelocitySize,
      kMiscContinuousStateSize));

  std::vector<std::unique_ptr<BasicVector<AutoDiffXd>>> xd;
  xd.push_back(std::make_unique<BasicVector<AutoDiffXd>>(1));
  xd.push_back(std::make_unique<BasicVector<AutoDiffXd>>(2));
  target.set_discrete_state(
      std::make_unique<DiscreteValues<AutoDiffXd>>(std::move(xd)));

  std::vector<std::unique_ptr<AbstractValue>> xa;
  xa.push_back(PackValue(76));
  target.set_abstract_state(std::make_unique<AbstractValues>(std::move(xa)));

  std::vector<std::unique_ptr<BasicVector<AutoDiffXd>>> params;
  params.push_back(std::make_unique<BasicVector<AutoDiffXd>>(3));
  params.push_back(std::make_unique<BasicVector<AutoDiffXd>>(4));
  target.get_mutable_parameters().set_numeric_parameters(
      std::make_unique<DiscreteValues<AutoDiffXd>>(std::move(params)));

  std::vector<std::unique_ptr<AbstractValue>> abstract_params;
  abstract_params.push_back(std::make_unique<Value<TestAbstractType>>());
  target.get_mutable_parameters().set_abstract_parameters(
      std::make_unique<AbstractValues>(std::move(abstract_params)));

  // Set the accuracy in the target- setting time, state, and parameters
  // should reset it.
  const double accuracy = 0.1;
  target.set_accuracy(accuracy);

  // Set the target from the source.
  target.SetTimeStateAndParametersFrom(context_);

  // Verify that accuracy is no longer set.
  EXPECT_FALSE(target.get_accuracy());

  // Verify that time was set.
  EXPECT_EQ(kTime, target.get_time());
  // Verify that state was set.
  const ContinuousState<AutoDiffXd>& xc = target.get_continuous_state();
  EXPECT_EQ(kGeneralizedPositionSize, xc.get_generalized_position().size());
  EXPECT_EQ(5.0, xc.get_generalized_velocity()[1].value());
  EXPECT_EQ(0, xc.get_generalized_velocity()[1].derivatives().size());
  EXPECT_EQ(128.0, target.get_discrete_state(0).GetAtIndex(0));
  // Verify that parameters were set.
  target.get_numeric_parameter(0);
  EXPECT_EQ(2.0, (target.get_numeric_parameter(0).GetAtIndex(1).value()));

  // Set the accuracy in the context.
  context_.set_accuracy(accuracy);
  target.SetTimeStateAndParametersFrom(context_);
  EXPECT_EQ(target.get_accuracy(), accuracy);
}

// Verifies that accuracy is set properly.
TEST_F(LeafContextTest, Accuracy) {
  // Verify accuracy is not set by default.
  EXPECT_FALSE(context_.get_accuracy());

  // Verify that setting the accuracy is reflected in cloning.
  const double unity = 1.0;
  context_.set_accuracy(unity);
  std::unique_ptr<Context<double>> clone = context_.Clone();
  EXPECT_EQ(clone->get_accuracy().value(), unity);
}

}  // namespace systems
}  // namespace drake
