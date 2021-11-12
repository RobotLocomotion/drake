#include "drake/systems/framework/leaf_context.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {

constexpr int kNumInputPorts = 2;
constexpr int kInputSize[kNumInputPorts] = {1, 2};
constexpr int kNumOutputPorts = 3;
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
    context_.SetTime(kTime);
    internal::SystemBaseContextBaseAttorney::set_system_id(
        &context_, internal::SystemId::get_new_id());

    // Set up slots for input and output ports.
    AddInputPorts(kNumInputPorts, &context_);
    AddOutputPorts(kNumOutputPorts, &context_);

    // Fixed input values get new tickets -- manually update the ticket
    // counter here so this test can add more ticketed things later.
    // (That's not allowed in user code.)
    for (int i = 0; i < kNumInputPorts; ++i) {
      context_.FixInputPort(
          i, Value<BasicVector<double>>(kInputSize[i]));
      ++next_ticket_;
    }

    // Reserve a continuous state with five elements.
    context_.init_continuous_state(std::make_unique<ContinuousState<double>>(
        BasicVector<double>::Make({1.0, 2.0, 3.0, 5.0, 8.0}),
        kGeneralizedPositionSize, kGeneralizedVelocitySize,
        kMiscContinuousStateSize));

    // Reserve a discrete state with a single element of size 1, and verify
    // that we can change it using get_mutable_discrete_state_vector().
    std::vector<std::unique_ptr<BasicVector<double>>> xd_single;
    xd_single.push_back(BasicVector<double>::Make({128.0}));
    context_.init_discrete_state(
        std::make_unique<DiscreteValues<double>>(std::move(xd_single)));
    context_.get_mutable_discrete_state_vector()[0] = 192.0;
    EXPECT_EQ(context_.get_discrete_state().get_vector()[0], 192.0);

    // Reserve a discrete state with two elements, of size 1 and size 2.
    std::vector<std::unique_ptr<BasicVector<double>>> xd;
    xd.push_back(BasicVector<double>::Make({128.0}));
    xd.push_back(BasicVector<double>::Make({256.0, 512.0}));
    context_.init_discrete_state(
        std::make_unique<DiscreteValues<double>>(std::move(xd)));

    // Add a tracker for the discrete variable xd0 and subscribe the xd
    // tracker to it.
    DependencyGraph& graph = context_.get_mutable_dependency_graph();
    auto& xd0_tracker = graph.CreateNewDependencyTracker(next_ticket_++,
                                                         "xd0");
    context_.AddDiscreteStateTicket(xd0_tracker.ticket());
    graph.get_mutable_tracker(DependencyTicket(internal::kXdTicket))
        .SubscribeToPrerequisite(&xd0_tracker);

    // Reserve an abstract state with one element, which is not owned.
    abstract_state_ = PackValue(42);
    std::vector<AbstractValue*> xa;
    xa.push_back(abstract_state_.get());
    context_.init_abstract_state(
        std::make_unique<AbstractValues>(std::move(xa)));

    // Add a tracker for the abstract variable xa0 and subscribe the xa
    // tracker to it.
    auto& xa0_tracker = graph.CreateNewDependencyTracker(next_ticket_++,
                                                         "xa0");
    context_.AddAbstractStateTicket(xa0_tracker.ticket());
    graph.get_mutable_tracker(DependencyTicket(internal::kXaTicket))
        .SubscribeToPrerequisite(&xa0_tracker);

    // Reserve two numeric parameters of size 3 and size 4.
    std::vector<std::unique_ptr<BasicVector<double>>> vector_params;
    vector_params.push_back(BasicVector<double>::Make({1.0, 2.0, 4.0}));
    vector_params.push_back(BasicVector<double>::Make({8.0, 16.0, 32.0, 64.0}));
    auto& pn0_tracker = graph.CreateNewDependencyTracker(next_ticket_++, "pn0");
    auto& pn1_tracker = graph.CreateNewDependencyTracker(next_ticket_++, "pn1");
    context_.AddNumericParameterTicket(pn0_tracker.ticket());
    context_.AddNumericParameterTicket(pn1_tracker.ticket());
    graph.get_mutable_tracker(DependencyTicket(internal::kPnTicket))
        .SubscribeToPrerequisite(&pn0_tracker);
    graph.get_mutable_tracker(DependencyTicket(internal::kPnTicket))
        .SubscribeToPrerequisite(&pn1_tracker);

    // Reserve one abstract-valued parameter of type TestAbstractType.
    std::vector<std::unique_ptr<AbstractValue>> abstract_params;
    abstract_params.push_back(std::make_unique<Value<TestAbstractType>>());
    auto& pa0_tracker = graph.CreateNewDependencyTracker(next_ticket_++, "pa0");
    context_.AddAbstractParameterTicket(pa0_tracker.ticket());
    graph.get_mutable_tracker(DependencyTicket(internal::kPaTicket))
        .SubscribeToPrerequisite(&pa0_tracker);

    context_.init_parameters(std::make_unique<Parameters<double>>(
        std::move(vector_params), std::move(abstract_params)));
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
    return free_value ? &free_value->get_value().get_value<std::string>()
                      : nullptr;
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  const AbstractValue* ReadAbstractInputPort(const Context<double>& context,
                                             int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_value() : nullptr;
  }

  // Mocks up an input port sufficient to allow us to give it a fixed value.
  template <typename T>
  void AddInputPort(InputPortIndex i, LeafContext<T>* context,
                    std::function<void(const AbstractValue&)> type_checker) {
    input_port_tickets_.push_back(next_ticket_);
    context->AddInputPort(i, next_ticket_++, std::move(type_checker));
  }

  // Mocks up input ports numbered [0, n).
  template <typename T>
  void AddInputPorts(int n, LeafContext<T>* context) {
    for (InputPortIndex i(0); i < n; ++i) {
      AddInputPort(i, context, {});
    }
  }

  // Mocks up some output ports sufficient to check that they are installed and
  // wired up properly. (We can't evaluate output ports without a System.)
  // This code mimics SystemBase::AllocateContext().
  template <typename T>
  void AddOutputPorts(int n, LeafContext<T>* context) {
    // Pretend the first output port has an external dependency (so tracking
    // should be deferred) while the rest are dependent on a built-in tracker.
    output_port_tickets_.push_back(next_ticket_);
    context->AddOutputPort(OutputPortIndex(0), next_ticket_++,
                           {SubsystemIndex(1), DependencyTicket(0)});

    for (OutputPortIndex i(1); i < n; ++i) {
      output_port_tickets_.push_back(next_ticket_);
      context->AddOutputPort(
          i, next_ticket_++,
          {std::nullopt, DependencyTicket(internal::kAllSourcesTicket)});
    }
  }

  DependencyTicket next_ticket_{internal::kNextAvailableTicket};
  LeafContext<double> context_;
  std::unique_ptr<AbstractValue> abstract_state_;

  // Track assigned tickets for sanity checking.
  std::vector<DependencyTicket> input_port_tickets_;
  std::vector<DependencyTicket> output_port_tickets_;
};

namespace {

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
  EXPECT_EQ(42, clone.get_abstract_state().get_value(0).get_value<int>());
  EXPECT_EQ(42, clone.get_abstract_state<int>(0));

  // Verify that the state type was preserved.
  const BasicVector<double>* xc_data =
      dynamic_cast<const BasicVector<double>*>(&xc.get_vector());
  ASSERT_NE(nullptr, xc_data);
  EXPECT_EQ(kContinuousStateSize, xc_data->size());

  // Verify that the second-order structure was preserved.
  EXPECT_EQ(kGeneralizedPositionSize, xc.get_generalized_position().size());
  EXPECT_EQ(1.0, xc.get_generalized_position()[0]);
  EXPECT_EQ(2.0, xc.get_generalized_position()[1]);

  EXPECT_EQ(kGeneralizedVelocitySize, xc.get_generalized_velocity().size());
  EXPECT_EQ(3.0, xc.get_generalized_velocity()[0]);
  EXPECT_EQ(5.0, xc.get_generalized_velocity()[1]);

  EXPECT_EQ(kMiscContinuousStateSize, xc.get_misc_continuous_state().size());
  EXPECT_EQ(8.0, xc.get_misc_continuous_state()[0]);
}

TEST_F(LeafContextTest, CheckPorts) {
  ASSERT_EQ(kNumInputPorts, context_.num_input_ports());
  ASSERT_EQ(kNumOutputPorts, context_.num_output_ports());

  // The "all inputs" tracker should have been subscribed to each of the
  // input ports. And each input port should have subscribed to its fixed
  // input value.
  auto& u_tracker =
      context_.get_tracker(DependencyTicket(internal::kAllInputPortsTicket));
  for (InputPortIndex i(0); i < kNumInputPorts; ++i) {
    EXPECT_EQ(context_.input_port_ticket(i), input_port_tickets_[i]);
    auto& tracker = context_.get_tracker(input_port_tickets_[i]);
    // The fixed input value is a prerequisite.
    EXPECT_EQ(tracker.num_prerequisites(), 1);
    EXPECT_EQ(tracker.num_subscribers(), 1);
    EXPECT_TRUE(u_tracker.HasPrerequisite(tracker));
  }

  // All output ports but port 0 should be subscribed to the "all sources"
  // tracker (just for testing -- would normally be subscribed to a cache
  // entry tracker).
  auto& all_sources_tracker =
      context_.get_tracker(DependencyTicket(internal::kAllSourcesTicket));
  for (OutputPortIndex i(0); i < kNumOutputPorts; ++i) {
    EXPECT_EQ(context_.output_port_ticket(i), output_port_tickets_[i]);
    auto& tracker = context_.get_tracker(output_port_tickets_[i]);
    EXPECT_EQ(tracker.num_subscribers(), 0);
    if (i == 0) {
      EXPECT_EQ(tracker.num_prerequisites(), 0);
    } else {
      EXPECT_EQ(tracker.num_prerequisites(), 1);
      EXPECT_TRUE(all_sources_tracker.HasSubscriber(tracker));
    }
  }
}

TEST_F(LeafContextTest, GetNumDiscreteStateGroups) {
  EXPECT_EQ(2, context_.num_discrete_state_groups());
}

TEST_F(LeafContextTest, GetNumAbstractStates) {
  EXPECT_EQ(1, context_.num_abstract_states());
}

TEST_F(LeafContextTest, IsStateless) {
  EXPECT_FALSE(context_.is_stateless());
  LeafContext<double> empty_context;
  EXPECT_TRUE(empty_context.is_stateless());
}

TEST_F(LeafContextTest, HasOnlyContinuousState) {
  EXPECT_FALSE(context_.has_only_continuous_state());
  context_.init_discrete_state(std::make_unique<DiscreteValues<double>>());
  context_.init_abstract_state(std::make_unique<AbstractValues>());
  EXPECT_TRUE(context_.has_only_continuous_state());
}

TEST_F(LeafContextTest, HasOnlyDiscreteState) {
  EXPECT_FALSE(context_.has_only_discrete_state());
  context_.init_continuous_state(std::make_unique<ContinuousState<double>>());
  context_.init_abstract_state(std::make_unique<AbstractValues>());
  EXPECT_TRUE(context_.has_only_discrete_state());
}

TEST_F(LeafContextTest, GetNumStates) {
  LeafContext<double> context;
  EXPECT_EQ(context.num_total_states(), 0);

  // Reserve a continuous state with five elements.
  context.init_continuous_state(std::make_unique<ContinuousState<double>>(
      BasicVector<double>::Make({1.0, 2.0, 3.0, 5.0, 8.0})));
  EXPECT_EQ(context.num_total_states(), 5);

  // Reserve a discrete state with two elements, of size 1 and size 2.
  std::vector<std::unique_ptr<BasicVector<double>>> xd;
  xd.push_back(BasicVector<double>::Make({128.0}));
  xd.push_back(BasicVector<double>::Make({256.0, 512.0}));
  context.init_discrete_state(
      std::make_unique<DiscreteValues<double>>(std::move(xd)));
  EXPECT_EQ(context.num_total_states(), 8);

  // Reserve an abstract state with one element, which is not owned.
  std::unique_ptr<AbstractValue> abstract_state = PackValue(42);
  std::vector<AbstractValue*> xa;
  xa.push_back(abstract_state.get());
  context.init_abstract_state(std::make_unique<AbstractValues>(std::move(xa)));
  EXPECT_THROW(context.num_total_states(), std::runtime_error);
}

TEST_F(LeafContextTest, GetVectorInput) {
  // N.B. This test ignores the member field `context_`.
  LeafContext<double> context;
  AddInputPorts(2, &context);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  context.FixInputPort(0, Value<BasicVector<double>>(
                              Eigen::Vector2d(5.0, 6.0)));

  // Test that port 0 is retrievable.
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, ReadVectorInputPort(context, 0)->get_value());

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadVectorInputPort(context, 1));
}

TEST_F(LeafContextTest, GetAbstractInput) {
  // N.B. This test ignores the member field `context_`.
  LeafContext<double> context;
  AddInputPorts(2, &context);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  context.FixInputPort(0, Value<std::string>("foo"));

  // Test that port 0 is retrievable.
  EXPECT_EQ("foo", *ReadStringInputPort(context, 0));

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadAbstractInputPort(context, 1));
}

TEST_F(LeafContextTest, ToString) {
  const std::string str = context_.to_string();
  using ::testing::HasSubstr;
  EXPECT_THAT(str, HasSubstr("Time: 12"));
  EXPECT_THAT(str, HasSubstr("5 continuous states"));
  EXPECT_THAT(str, HasSubstr("2 discrete state groups"));
  EXPECT_THAT(str, HasSubstr("1 abstract state"));
  EXPECT_THAT(str, HasSubstr("2 numeric parameter groups"));
  EXPECT_THAT(str, HasSubstr("1 abstract parameters"));
}

// Tests that items can be stored and retrieved in the cache.
TEST_F(LeafContextTest, SetAndGetCache) {
  CacheIndex index = context_.get_mutable_cache()
                         .CreateNewCacheEntryValue(
                             CacheIndex(0), ++next_ticket_, "entry",
                             {DependencyTicket(internal::kNothingTicket)},
                             &context_.get_mutable_dependency_graph())
                         .cache_index();
  CacheEntryValue& entry_value =
      context_.get_mutable_cache().get_mutable_cache_entry_value(index);
  entry_value.SetInitialValue(PackValue(42));
  EXPECT_EQ(entry_value.cache_index(), index);
  EXPECT_TRUE(entry_value.ticket().is_valid());
  EXPECT_EQ(entry_value.description(), "entry");

  EXPECT_TRUE(entry_value.is_out_of_date());  // Initial value isn't up to date.
  EXPECT_THROW(entry_value.GetValueOrThrow<int>(), std::logic_error);
  entry_value.mark_up_to_date();
  DRAKE_EXPECT_NO_THROW(entry_value.GetValueOrThrow<int>());

  const AbstractValue& value = entry_value.GetAbstractValueOrThrow();
  EXPECT_EQ(42, UnpackIntValue(value));
  EXPECT_EQ(42, entry_value.GetValueOrThrow<int>());
  EXPECT_EQ(42, entry_value.get_value<int>());

  // Already up to date.
  EXPECT_THROW(entry_value.SetValueOrThrow<int>(43), std::logic_error);
  entry_value.mark_out_of_date();

  DRAKE_EXPECT_NO_THROW(entry_value.SetValueOrThrow<int>(43));
  EXPECT_FALSE(entry_value.is_out_of_date());  // Set marked it up to date.
  EXPECT_EQ(43, UnpackIntValue(entry_value.GetAbstractValueOrThrow()));

  entry_value.mark_out_of_date();
  entry_value.set_value<int>(99);
  EXPECT_FALSE(entry_value.is_out_of_date());  // Set marked it up to date.
  EXPECT_EQ(99, entry_value.get_value<int>());
}

TEST_F(LeafContextTest, Clone) {
  std::unique_ptr<Context<double>> clone = context_.Clone();
  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the system id was copied.
  EXPECT_TRUE(context_.get_system_id().is_valid());
  EXPECT_EQ(context_.get_system_id(), clone->get_system_id());
  ContinuousState<double>& xc = clone->get_mutable_continuous_state();
  EXPECT_TRUE(xc.get_system_id().is_valid());
  EXPECT_EQ(xc.get_system_id(), context_.get_system_id());
  const DiscreteValues<double>& xd = clone->get_discrete_state();
  EXPECT_TRUE(xd.get_system_id().is_valid());
  EXPECT_EQ(xd.get_system_id(), context_.get_system_id());

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(kNumInputPorts, clone->num_input_ports());
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
  xc.get_mutable_generalized_velocity()[1] = 42.0;
  EXPECT_EQ(42.0, xc[3]);
  EXPECT_EQ(5.0, context_.get_continuous_state_vector()[3]);

  // -- Discrete
  BasicVector<double>& xd1 = clone->get_mutable_discrete_state(1);
  xd1[0] = 1024.0;
  EXPECT_EQ(1024.0, clone->get_discrete_state(1)[0]);
  EXPECT_EQ(256.0, context_.get_discrete_state(1)[0]);

  // Check State indexed discrete methods too.
  State<double>& state = clone->get_mutable_state();
  EXPECT_EQ(1024.0, state.get_discrete_state(1)[0]);
  EXPECT_EQ(1024.0, state.get_mutable_discrete_state(1)[0]);

  // -- Abstract (even though it's not owned in context_)
  clone->get_mutable_abstract_state<int>(0) = 2048;
  EXPECT_EQ(42, context_.get_abstract_state<int>(0));
  EXPECT_EQ(42, context_.get_abstract_state().get_value(0).get_value<int>());
  EXPECT_EQ(2048, clone->get_abstract_state<int>(0));

  // Verify that the parameters were copied.
  LeafContext<double>* leaf_clone =
      dynamic_cast<LeafContext<double>*>(clone.get());
  EXPECT_EQ(2, leaf_clone->num_numeric_parameter_groups());
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
      &leaf_clone->get_abstract_parameter(0).get_value<TestAbstractType>()));

  // Verify that changes to the cloned parameters do not affect the originals.
  leaf_clone->get_mutable_numeric_parameter(0)[0] = 76.0;
  EXPECT_EQ(1.0, context_.get_numeric_parameter(0)[0]);
}

// Violates the Context `DoCloneWithoutPointers` law.
class InvalidContext : public LeafContext<double> {
 public:
  InvalidContext() : LeafContext<double>() {}
};

TEST_F(LeafContextTest, BadClone) {
  InvalidContext bad_context;
  DRAKE_EXPECT_THROWS_MESSAGE(
      bad_context.Clone(), std::runtime_error,
      ".*typeid.source. == typeid.clone.*");
}

// Tests that a LeafContext can provide a clone of its State.
TEST_F(LeafContextTest, CloneState) {
  std::unique_ptr<State<double>> clone = context_.CloneState();
  VerifyClonedState(*clone);

  // Verify that the system id was copied.
  EXPECT_TRUE(clone->get_system_id().is_valid());
  EXPECT_EQ(clone->get_system_id(), context_.get_system_id());
  const ContinuousState<double>& xc = clone->get_continuous_state();
  EXPECT_TRUE(xc.get_system_id().is_valid());
  EXPECT_EQ(xc.get_system_id(), context_.get_system_id());
  const DiscreteValues<double>& xd = clone->get_discrete_state();
  EXPECT_TRUE(xd.get_system_id().is_valid());
  EXPECT_EQ(xd.get_system_id(), context_.get_system_id());
}

// Tests that the State can be copied from another State.
TEST_F(LeafContextTest, CopyStateFrom) {
  std::unique_ptr<Context<double>> clone = context_.Clone();
  clone->get_mutable_continuous_state()[0] = 81.0;
  clone->get_mutable_discrete_state(0)[0] = 243.0;
  clone->get_mutable_abstract_state<int>(0) = 729;

  context_.get_mutable_state().SetFrom(clone->get_state());

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
  target.init_continuous_state(std::make_unique<ContinuousState<AutoDiffXd>>(
      std::make_unique<BasicVector<AutoDiffXd>>(5),
      kGeneralizedPositionSize, kGeneralizedVelocitySize,
      kMiscContinuousStateSize));

  std::vector<std::unique_ptr<BasicVector<AutoDiffXd>>> xd;
  xd.push_back(std::make_unique<BasicVector<AutoDiffXd>>(1));
  xd.push_back(std::make_unique<BasicVector<AutoDiffXd>>(2));
  target.init_discrete_state(
      std::make_unique<DiscreteValues<AutoDiffXd>>(std::move(xd)));

  std::vector<std::unique_ptr<AbstractValue>> xa;
  xa.push_back(PackValue(76));
  target.init_abstract_state(std::make_unique<AbstractValues>(std::move(xa)));

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
  target.SetAccuracy(accuracy);

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
  EXPECT_EQ(128.0, target.get_discrete_state(0)[0]);
  // Verify that parameters were set.
  target.get_numeric_parameter(0);
  EXPECT_EQ(2.0, (target.get_numeric_parameter(0)[1].value()));

  // Set the accuracy in the context.
  context_.SetAccuracy(accuracy);
  target.SetTimeStateAndParametersFrom(context_);
  EXPECT_EQ(target.get_accuracy(), accuracy);
}

// Verifies that accuracy is set properly.
TEST_F(LeafContextTest, Accuracy) {
  // Verify accuracy is not set by default.
  EXPECT_FALSE(context_.get_accuracy());

  // Verify that setting the accuracy is reflected in cloning.
  const double unity = 1.0;
  context_.SetAccuracy(unity);
  std::unique_ptr<Context<double>> clone = context_.Clone();
  EXPECT_EQ(clone->get_accuracy().value(), unity);
}

void MarkAllCacheValuesUpToDate(Cache* cache) {
  for (CacheIndex i(0); i < cache->cache_size(); ++i) {
    if (cache->has_cache_entry_value(i))
      cache->get_mutable_cache_entry_value(i).mark_up_to_date();
  }
}

void CheckAllCacheValuesUpToDateExcept(
    const Cache& cache, const std::set<CacheIndex>& should_be_out_of_date) {
  for (CacheIndex i(0); i < cache.cache_size(); ++i) {
    if (!cache.has_cache_entry_value(i)) continue;
    const CacheEntryValue& entry = cache.get_cache_entry_value(i);
    EXPECT_EQ(entry.is_out_of_date(), should_be_out_of_date.count(i) != 0) << i;
  }
}

// Test that changing any Context source value invalidates computations that
// are dependent on that source value. The possible sources are:
// time, accuracy, state, parameters, and input ports. In addition, state
// is partitioned into continuous, discrete, and abstract, and parameters
// are partitioned into numeric and abstract.
//
TEST_F(LeafContextTest, Invalidation) {
  // Add cache entries to the context, each dependent on one ticket and
  // record the associated CacheIndex. Start with everything valid.
  Cache& cache = context_.get_mutable_cache();
  CacheIndex index(cache.cache_size());  // Next available index.
  std::map<int, CacheIndex> depends;     // Maps ticket number to cache index.
  for (int ticket = internal::kNothingTicket;
       ticket <= internal::kLastSourceTicket; ++ticket) {
    CacheEntryValue& entry = cache.CreateNewCacheEntryValue(
        index, next_ticket_++, "entry" + std::to_string(index),
        {DependencyTicket(ticket)}, &context_.get_mutable_dependency_graph());
    depends[ticket] = index;
    entry.SetInitialValue(AbstractValue::Make<int>(int{index}));
    ++index;
  }

  // Baseline: nothing modified.
  MarkAllCacheValuesUpToDate(&cache);
  CheckAllCacheValuesUpToDateExcept(cache, {});

  // Modify time.
  MarkAllCacheValuesUpToDate(&cache);
  context_.SetTime(context_.get_time() + 1);  // Ensure this is a change.
  CheckAllCacheValuesUpToDateExcept(cache,
      {depends[internal::kTimeTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]});

  // Accuracy.
  MarkAllCacheValuesUpToDate(&cache);
  context_.SetAccuracy(7.123e-4);  // Ensure this is a change.
  CheckAllCacheValuesUpToDateExcept(cache,
      {depends[internal::kAccuracyTicket],
       depends[internal::kConfigurationTicket],
       depends[internal::kKinematicsTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]});

  // This is everything that depends on generalized positions q.
  const std::set<CacheIndex> q_dependent{
      depends[internal::kQTicket], depends[internal::kXcTicket],
      depends[internal::kXTicket], depends[internal::kConfigurationTicket],
      depends[internal::kKinematicsTicket],
      depends[internal::kAllSourcesExceptInputPortsTicket],
      depends[internal::kAllSourcesTicket]};

  // This is everything that depends on generalized velocities v and misc. z.
  const std::set<CacheIndex> vz_dependent{
      depends[internal::kVTicket], depends[internal::kZTicket],
      depends[internal::kXcTicket], depends[internal::kXTicket],
      depends[internal::kConfigurationTicket],
      depends[internal::kKinematicsTicket],
      depends[internal::kAllSourcesExceptInputPortsTicket],
      depends[internal::kAllSourcesTicket]};

  // This is everything that depends on continuous state.
  std::set<CacheIndex> xc_dependent(q_dependent);
  xc_dependent.insert(vz_dependent.cbegin(), vz_dependent.cend());

  // This is everything depends on continuous, discrete, or abstract state.
  std::set<CacheIndex> x_dependent(xc_dependent);
  x_dependent.insert(depends[internal::kXdTicket]);
  x_dependent.insert(depends[internal::kXaTicket]);

  // Modify all of state.
  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_state();
  CheckAllCacheValuesUpToDateExcept(cache, x_dependent);

  // Modify just continuous state.
  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_continuous_state();
  CheckAllCacheValuesUpToDateExcept(cache, xc_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_continuous_state_vector();
  CheckAllCacheValuesUpToDateExcept(cache, xc_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.SetContinuousState(
      context_.get_continuous_state_vector().CopyToVector());
  CheckAllCacheValuesUpToDateExcept(cache, xc_dependent);

  // Modify time and continuous state together.
  std::set<CacheIndex> t_and_xc_dependent(xc_dependent);
  t_and_xc_dependent.insert(depends[internal::kTimeTicket]);
  MarkAllCacheValuesUpToDate(&cache);
  context_.SetTimeAndContinuousState(
      context_.get_time() + 1.,
      context_.get_continuous_state_vector().CopyToVector());
  CheckAllCacheValuesUpToDateExcept(cache, t_and_xc_dependent);

  context_.SetTime(1.);
  MarkAllCacheValuesUpToDate(&cache);
  VectorBase<double>& xc1 =
      context_.SetTimeAndGetMutableContinuousStateVector(2.);
  CheckAllCacheValuesUpToDateExcept(cache, t_and_xc_dependent);
  EXPECT_EQ(context_.get_time(), 2.);
  EXPECT_EQ(&xc1, &context_.get_continuous_state_vector());

  std::set<CacheIndex> t_and_q_dependent(q_dependent);
  t_and_q_dependent.insert(depends[internal::kTimeTicket]);
  MarkAllCacheValuesUpToDate(&cache);
  VectorBase<double>& q1 = context_.SetTimeAndGetMutableQVector(3.);
  CheckAllCacheValuesUpToDateExcept(cache, t_and_q_dependent);
  EXPECT_EQ(context_.get_time(), 3.);
  EXPECT_EQ(&q1, &context_.get_continuous_state().get_generalized_position());

  MarkAllCacheValuesUpToDate(&cache);
  context_.SetTimeAndNoteContinuousStateChange(4.);
  CheckAllCacheValuesUpToDateExcept(cache, t_and_xc_dependent);
  EXPECT_EQ(context_.get_time(), 4.);

  MarkAllCacheValuesUpToDate(&cache);
  context_.NoteContinuousStateChange();
  CheckAllCacheValuesUpToDateExcept(cache, xc_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  VectorBase<double>* v1{};
  VectorBase<double>* z1{};
  std::tie(v1, z1) = context_.GetMutableVZVectors();
  CheckAllCacheValuesUpToDateExcept(cache, vz_dependent);
  EXPECT_EQ(v1, &context_.get_continuous_state().get_generalized_velocity());
  EXPECT_EQ(z1, &context_.get_continuous_state().get_misc_continuous_state());

  // Modify discrete state).
  const std::set<CacheIndex> xd_dependent
      {depends[internal::kXdTicket],
       depends[internal::kXTicket],
       depends[internal::kConfigurationTicket],
       depends[internal::kKinematicsTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]};
  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_discrete_state();
  CheckAllCacheValuesUpToDateExcept(cache, xd_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_discrete_state(DiscreteStateIndex(0));
  CheckAllCacheValuesUpToDateExcept(cache, xd_dependent);

  const Vector1d xd0_val(2.);
  const Eigen::Vector2d xd1_val(3., 4.);

  // SetDiscreteState(group) should be more discerning but currently invalidates
  // dependents of all groups when only one changes.
  MarkAllCacheValuesUpToDate(&cache);
  context_.SetDiscreteState(DiscreteStateIndex(0), xd0_val);
  CheckAllCacheValuesUpToDateExcept(cache, xd_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.SetDiscreteState(DiscreteStateIndex(1), xd1_val);
  CheckAllCacheValuesUpToDateExcept(cache, xd_dependent);

  // Modify abstract state.
  const std::set<CacheIndex> xa_dependent
      {depends[internal::kXaTicket],
       depends[internal::kXTicket],
       depends[internal::kConfigurationTicket],
       depends[internal::kKinematicsTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]};
  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_abstract_state();
  CheckAllCacheValuesUpToDateExcept(cache, xa_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_abstract_state<int>(AbstractStateIndex(0));
  CheckAllCacheValuesUpToDateExcept(cache, xa_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.SetAbstractState(AbstractStateIndex(0), 5);  // <int> inferred.
  CheckAllCacheValuesUpToDateExcept(cache, xa_dependent);

  // Modify parameters.
  const std::set<CacheIndex> pn_dependent
      {depends[internal::kPnTicket],
       depends[internal::kConfigurationTicket],
       depends[internal::kKinematicsTicket],
       depends[internal::kAllParametersTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]};

  const std::set<CacheIndex> pa_dependent
      {depends[internal::kPaTicket],
       depends[internal::kConfigurationTicket],
       depends[internal::kKinematicsTicket],
       depends[internal::kAllParametersTicket],
       depends[internal::kAllSourcesExceptInputPortsTicket],
       depends[internal::kAllSourcesTicket]};

  std::set<CacheIndex> p_dependent(pn_dependent);
  p_dependent.insert(depends[internal::kPaTicket]);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_numeric_parameter(NumericParameterIndex(0));
  CheckAllCacheValuesUpToDateExcept(cache, pn_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_abstract_parameter(AbstractParameterIndex(0));
  CheckAllCacheValuesUpToDateExcept(cache, pa_dependent);

  MarkAllCacheValuesUpToDate(&cache);
  context_.get_mutable_parameters();
  CheckAllCacheValuesUpToDateExcept(cache, p_dependent);

  // Modify an input port.
  FixedInputPortValue* port_value =
      context_.MaybeGetMutableFixedInputPortValue(InputPortIndex(0));
  ASSERT_NE(port_value, nullptr);
  MarkAllCacheValuesUpToDate(&cache);
  port_value->GetMutableData();
  CheckAllCacheValuesUpToDateExcept(cache,
      {depends[internal::kAllInputPortsTicket],
       depends[internal::kAllSourcesTicket]});
}

// Verify that safe Set() sugar for modifying state variables works. Cache
// invalidation is tested separately above; this just checks values.
TEST_F(LeafContextTest, TestStateSettingSugar) {
  const Vector1d xd0_init{128.}, xd0_new{1.};
  const Eigen::Vector2d xd1_init{256., 512.}, xd1_new{2., 3.};
  EXPECT_EQ(context_.get_discrete_state(0).get_value(), xd0_init);
  EXPECT_EQ(context_.get_discrete_state(1).get_value(), xd1_init);

  context_.SetDiscreteState(0, xd0_new);
  EXPECT_EQ(context_.get_discrete_state(0).get_value(), xd0_new);
  context_.SetDiscreteState(1, xd1_new);
  EXPECT_EQ(context_.get_discrete_state(1).get_value(), xd1_new);

  // With two groups the abbreviated signature isn't allowed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      context_.SetDiscreteState(xd0_new), std::logic_error,
      ".*SetDiscreteState.*: expected exactly 1.*but there were 2 groups.*");

  // Change to just one group, then it should work.
  std::vector<std::unique_ptr<BasicVector<double>>> xd;
  const Eigen::VectorXd xd_init = Eigen::Vector3d{1., 2., 3.};
  const Eigen::Vector3d xd_new{4., 5., 6.};
  xd.push_back(std::make_unique<BasicVector<double>>(xd_init));
  context_.init_discrete_state(
      std::make_unique<DiscreteValues<double>>(std::move(xd)));
  EXPECT_EQ(context_.get_discrete_state_vector().get_value(), xd_init);
  context_.SetDiscreteState(xd_new);
  EXPECT_EQ(context_.get_discrete_state_vector().get_value(), xd_new);

  EXPECT_EQ(context_.get_abstract_state<int>(0), 42);
  context_.SetAbstractState(0, 29);  // Template arg is inferred.
  EXPECT_EQ(context_.get_abstract_state<int>(0), 29);

  // Type mismatch should be caught.
  DRAKE_EXPECT_THROWS_MESSAGE(
      context_.SetAbstractState(0, std::string("hello")), std::logic_error,
      ".*cast to.*std::string.*failed.*static type.*int.*");
}

// Check that hidden internal functionality needed by Simulator::Initialize()
// and CalcNextUpdateTime() is functional in the Context.
TEST_F(LeafContextTest, PerturbTime) {
  // This is a hidden method. Should set time to perturbed_time but current
  // time to true_time.
  const double true_time = 2.;
  const double perturbed_time = true_time - 1e-14;
  ASSERT_NE(perturbed_time, true_time);  // Watch for fatal roundoff.
  context_.PerturbTime(perturbed_time, true_time);
  EXPECT_EQ(context_.get_time(), perturbed_time);
  EXPECT_EQ(*context_.get_true_time(), true_time);  // This is an std::optional.

  // Setting time the normal way clears the "true time".
  context_.SetTime(1.);
  EXPECT_FALSE(context_.get_true_time());
}

}  // namespace
}  // namespace systems
}  // namespace drake
