#include "drake/systems/framework/system.h"

#include <cctype>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;

// Note that Systems in this file are derived directly from drake::System for
// testing purposes. User Systems should be derived only from LeafSystem which
// handles much of the bookkeeping you'll see here, and won't need to call
// methods that are intended only for internal use.

// A shell System to test the default implementations.
class TestSystem : public System<double> {
 public:
  TestSystem() : System<double>(SystemScalarConverter{}) {
    this->set_forced_publish_events(
        this->AllocateForcedPublishEventCollection());
    this->set_forced_discrete_update_events(
        this->AllocateForcedDiscreteUpdateEventCollection());
    this->set_forced_unrestricted_update_events(
        this->AllocateForcedUnrestrictedUpdateEventCollection());
    this->set_name("TestSystem");
  }
  ~TestSystem() override {}

  using System::AddConstraint;  // allow access to protected method.

  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const override {
    return nullptr;
  }

  std::unique_ptr<CompositeEventCollection<double>>
  AllocateCompositeEventCollection() const override {
    return std::make_unique<LeafCompositeEventCollection<double>>();
  }

  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override {}

  void SetDefaultParameters(const Context<double>& context,
                            Parameters<double>* params) const override {}

  const InputPortDescriptor<double>& AddAbstractInputPort() {
    return this->DeclareAbstractInputPort();
  }

  const LeafOutputPort<double>& AddAbstractOutputPort() {
    // Create an abstract output port with dummy alloc and calc.
    const CacheEntry& cache_entry = this->DeclareCacheEntry(
        "null output port",
        [] { return Value<int>::Make(0); },
        [](const ContextBase&, AbstractValue*) {});
    // TODO(sherm1) Use implicit_cast when available (from abseil). Several
    // places in this test.
    auto port = std::make_unique<LeafOutputPort<double>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        OutputPortIndex(this->get_num_output_ports()),
        assign_next_dependency_ticket(),
        kAbstractValued, 0, &cache_entry);
    LeafOutputPort<double>* const port_ptr = port.get();
    this->AddOutputPort(std::move(port));
    return *port_ptr;
  }

  std::multimap<int, int> GetDirectFeedthroughs() const override {
    std::multimap<int, int> pairs;
    // Report *everything* as having direct feedthrough.
    for (int i = 0; i < get_num_input_ports(); ++i) {
      for (int o = 0; o < get_num_output_ports(); ++o) {
        pairs.emplace(i, o);
      }
    }
    return pairs;
  }

  int get_publish_count() const { return publish_count_; }
  int get_update_count() const { return update_count_; }
  const std::vector<int>& get_published_numbers() const {
    return published_numbers_;
  }
  const std::vector<int>& get_updated_numbers() const {
    return updated_numbers_;
  }

  double DoCalcWitnessValue(const Context<double>&,
                            const WitnessFunction<double>&) const override {
    // This system uses no witness functions.
    DRAKE_ABORT();
  }

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<double>*,
      CompositeEventCollection<double>*) const override {
    // This system uses no witness functions.
    DRAKE_ABORT();
  }

  // The default publish function.
  void MyPublish(const Context<double>& context,
                 const std::vector<const PublishEvent<double>*>& events) const {
    ++publish_count_;
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

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {}

  void DispatchPublishHandler(
      const Context<double>& context,
      const EventCollection<PublishEvent<double>>& events) const final {
    const LeafEventCollection<PublishEvent<double>>& leaf_events =
       dynamic_cast<const LeafEventCollection<PublishEvent<double>>&>(events);
    if (leaf_events.HasEvents()) {
      this->MyPublish(context, leaf_events.get_events());
    }
  }

  void DispatchDiscreteVariableUpdateHandler(
      const Context<double>& context,
      const EventCollection<DiscreteUpdateEvent<double>>& events,
      DiscreteValues<double>* discrete_state) const final {
    const LeafEventCollection<DiscreteUpdateEvent<double>>& leaf_events =
        dynamic_cast<const LeafEventCollection<DiscreteUpdateEvent<double>>&>(
            events);
    if (leaf_events.HasEvents()) {
      this->MyCalcDiscreteVariableUpdates(context, leaf_events.get_events(),
          discrete_state);
    }
  }

  void DispatchUnrestrictedUpdateHandler(
      const Context<double>&,
      const EventCollection<UnrestrictedUpdateEvent<double>>&,
      State<double>*) const final {
    DRAKE_ABORT_MSG("test should not get here");
  }

  // Sets up an arbitrary mapping from the current time to the next discrete
  // action, to exercise several different forms of discrete action.
  void DoCalcNextUpdateTime(const Context<double>& context,
                            CompositeEventCollection<double>* event_info,
                            double* time) const override {
    *time = context.get_time() + 1;

    if (context.get_time() < 10.0) {
      PublishEvent<double> event(Event<double>::TriggerType::kPeriodic);
      event.add_to_composite(event_info);
    } else {
      DiscreteUpdateEvent<double> event(Event<double>::TriggerType::kPeriodic);
      event.add_to_composite(event_info);
    }
  }

  // The default update function.
  void MyCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>* discrete_state) const {
    ++update_count_;
  }

  std::unique_ptr<EventCollection<PublishEvent<double>>>
  AllocateForcedPublishEventCollection() const override {
    return LeafEventCollection<
        PublishEvent<double>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<double>>>
  AllocateForcedDiscreteUpdateEventCollection() const override {
    return LeafEventCollection<
        DiscreteUpdateEvent<double>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<double>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const override {
    return LeafEventCollection<
        UnrestrictedUpdateEvent<double>>::MakeForcedEventCollection();
  }

  std::map<PeriodicEventData,
      std::vector<const Event<double>*>,
      PeriodicEventDataComparator>
      DoGetPeriodicEvents() const override {
    return {};
  }

 private:
  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<LeafContext<double>>();
    InitializeContextBase(&*context);
    return context;
  }

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
  auto event_info = system_.AllocateCompositeEventCollection();
  system_.CalcNextUpdateTime(context_, event_info.get());
  const auto& events =
      dynamic_cast<const LeafCompositeEventCollection<double>*>(
          event_info.get())->get_publish_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            Event<double>::TriggerType::kPeriodic);

  system_.Publish(context_, event_info->get_publish_events());
  EXPECT_EQ(1, system_.get_publish_count());
}

// Tests that the default DoEvalDiscreteVariableUpdates is invoked when no other
// handler is
// registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscreteUpdate) {
  context_.set_time(15.0);

  auto event_info = system_.AllocateCompositeEventCollection();
  system_.CalcNextUpdateTime(context_, event_info.get());

  std::unique_ptr<DiscreteValues<double>> update =
      system_.AllocateDiscreteVariables();
  system_.CalcDiscreteVariableUpdates(
      context_, event_info->get_discrete_update_events(), update.get());
  EXPECT_EQ(1, system_.get_update_count());
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

// Tests the constraint list logic.
TEST_F(SystemTest, SystemConstraintTest) {
  EXPECT_EQ(system_.get_num_constraints(), 0);
  EXPECT_THROW(system_.get_constraint(SystemConstraintIndex(0)),
               std::out_of_range);

  SystemConstraint<double>::CalcCallback calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    unused(context);
    (*value)[0] = 1.0;
  };
  SystemConstraintIndex test_constraint =
      system_.AddConstraint(std::make_unique<SystemConstraint<double>>(
          calc, 1, SystemConstraintType::kInequality, "test"));
  EXPECT_EQ(test_constraint, 0);

  EXPECT_NO_THROW(system_.get_constraint(test_constraint));
  EXPECT_EQ(system_.get_constraint(test_constraint).description(), "test");

  const double tol = 1e-6;
  EXPECT_TRUE(system_.CheckSystemConstraintsSatisfied(context_, tol));
  SystemConstraint<double>::CalcCallback calc_false = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    unused(context);
    (*value)[0] = -1.0;
  };
  system_.AddConstraint(std::make_unique<SystemConstraint<double>>(
      calc_false, 1, SystemConstraintType::kInequality, "bad constraint"));
  EXPECT_FALSE(system_.CheckSystemConstraintsSatisfied(context_, tol));
}

// Tests GetMemoryObjectName.
TEST_F(SystemTest, GetMemoryObjectName) {
  const std::string name = system_.GetMemoryObjectName();

  // The nominal value for 'name' is something like:
  //   drake/systems/(anonymous namespace)/TestSystem@0123456789abcdef
  // We check only some platform-agnostic portions of that.
  EXPECT_THAT(name, ::testing::HasSubstr("drake/systems/"));
  EXPECT_THAT(name, ::testing::ContainsRegex("/TestSystem@[0-9a-fA-F]{16}$"));
}

// Tests that by default, transmogrification fails appropriately.
// (For testing transmogrification success, we rely on leaf_system_test.)
TEST_F(SystemTest, TransmogrifyNotSupported) {
  // Use the static method.
  EXPECT_THROW(System<double>::ToAutoDiffXd<System>(system_), std::exception);
  EXPECT_THROW(System<double>::ToSymbolic<System>(system_), std::exception);

  // Use the instance method that throws.
  EXPECT_THROW(system_.ToAutoDiffXd(), std::exception);
  EXPECT_THROW(system_.ToSymbolic(), std::exception);

  // Use the instance method that returns nullptr.
  EXPECT_EQ(system_.ToAutoDiffXdMaybe(), nullptr);
  EXPECT_EQ(system_.ToSymbolicMaybe(), nullptr);

  // Spot check the specific converter object.
  EXPECT_FALSE((
      system_.get_system_scalar_converter().IsConvertible<double, double>()));
}

template <typename T>
using TestTypedVector = MyVector<1, T>;

// A shell System for AbstractValue IO test.
template <typename T>
class ValueIOTestSystem : public System<T> {
 public:
  // Has 4 input and 2 output ports.
  // The first input / output pair are abstract type, but assumed to be
  // std::string.
  // The second input / output pair are vector type with length 1.
  // There are two other vector-valued random input ports.
  ValueIOTestSystem() : System<T>(SystemScalarConverter{}) {
    this->set_forced_publish_events(
        this->AllocateForcedPublishEventCollection());
    this->set_forced_discrete_update_events(
        this->AllocateForcedDiscreteUpdateEventCollection());
    this->set_forced_unrestricted_update_events(
        this->AllocateForcedUnrestrictedUpdateEventCollection());

    this->DeclareAbstractInputPort();
    this->AddOutputPort(std::make_unique<LeafOutputPort<T>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        OutputPortIndex(this->get_num_output_ports()),
        this->assign_next_dependency_ticket(),
        kAbstractValued, 0 /* size */,
        &this->DeclareCacheEntry(
            "absport",
            []() { return AbstractValue::Make(std::string()); },
            [this](const ContextBase& context, AbstractValue* output) {
              this->CalcStringOutput(context, output);
            })));
    this->DeclareInputPort(kVectorValued, 1);
    this->DeclareInputPort(kVectorValued, 1, RandomDistribution::kUniform);
    this->DeclareInputPort(kVectorValued, 1, RandomDistribution::kGaussian);
    this->AddOutputPort(std::make_unique<LeafOutputPort<T>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        OutputPortIndex(this->get_num_output_ports()),
        this->assign_next_dependency_ticket(),
        kVectorValued, 1 /* size */,
        &this->DeclareCacheEntry(
            "vecport",
            []() { return std::make_unique<Value<BasicVector<T>>>(1); },
            [this](const ContextBase& context, AbstractValue* output) {
              this->CalcVectorOutput(context, output);
            })));

    this->set_name("ValueIOTestSystem");
  }

  ~ValueIOTestSystem() override {}

  T DoCalcWitnessValue(const Context<T>&,
                       const WitnessFunction<T>&) const override {
    // This system uses no witness functions.
    DRAKE_ABORT();
  }

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>*,
      CompositeEventCollection<T>*) const override {
    // This system uses no witness functions.
    DRAKE_ABORT();
  }

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    // Should only get called for the first input.
    EXPECT_EQ(descriptor.get_index(), 0);
    return AbstractValue::Make<std::string>("").release();
  }

  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    // Should not get called for the first (abstract) input.
    EXPECT_GE(descriptor.get_index(), 1);
    return new TestTypedVector<T>();
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return std::make_unique<ContinuousState<T>>();
  }

  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<LeafContext<T>>();
    this->InitializeContextBase(&*context);
    return context;
  }

  std::unique_ptr<CompositeEventCollection<T>>
  AllocateCompositeEventCollection() const override {
    return std::make_unique<LeafCompositeEventCollection<T>>();
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {}

  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* params) const override {}

  std::multimap<int, int> GetDirectFeedthroughs() const override {
    std::multimap<int, int> pairs;
    // Report *everything* as having direct feedthrough.
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      for (int o = 0; o < this->get_num_output_ports(); ++o) {
        pairs.emplace(i, o);
      }
    }
    return pairs;
  }

  // Appends "output" to input(0) for output(0).
  void CalcStringOutput(const ContextBase& context,
                        AbstractValue* output) const {
    const std::string* str_in =
        this->template EvalInputValue<std::string>(context, 0);

    std::string& str_out = output->template GetMutableValue<std::string>();
    str_out = *str_in + "output";
  }

  // Sets output(1) = 2 * input(1).
  void CalcVectorOutput(const ContextBase& context_base,
                        AbstractValue* output) const {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    const BasicVector<T>* vec_in = this->EvalVectorInput(context, 1);
    auto& vec_out = output->template GetMutableValue<BasicVector<T>>();
    vec_out.get_mutable_value() = 2 * vec_in->get_value();
  }

  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& event_info) const final {
    DRAKE_ABORT_MSG("test should not get here");
  }

  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& event_info,
      DiscreteValues<T>* discrete_state) const final {
    DRAKE_ABORT_MSG("test should not get here");
  }

  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& event_info,
      State<T>* state) const final {
    DRAKE_ABORT_MSG("test should not get here");
  }

  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const override {
    return LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const override {
    return LeafEventCollection<
        DiscreteUpdateEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const override {
    return LeafEventCollection<
        UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection();
  }

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator>
  DoGetPeriodicEvents() const override {
    return {};
  }
};

// Just creates System and Context without providing values for inputs, to
// allow for lots of error conditions.
class SystemInputErrorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = system_.CreateDefaultContext();
  }

  ValueIOTestSystem<double> system_;
  std::unique_ptr<Context<double>> context_;
};

// A BasicVector-derived type we can complain about in the next test.
template <typename T>
class WrongVector : public MyVector<2, T> {
 public:
  using MyVector<2, T>::MyVector;
};

// Test error messages from the EvalInput methods.
TEST_F(SystemInputErrorTest, CheckMessages) {
  ASSERT_EQ(system_.get_num_input_ports(), 4);

  // Sanity check that this works with a good port number.
  EXPECT_NO_THROW(system_.get_input_port(1));

  // Try some illegal port numbers.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.get_input_port(-1), std::out_of_range,
      ".*get_input_port.*negative.*-1.*illegal.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput(*context_, -1), std::out_of_range,
      ".*EvalVectorInput.*negative.*-1.*illegal.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalAbstractInput(*context_, -2), std::out_of_range,
      ".*EvalAbstractInput.*negative.*-2.*illegal.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<int>(*context_, -3), std::out_of_range,
      ".*EvalInputValue.*negative.*-3.*illegal.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, -4), std::out_of_range,
      ".*EvalEigenVectorInput.*negative.*-4.*illegal.*");

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.get_input_port(9), std::out_of_range,
      ".*get_input_port.*no input port.*9.*only.*4.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput(*context_, 9), std::out_of_range,
      ".*EvalVectorInput.*no input port.*9.*only.*4.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalAbstractInput(*context_, 10), std::out_of_range,
      ".*EvalAbstractInput.*no input port.*10.*only.*4.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<int>(*context_, 11), std::out_of_range,
      ".*EvalInputValue.*no input port.*11.*only.*4.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 12), std::out_of_range,
      ".*EvalEigenVectorInput.*no input port.*12.*only.*4.*");

  // No ports have values yet. EvalEigenVectorInput() requires a value, but
  // the others should return nullptr.
  EXPECT_EQ(system_.EvalVectorInput(*context_, 1), nullptr);
  EXPECT_EQ(system_.EvalAbstractInput(*context_, 1), nullptr);
  EXPECT_EQ(system_.EvalInputValue<int>(*context_, 1), nullptr);

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 1), std::logic_error,
      ".*EvalEigenVectorInput.*input port\\[1\\].*neither connected nor "
          "fixed.*");

  // Assign values to all ports. All but port 0 are BasicVector ports.
  system_.AllocateFixedInputs(context_.get());

  EXPECT_NO_THROW(system_.EvalVectorInput(*context_, 2));  // BasicVector OK.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput<WrongVector>(*context_, 2), std::logic_error,
      ".*EvalVectorInput.*expected.*WrongVector"
          ".*input port.*2.*actual.*MyVector.*");

  EXPECT_NO_THROW(system_.EvalInputValue<BasicVector<double>>(*context_, 1));
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<int>(*context_, 1), std::logic_error,
      ".*EvalInputValue.*expected.*int.*input port.*1.*actual.*MyVector.*");

  // Now induce errors that only apply to abstract-valued input ports.

  EXPECT_EQ(*system_.EvalInputValue<std::string>(*context_, 0), "");
  EXPECT_NO_THROW(system_.EvalAbstractInput(*context_, 0));
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput(*context_, 0), std::logic_error,
      ".*EvalVectorInput.*vector port required.*input port.*0.*"
          "was declared abstract.*");

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 0),
      std::logic_error,
      ".*EvalEigenVectorInput.*vector port required.*input port.*0.*"
          "was declared abstract.*");

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<double>(*context_, 0),
      std::logic_error,
      ".*EvalInputValue.*expected.*double.*input port.*0.*actual.*string.*");
}

// Provides values for some of the inputs and sets up for outputs.
class SystemIOTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = test_sys_.CreateDefaultContext();
    output_ = test_sys_.AllocateOutput();

    // make string input
    context_->FixInputPort(0, Value<std::string>("input"));

    // make vector input
    context_->FixInputPort(1, {2.0});
  }

  ValueIOTestSystem<double> test_sys_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(SystemIOTest, SystemValueIOTest) {
  test_sys_.CalcOutput(*context_, output_.get());

  EXPECT_EQ(context_->get_num_input_ports(), 4);
  EXPECT_EQ(output_->get_num_ports(), 2);

  EXPECT_EQ(output_->get_data(0)->GetValue<std::string>(),
            std::string("inputoutput"));
  EXPECT_EQ(output_->get_vector_data(1)->get_value()(0), 4);

  // Connected inputs ports can be evaluated.  (Port #1 was set to [2]).
  const auto& block = test_sys_.EvalEigenVectorInput(*context_, 1);
  ASSERT_EQ(block.size(), 1);
  ASSERT_EQ(block[0], 2.0);

  // Disconnected inputs are nullptr, or generate an exception (not assert).
  EXPECT_EQ(test_sys_.EvalVectorInput(*context_, 2), nullptr);
  EXPECT_THROW(test_sys_.EvalEigenVectorInput(*context_, 2), std::exception);

  // Test AllocateInput*
  // Second input is not (yet) a TestTypedVector, since I haven't called the
  // Allocate methods directly yet.
  EXPECT_EQ(dynamic_cast<const TestTypedVector<double> *>(
                test_sys_.EvalVectorInput(*context_, 1)),
            nullptr);
  // Now allocate.
  test_sys_.AllocateFixedInputs(context_.get());
  // First input should have been re-allocated to the empty string.
  EXPECT_EQ(test_sys_.EvalAbstractInput(*context_, 0)->GetValue<std::string>(),
            "");
  // Second input should now be of type TestTypedVector.
  EXPECT_NE(dynamic_cast<const TestTypedVector<double> *>(
                test_sys_.EvalVectorInput(*context_, 1)),
            nullptr);
}

// Checks that the input ports randomness labels were set as expected.
TEST_F(SystemIOTest, RandomInputPortTest) {
  EXPECT_FALSE(test_sys_.get_input_port(0).is_random());
  EXPECT_FALSE(test_sys_.get_input_port(1).is_random());
  EXPECT_TRUE(test_sys_.get_input_port(2).is_random());
  EXPECT_TRUE(test_sys_.get_input_port(3).is_random());

  EXPECT_FALSE(test_sys_.get_input_port(0).get_random_type());
  EXPECT_FALSE(test_sys_.get_input_port(1).get_random_type());
  EXPECT_EQ(test_sys_.get_input_port(2).get_random_type(),
            RandomDistribution::kUniform);
  EXPECT_EQ(test_sys_.get_input_port(3).get_random_type(),
            RandomDistribution::kGaussian);
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
