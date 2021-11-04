#include "drake/systems/framework/system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/random.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 3;

// Note that Systems in this file are derived directly from drake::System for
// testing purposes. User Systems should be derived only from LeafSystem which
// handles much of the bookkeeping you'll see here, and won't need to call
// methods that are intended only for internal use. Some additional System tests
// are found in leaf_system_test.cc in order to exploit LeafSystem to satisfy
// the many pure virtuals in System.

// This class absorbs most of the boilerplate of deriving directly from
// System<T>. Implementation choices (method bodies, override vs. final) were
// made to support the needs of the derived classes and tests in this file.
template <typename T>
class TestSystemBase : public System<T> {
 public:
  TestSystemBase() : System<T>(SystemScalarConverter{}) {}

  void SetDefaultState(const Context<T>&, State<T>*) const final {}

  void SetDefaultParameters(const Context<T>&, Parameters<T>*) const final {}

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>*, CompositeEventCollection<T>*) const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const final {
    return LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const final {
    return LeafEventCollection<
        DiscreteUpdateEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const final {
    return LeafEventCollection<
        UnrestrictedUpdateEvent<T>>::MakeForcedEventCollection();
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    auto result = std::make_unique<ContinuousState<T>>();
    result->set_system_id(this->get_system_id());
    return result;
  }

  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    auto result = std::make_unique<DiscreteValues<T>>();
    result->set_system_id(this->get_system_id());
    return result;
  }

 private:
  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<LeafContext<T>>();
    this->InitializeContextBase(context.get());
    return context;
  }

  std::unique_ptr<CompositeEventCollection<T>>
  DoAllocateCompositeEventCollection() const final {
    auto result = std::make_unique<LeafCompositeEventCollection<T>>();
    result->set_system_id(this->get_system_id());
    return result;
  }

  T DoCalcWitnessValue(const Context<T>&,
                       const WitnessFunction<T>&) const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
    return {};
  }

  void DoApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  void DispatchUnrestrictedUpdateHandler(
      const Context<T>&, const EventCollection<UnrestrictedUpdateEvent<T>>&,
      State<T>*) const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  void DoApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator>
  DoGetPeriodicEvents() const final {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
    return {};
  }

  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>&) const override {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
    return {};
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {}

  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& event_info) const override {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& event_info,
      DiscreteValues<T>* discrete_state) const override {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
  }

  std::multimap<int, int> GetDirectFeedthroughs() const override {
    ADD_FAILURE() << "A test called a method that was expected to be unused.";
    return {};
  }
};

// A shell System to test the default implementations.
class TestSystem : public TestSystemBase<double> {
 public:
  TestSystem() {
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
  using System::DeclareInputPort;

  const InputPort<double>& AddAbstractInputPort() {
    return this->DeclareInputPort(kUseDefaultName, kAbstractValued, 0);
  }

  LeafOutputPort<double>& AddAbstractOutputPort() {
    // Create an abstract output port with dummy alloc and calc.
    CacheEntry& cache_entry = this->DeclareCacheEntry(
        "null output port", ValueProducer(
             0, &ValueProducer::NoopCalc));
    // TODO(sherm1) Use implicit_cast when available (from abseil). Several
    // places in this test.
    auto port = internal::FrameworkFactory::Make<LeafOutputPort<double>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        this->get_system_id(),
        "y" + std::to_string(num_output_ports()),
        OutputPortIndex(this->num_output_ports()),
        assign_next_dependency_ticket(),
        kAbstractValued, 0, &cache_entry);
    LeafOutputPort<double>* const port_ptr = port.get();
    this->AddOutputPort(std::move(port));
    return *port_ptr;
  }

  std::multimap<int, int> GetDirectFeedthroughs() const override {
    std::multimap<int, int> pairs;
    // Report *everything* as having direct feedthrough.
    for (int i = 0; i < num_input_ports(); ++i) {
      for (int o = 0; o < num_output_ports(); ++o) {
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

  // The default publish function.
  void MyPublish(const Context<double>& context,
                 const std::vector<const PublishEvent<double>*>& events) const {
    ++publish_count_;
  }

 protected:
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

  // Sets up an arbitrary mapping from the current time to the next discrete
  // action, to exercise several different forms of discrete action.
  void DoCalcNextUpdateTime(const Context<double>& context,
                            CompositeEventCollection<double>* event_info,
                            double* time) const override {
    *time = context.get_time() + 1;

    if (context.get_time() < 10.0) {
      PublishEvent<double> event(TriggerType::kPeriodic);
      event.AddToComposite(event_info);
    } else {
      DiscreteUpdateEvent<double> event(TriggerType::kPeriodic);
      event.AddToComposite(event_info);
    }
  }

  // The default update function.
  void MyCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>* discrete_state) const {
    ++update_count_;
  }

 private:
  mutable int publish_count_ = 0;
  mutable int update_count_ = 0;
  mutable std::vector<int> published_numbers_;
  mutable std::vector<int> updated_numbers_;
};

class SystemTest : public ::testing::Test {
 protected:
  void SetUp() override { context_ = system_.CreateDefaultContext(); }

  TestSystem system_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(SystemTest, ContextBelongsWithSystem) {
  TestSystem system2;

  // These just uses a couple of arbitrary methods to test that a Context not
  // created by a System throws the appropriate exception.
  DRAKE_EXPECT_THROWS_MESSAGE(system2.Publish(*context_), std::logic_error,
                              "Context was not created for.*");
  DRAKE_EXPECT_THROWS_MESSAGE(system2.SetDefaultContext(context_.get()),
                              std::logic_error,
                              "Context was not created for.*");
}

TEST_F(SystemTest, MapVelocityToConfigurationDerivatives) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize);

  system_.MapVelocityToQDot(*context_, *state_vec1, &state_vec2);
  EXPECT_EQ(1.0, state_vec2[0]);
  EXPECT_EQ(2.0, state_vec2[1]);
  EXPECT_EQ(3.0, state_vec2[2]);

  // Test Eigen specialized function specially.
  system_.MapVelocityToQDot(*context_, state_vec1->CopyToVector(), &state_vec2);
  EXPECT_EQ(1.0, state_vec2[0]);
  EXPECT_EQ(2.0, state_vec2[1]);
  EXPECT_EQ(3.0, state_vec2[2]);
}

TEST_F(SystemTest, MapConfigurationDerivativesToVelocity) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize);

  system_.MapQDotToVelocity(*context_, *state_vec1, &state_vec2);
  EXPECT_EQ(1.0, state_vec2[0]);
  EXPECT_EQ(2.0, state_vec2[1]);
  EXPECT_EQ(3.0, state_vec2[2]);

  // Test Eigen specialized function specially.
  system_.MapQDotToVelocity(*context_, state_vec1->CopyToVector(), &state_vec2);
  EXPECT_EQ(1.0, state_vec2[0]);
  EXPECT_EQ(2.0, state_vec2[1]);
  EXPECT_EQ(3.0, state_vec2[2]);
}

TEST_F(SystemTest, ConfigurationDerivativeVelocitySizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapQDotToVelocity(*context_, *state_vec1, &state_vec2),
               std::runtime_error);
}

TEST_F(SystemTest, VelocityConfigurationDerivativeSizeMismatch) {
  auto state_vec1 = BasicVector<double>::Make({1.0, 2.0, 3.0});
  BasicVector<double> state_vec2(kSize + 1);

  EXPECT_THROW(system_.MapVelocityToQDot(*context_, *state_vec1, &state_vec2),
               std::runtime_error);
}

// Tests that the default DoPublish is invoked when no other handler is
// registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscretePublish) {
  context_->SetTime(5.0);
  auto event_info = system_.AllocateCompositeEventCollection();
  system_.CalcNextUpdateTime(*context_, event_info.get());
  const auto& events =
      dynamic_cast<const LeafCompositeEventCollection<double>*>(
          event_info.get())->get_publish_events().get_events();
  EXPECT_EQ(events.size(), 1);
  EXPECT_EQ(events.front()->get_trigger_type(),
            TriggerType::kPeriodic);

  system_.Publish(*context_, event_info->get_publish_events());
  EXPECT_EQ(1, system_.get_publish_count());
}

// Tests that the default DoEvalDiscreteVariableUpdates is invoked when no other
// handler is registered in DoCalcNextUpdateTime.
TEST_F(SystemTest, DiscreteUpdate) {
  context_->SetTime(15.0);

  auto event_info = system_.AllocateCompositeEventCollection();
  system_.CalcNextUpdateTime(*context_, event_info.get());

  std::unique_ptr<DiscreteValues<double>> update =
      system_.AllocateDiscreteVariables();
  system_.CalcDiscreteVariableUpdates(
      *context_, event_info->get_discrete_update_events(), update.get());
  EXPECT_EQ(1, system_.get_update_count());
}

// Tests that port references remain valid even if lots of other ports are added
// to the system, forcing a vector resize.
TEST_F(SystemTest, PortReferencesAreStable) {
  const auto& first_input = system_.AddAbstractInputPort();
  const auto& first_output = system_.AddAbstractOutputPort();
  for (int i = 0; i < 1000; i++) {
    system_.AddAbstractInputPort();
    system_.AddAbstractOutputPort();
  }
  EXPECT_EQ(1001, system_.num_input_ports());
  EXPECT_EQ(1001, system_.num_output_ports());

  // Check for address equality.
  EXPECT_EQ(&first_input, &system_.get_input_port(0));
  EXPECT_EQ(&first_output, &system_.get_output_port(0));

  // Check for valid content.
  EXPECT_EQ(kAbstractValued, first_input.get_data_type());
  EXPECT_EQ(kAbstractValued, first_output.get_data_type());
}

// Tests the convenience methods for the case when we have exactly one input or
// output port.
TEST_F(SystemTest, ExactlyOnePortConvenience) {
  DRAKE_EXPECT_THROWS_MESSAGE(system_.get_input_port(), std::logic_error,
                              ".*num_input_ports\\(\\) = 0");

  system_.DeclareInputPort("one", kVectorValued, 2);
  EXPECT_EQ(&system_.get_input_port(), &system_.get_input_port(0));

  system_.DeclareInputPort("two", kVectorValued, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(system_.get_input_port(), std::logic_error,
                              ".*num_input_ports\\(\\) = 2");

  DRAKE_EXPECT_THROWS_MESSAGE(system_.get_output_port(), std::logic_error,
                              ".*num_output_ports\\(\\) = 0");

  system_.AddAbstractOutputPort();
  EXPECT_EQ(&system_.get_output_port(), &system_.get_output_port(0));

  system_.AddAbstractOutputPort();
  DRAKE_EXPECT_THROWS_MESSAGE(system_.get_output_port(), std::logic_error,
                              ".*num_output_ports\\(\\) = 2");
}

TEST_F(SystemTest, PortNameTest) {
  const auto& unnamed_input =
      system_.DeclareInputPort(kUseDefaultName, kVectorValued, 2);
  const auto& named_input =
      system_.DeclareInputPort("my_input", kVectorValued, 3);
  const auto& named_abstract_input =
      system_.DeclareInputPort("abstract", kAbstractValued, 0);

  EXPECT_EQ(unnamed_input.get_name(), "u0");
  EXPECT_EQ(named_input.get_name(), "my_input");
  EXPECT_EQ(named_abstract_input.get_name(), "abstract");

  // Duplicate port names should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      system_.DeclareInputPort("my_input", kAbstractValued, 0),
      std::logic_error, ".*already has an input port named.*");

  // Test string-based get_input_port accessors.
  EXPECT_EQ(&system_.GetInputPort("u0"), &unnamed_input);
  EXPECT_EQ(&system_.GetInputPort("my_input"), &named_input);
  EXPECT_EQ(&system_.GetInputPort("abstract"), &named_abstract_input);
  EXPECT_EQ(system_.HasInputPort("u0"), true);
  EXPECT_EQ(system_.HasInputPort("fake_name"), false);

  // Test output port names.
  const auto& output_port = system_.AddAbstractOutputPort();
  EXPECT_EQ(output_port.get_name(), "y0");
  EXPECT_EQ(&system_.GetOutputPort("y0"), &output_port);
  EXPECT_EQ(system_.HasOutputPort("y0"), true);
  EXPECT_EQ(system_.HasOutputPort("fake_name"), false);

  // Requesting a non-existing port name should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      system_.GetInputPort("not_my_input"),
      std::logic_error, ".*does not have an input port named.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      system_.GetOutputPort("not_my_output"),
      std::logic_error, ".*does not have an output port named.*");
}

TEST_F(SystemTest, PortSelectionTest) {
  // Input ports.
  EXPECT_EQ(system_.get_input_port_selection(InputPortSelection::kNoInput),
            nullptr);
  EXPECT_EQ(system_.get_input_port_selection(
                InputPortSelection::kUseFirstInputIfItExists),
            nullptr);

  const auto& input_port =
      system_.DeclareInputPort("my_input", kVectorValued, 3);
  EXPECT_EQ(system_.get_input_port_selection(
                InputPortSelection::kUseFirstInputIfItExists),
            &input_port);

  EXPECT_EQ(system_.get_input_port_selection(InputPortIndex{0}),
            &system_.get_input_port(0));

  // Output ports.
  EXPECT_EQ(system_.get_output_port_selection(OutputPortSelection::kNoOutput),
            nullptr);
  EXPECT_EQ(system_.get_output_port_selection(
                OutputPortSelection::kUseFirstOutputIfItExists),
            nullptr);

  const auto& output_port = system_.AddAbstractOutputPort();
  EXPECT_EQ(system_.get_output_port_selection(
                OutputPortSelection::kUseFirstOutputIfItExists),
            &output_port);

  EXPECT_EQ(system_.get_output_port_selection(OutputPortIndex{0}),
            &system_.get_output_port(0));
}

// Tests the constraint list logic.
TEST_F(SystemTest, SystemConstraintTest) {
  EXPECT_EQ(system_.num_constraints(), 0);
  EXPECT_THROW(system_.get_constraint(SystemConstraintIndex(0)),
               std::out_of_range);

  ContextConstraintCalc<double> calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    unused(context);
    (*value)[0] = 1.0;
  };
  const double kInf = std::numeric_limits<double>::infinity();
  SystemConstraintIndex test_constraint =
      system_.AddConstraint(std::make_unique<SystemConstraint<double>>(
          &system_, calc, SystemConstraintBounds(Vector1d(0), std::nullopt),
          "test"));
  EXPECT_EQ(test_constraint, 0);

  DRAKE_EXPECT_NO_THROW(system_.get_constraint(test_constraint));
  const auto& constraint_ref = system_.get_constraint(test_constraint);
  EXPECT_EQ(constraint_ref.description(), "test");
  EXPECT_TRUE(constraint_ref.get_system_id().has_value());
  EXPECT_EQ(*constraint_ref.get_system_id(), context_->get_system_id());

  const double tol = 1e-6;
  EXPECT_TRUE(system_.CheckSystemConstraintsSatisfied(*context_, tol));
  ContextConstraintCalc<double> calc_false = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    unused(context);
    (*value)[0] = -1.0;
  };
  system_.AddConstraint(std::make_unique<SystemConstraint<double>>(
      &system_, calc_false, SystemConstraintBounds(Vector1d(0), Vector1d(kInf)),
      "bad constraint"));
  EXPECT_FALSE(system_.CheckSystemConstraintsSatisfied(*context_, tol));
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
  EXPECT_THROW(
      (System<double>::ToScalarType<AutoDiffXd, TestSystemBase>(system_)),
      std::exception);
  EXPECT_THROW(
      (System<double>::ToScalarType<symbolic::Expression, TestSystemBase>(
          system_)),
      std::exception);

  // Use the instance method that throws.
  EXPECT_THROW(system_.ToAutoDiffXd(), std::exception);
  EXPECT_THROW(system_.ToSymbolic(), std::exception);
  EXPECT_THROW(system_.ToScalarType<AutoDiffXd>(), std::exception);
  EXPECT_THROW(system_.ToScalarType<symbolic::Expression>(),
               std::exception);

  // Use the instance method that returns nullptr.
  EXPECT_EQ(system_.ToAutoDiffXdMaybe(), nullptr);
  EXPECT_EQ(system_.ToSymbolicMaybe(), nullptr);
  EXPECT_EQ(system_.ToScalarTypeMaybe<AutoDiffXd>(), nullptr);
  EXPECT_EQ(system_.ToScalarTypeMaybe<symbolic::Expression>(), nullptr);

  // Spot check the specific converter object.
  EXPECT_FALSE((
      system_.get_system_scalar_converter().IsConvertible<double, double>()));
}

// Tests IsDifferenceEquationSystem works for this one System.  Additional
// test coverage is provided in linear_system_test.cc and diagram_test.cc.
TEST_F(SystemTest, IsDifferenceEquationSystem) {
  double period = 1.23;
  EXPECT_FALSE(system_.IsDifferenceEquationSystem(&period));
  // Confirm that the return parameter was not changed.
  EXPECT_EQ(period, 1.23);
}

template <typename T>
using TestTypedVector = MyVector<T, 1>;

// A shell System for AbstractValue IO test.
template <typename T>
class ValueIOTestSystem : public TestSystemBase<T> {
 public:
  // Has 4 input and 2 output ports.
  // The first input / output pair are abstract type, but assumed to be
  // std::string.
  // The second input / output pair are vector type with length 1.
  // There are two other vector-valued random input ports.
  ValueIOTestSystem() {
    this->set_forced_publish_events(
        this->AllocateForcedPublishEventCollection());
    this->set_forced_discrete_update_events(
        this->AllocateForcedDiscreteUpdateEventCollection());
    this->set_forced_unrestricted_update_events(
        this->AllocateForcedUnrestrictedUpdateEventCollection());

    this->DeclareInputPort(kUseDefaultName, kAbstractValued, 0);

    this->AddOutputPort(internal::FrameworkFactory::Make<LeafOutputPort<T>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        this->get_system_id(),
        "absport",
        OutputPortIndex(this->num_output_ports()),
        this->assign_next_dependency_ticket(),
        kAbstractValued, 0 /* size */,
        &this->DeclareCacheEntry(
            "absport", &ValueIOTestSystem::CalcStringOutput)));
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 1);
    this->DeclareInputPort("uniform", kVectorValued, 1,
                           RandomDistribution::kUniform);
    this->DeclareInputPort("gaussian", kVectorValued, 1,
                           RandomDistribution::kGaussian);
    this->AddOutputPort(internal::FrameworkFactory::Make<LeafOutputPort<T>>(
        this,  // implicit_cast<const System<T>*>(this)
        this,  // implicit_cast<const SystemBase*>(this)
        this->get_system_id(),
        "vecport",
        OutputPortIndex(this->num_output_ports()),
        this->assign_next_dependency_ticket(),
        kVectorValued, 1 /* size */,
        &this->DeclareCacheEntry(
            "vecport", BasicVector<T>(1),
            &ValueIOTestSystem::CalcVectorOutput)));

    this->set_name("ValueIOTestSystem");
  }

  ~ValueIOTestSystem() override {}

  const InputPort<T>& AddAbstractInputPort() {
    return this->DeclareInputPort(kUseDefaultName, kAbstractValued, 0);
  }

  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const override {
    if (input_port.get_index() == 0) {
      return AbstractValue::Make<std::string>();
    } else {
      return std::make_unique<Value<BasicVector<T>>>(TestTypedVector<T>{});
    }
  }

  std::multimap<int, int> GetDirectFeedthroughs() const override {
    std::multimap<int, int> pairs;
    // Report *everything* as having direct feedthrough.
    for (int i = 0; i < this->num_input_ports(); ++i) {
      for (int o = 0; o < this->num_output_ports(); ++o) {
        pairs.emplace(i, o);
      }
    }
    return pairs;
  }

  // Appends "output" to input(0) for output(0).
  void CalcStringOutput(const ContextBase& context,
                        std::string* output) const {
    const std::string* str_in =
        this->template EvalInputValue<std::string>(context, 0);
    *output = *str_in + "output";
  }

  // Sets output(1) = 2 * input(1).
  void CalcVectorOutput(const ContextBase& context_base,
                        BasicVector<T>* output) const {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    const BasicVector<T>* vec_in = this->EvalVectorInput(context, 1);
    output->get_mutable_value() = 2 * vec_in->get_value();
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
class WrongVector : public MyVector<T, 2> {
 public:
  using MyVector<T, 2>::MyVector;
};

// Test error messages from the EvalInput methods.
TEST_F(SystemInputErrorTest, CheckMessages) {
  ASSERT_EQ(system_.num_input_ports(), 4);

  // Sanity check that this works with a good port number.
  DRAKE_EXPECT_NO_THROW(system_.get_input_port(1));

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

  // No ports have values yet.
  EXPECT_EQ(system_.EvalVectorInput(*context_, 1), nullptr);
  EXPECT_EQ(system_.EvalAbstractInput(*context_, 1), nullptr);
  EXPECT_EQ(system_.EvalInputValue<int>(*context_, 1), nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 1), std::logic_error,
      ".*EvalEigenVectorInput.*input port 'u1' .*index 1.* is neither "
      "connected nor fixed.*");
#pragma GCC diagnostic pop

  // Assign values to all ports. All but port 0 are BasicVector ports.
  system_.AllocateFixedInputs(context_.get());

  DRAKE_EXPECT_NO_THROW(
      system_.EvalVectorInput(*context_, 2));  // BasicVector OK.
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput<WrongVector>(*context_, 2), std::logic_error,
      ".*EvalVectorInput.*expected.*WrongVector"
          ".*input port.*2.*actual.*MyVector.*");

  DRAKE_EXPECT_NO_THROW(
      system_.EvalInputValue<BasicVector<double>>(*context_, 1));
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<int>(*context_, 1), std::logic_error,
      ".*EvalInputValue.*expected.*int.*input port.*1.*actual.*MyVector.*");

  // Now induce errors that only apply to abstract-valued input ports.

  EXPECT_EQ(*system_.EvalInputValue<std::string>(*context_, 0), "");
  DRAKE_EXPECT_NO_THROW(system_.EvalAbstractInput(*context_, 0));
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalVectorInput(*context_, 0), std::logic_error,
      ".*EvalVectorInput.*vector port required.*input port.*0.*"
          "was declared abstract.*");

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalInputValue<double>(*context_, 0),
      std::logic_error,
      ".*EvalInputValue.*expected.*double.*input port.*0.*actual.*string.*");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, -4), std::out_of_range,
      ".*EvalEigenVectorInput.*negative.*-4.*illegal.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 12), std::out_of_range,
      ".*EvalEigenVectorInput.*no input port.*12.*only.*4.*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      system_.EvalEigenVectorInput(*context_, 0),
      std::logic_error,
      ".*EvalEigenVectorInput.*vector port required.*input port.*0.*"
          "was declared abstract.*");
#pragma GCC diagnostic pop
}

// Provides values for some of the inputs and sets up for outputs.
class SystemIOTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = test_sys_.CreateDefaultContext();
    output_ = test_sys_.AllocateOutput();

    // make string input
    test_sys_.get_input_port(0).FixValue(context_.get(), "input");

    // make vector input
    test_sys_.get_input_port(1).FixValue(context_.get(), 2.0);
  }

  ValueIOTestSystem<double> test_sys_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

TEST_F(SystemIOTest, SystemValueIOTest) {
  test_sys_.CalcOutput(*context_, output_.get());

  EXPECT_EQ(context_->num_input_ports(), 4);
  EXPECT_EQ(output_->num_ports(), 2);

  EXPECT_EQ(output_->get_data(0)->get_value<std::string>(),
            std::string("inputoutput"));
  EXPECT_EQ(output_->get_vector_data(1)->get_value()(0), 4);


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Connected inputs ports can be evaluated.  (Port #1 was set to [2]).
  const auto& block = test_sys_.EvalEigenVectorInput(*context_, 1);
  ASSERT_EQ(block.size(), 1);
  ASSERT_EQ(block[0], 2.0);
  EXPECT_THROW(test_sys_.EvalEigenVectorInput(*context_, 2), std::exception);
#pragma GCC diagnostic pop

  // Disconnected inputs are nullptr.
  EXPECT_EQ(test_sys_.EvalVectorInput(*context_, 2), nullptr);

  // Test AllocateInput*
  // Second input is not (yet) a TestTypedVector, since I haven't called the
  // Allocate methods directly yet.
  EXPECT_EQ(dynamic_cast<const TestTypedVector<double> *>(
                test_sys_.EvalVectorInput(*context_, 1)),
            nullptr);
  // Now allocate.
  test_sys_.AllocateFixedInputs(context_.get());
  // First input should have been re-allocated to the empty string.
  EXPECT_EQ(test_sys_.EvalAbstractInput(*context_, 0)->get_value<std::string>(),
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
      dest_system.EvalAbstractInput(*dest_context, 0)->get_value<std::string>(),
      "input");

  const TestTypedVector<AutoDiffXd>* fixed_vec =
      dest_system.EvalVectorInput<TestTypedVector>(*dest_context, 1);
  EXPECT_NE(fixed_vec, nullptr);
  EXPECT_EQ(2, fixed_vec->GetAtIndex(0).value());
  EXPECT_EQ(0, fixed_vec->GetAtIndex(0).derivatives().size());
}

// Confirm that FixInputPortsFrom does *not* convert type-dependent abstract
// input ports.
// TODO(5454) Once transmogrification of scalar-dependent abstract values is
// implemented, this test and the corresponding @throws documentation on
// System::FixInputPortsFrom can simply be removed (as we no longer have to
// track this undesirable behavior) .
TEST_F(SystemIOTest, FixFromTypeDependentAbstractInput) {
  // Adds an abstract input port with type BasicVector<T>.
  const auto& typed_input = test_sys_.AddAbstractInputPort();

  // Confirm that the type is indeed BasicVector<double>.
  std::unique_ptr<AbstractValue> input_value =
      test_sys_.AllocateInputAbstract(typed_input);
  DRAKE_EXPECT_NO_THROW(input_value->get_value<BasicVector<double>>());

  const auto context = test_sys_.CreateDefaultContext();
  typed_input.FixValue(context.get(),
                       BasicVector<double>(Eigen::VectorXd::Zero(1)));

  ValueIOTestSystem<AutoDiffXd> autodiff_system;
  autodiff_system.AddAbstractInputPort();
  auto autodiff_context = autodiff_system.CreateDefaultContext();

  DRAKE_EXPECT_THROWS_MESSAGE(autodiff_system.FixInputPortsFrom(
                                  test_sys_, *context, autodiff_context.get()),
                              ".*System::FixInputPortTypeCheck.*");
}

// This class implements various computational methods so we can check that
// they get invoked properly. The particular results don't mean anything.
// As above, lots of painful bookkeeping here that is normally buried by
// LeafSystem.
class ComputationTestSystem final : public TestSystemBase<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ComputationTestSystem)

  ComputationTestSystem() {
    DeclareInputPort("u0", kVectorValued, 1);
  }

  // Just u0.
  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<double>& input_port) const final {
    DRAKE_DEMAND(input_port.get_index() == 0);
    return std::make_unique<Value<BasicVector<double>>>(1);
  }

  // One q, one v, one z.
  std::unique_ptr<ContinuousState<double>> AllocateTimeDerivatives()
      const final {
    auto result = std::make_unique<ContinuousState<double>>(
        std::make_unique<BasicVector<double>>(3), 1, 1, 1);  // q, v, z
    result->set_system_id(this->get_system_id());
    return result;
  }

  // Verify that the number of calls is as expected.
  void ExpectCount(int xcdot, int pe, int ke, int pc, int pnc) const {
    EXPECT_EQ(xcdot, xcdot_count_);
    EXPECT_EQ(pe, pe_count_);
    EXPECT_EQ(ke, ke_count_);
    EXPECT_EQ(pc, pc_count_);
    EXPECT_EQ(pnc, pnc_count_);
  }

 private:
  // Two discrete variable groups of lengths 2 and 4.
  std::unique_ptr<DiscreteValues<double>> AllocateDiscreteVariables()
      const final {
    std::vector<std::unique_ptr<BasicVector<double>>> data;
    data.emplace_back(std::make_unique<BasicVector<double>>(2));
    data.emplace_back(std::make_unique<BasicVector<double>>(4));
    auto result = std::make_unique<DiscreteValues<double>>(std::move(data));
    result->set_system_id(this->get_system_id());
    return result;
  }

  // Derivatives can depend on time. Here x = (-1, -2, -3) * t.
  void DoCalcTimeDerivatives(const Context<double>& context,
                             ContinuousState<double>* derivatives) const final {
    unused(context);
    EXPECT_EQ(derivatives->size(), 3);
    const double t = context.get_time();
    (*derivatives)[0] = -1 * t;
    (*derivatives)[1] = -2 * t;
    (*derivatives)[2] = -3 * t;
    ++xcdot_count_;
  }

  double DoCalcPotentialEnergy(const Context<double>& context) const final {
    unused(context);
    ++pe_count_;
    return 1.;
  }

  double DoCalcKineticEnergy(const Context<double>& context) const final {
    unused(context);
    ++ke_count_;
    return 2.;
  }

  double DoCalcConservativePower(const Context<double>& context) const final {
    unused(context);
    ++pc_count_;
    return 3.;
  }

  // Non-conservative power can depend on time. Here it is 4*t.
  double DoCalcNonConservativePower(
      const Context<double>& context) const final {
    ++pnc_count_;
    return 4. * context.get_time();
  }

  mutable int xcdot_count_{};
  mutable int pe_count_{};
  mutable int ke_count_{};
  mutable int pc_count_{};
  mutable int pnc_count_{};
};

// Provides values for some of the inputs and sets up for outputs.
class ComputationTest : public ::testing::Test {
 protected:
  void SetUp() final {
    context_->EnableCaching();
  }

  ComputationTestSystem test_sys_;
  std::unique_ptr<Context<double>> context_ =
      test_sys_.CreateDefaultContext();
};

TEST_F(ComputationTest, Eval) {
  context_->SetTime(1.);

  //                 xcdot, pe, ke, pc, pnc
  test_sys_.ExpectCount(0, 0, 0, 0, 0);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[0], -1.);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[1], -2.);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[2], -3.);
  // Caching should have kept us to a single derivative evaluation.
  test_sys_.ExpectCount(1, 0, 0, 0, 0);

  EXPECT_EQ(test_sys_.EvalPotentialEnergy(*context_), 1.);
  test_sys_.ExpectCount(1, 1, 0, 0, 0);
  EXPECT_EQ(test_sys_.EvalKineticEnergy(*context_), 2.);
  test_sys_.ExpectCount(1, 1, 1, 0, 0);
  EXPECT_EQ(test_sys_.EvalConservativePower(*context_), 3.);
  test_sys_.ExpectCount(1, 1, 1, 1, 0);
  EXPECT_EQ(test_sys_.EvalNonConservativePower(*context_), 4.);
  test_sys_.ExpectCount(1, 1, 1, 1, 1);

  // These should not require re-evaluation.
  EXPECT_EQ(test_sys_.EvalPotentialEnergy(*context_), 1.);
  EXPECT_EQ(test_sys_.EvalKineticEnergy(*context_), 2.);
  EXPECT_EQ(test_sys_.EvalConservativePower(*context_), 3.);
  EXPECT_EQ(test_sys_.EvalNonConservativePower(*context_), 4.);
  test_sys_.ExpectCount(1, 1, 1, 1, 1);


  // Each of the Calc methods should cause computation.
  auto derivatives = test_sys_.AllocateTimeDerivatives();
  test_sys_.CalcTimeDerivatives(*context_, derivatives.get());
  test_sys_.ExpectCount(2, 1, 1, 1, 1);
  EXPECT_EQ((*derivatives)[1], -2.);
  EXPECT_EQ(test_sys_.CalcPotentialEnergy(*context_), 1.);
  test_sys_.ExpectCount(2, 2, 1, 1, 1);
  EXPECT_EQ(test_sys_.CalcKineticEnergy(*context_), 2.);
  test_sys_.ExpectCount(2, 2, 2, 1, 1);
  EXPECT_EQ(test_sys_.CalcConservativePower(*context_), 3.);
  test_sys_.ExpectCount(2, 2, 2, 2, 1);
  EXPECT_EQ(test_sys_.CalcNonConservativePower(*context_), 4.);
  test_sys_.ExpectCount(2, 2, 2, 2, 2);

  // TODO(sherm1) Pending resolution of issue #9171 we can be more precise
  // about dependencies here. For now we'll just verify that changing
  // some significant variables causes recomputation, not that *only*
  // significant variables cause recomputation.
  context_->SetTime(2.);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[0], -2.);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[1], -4.);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[2], -6.);
  test_sys_.ExpectCount(3, 2, 2, 2, 2);  // Above is just one evaluation.
  EXPECT_EQ(test_sys_.EvalPotentialEnergy(*context_), 1.);
  EXPECT_EQ(test_sys_.EvalKineticEnergy(*context_), 2.);
  EXPECT_EQ(test_sys_.EvalConservativePower(*context_), 3.);
  EXPECT_EQ(test_sys_.EvalNonConservativePower(*context_), 8.);
  test_sys_.ExpectCount(3, 2, 2, 2, 3);  // Only pnc depends on time.

  // Modify an input. Derivatives are recomputed, but PE, KE, PC are not.
  const Eigen::VectorXd u0 = Eigen::VectorXd::Constant(1, 0.0);
  test_sys_.get_input_port(0).FixValue(context_.get(), u0);
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[0], -2.);
  test_sys_.ExpectCount(4, 2, 2, 2, 3);
  EXPECT_EQ(test_sys_.EvalPotentialEnergy(*context_), 1.);
  EXPECT_EQ(test_sys_.EvalKineticEnergy(*context_), 2.);
  EXPECT_EQ(test_sys_.EvalConservativePower(*context_), 3.);
  EXPECT_EQ(test_sys_.EvalNonConservativePower(*context_), 8.);
  test_sys_.ExpectCount(4, 2, 2, 2, 4);  // Only pnc depends on input.

  // This should mark all state variables as changed and force recomputation.
  context_->get_mutable_state();
  EXPECT_EQ(test_sys_.EvalTimeDerivatives(*context_)[2], -6.);  // Again.
  test_sys_.ExpectCount(5, 2, 2, 2, 4);
  EXPECT_EQ(test_sys_.EvalPotentialEnergy(*context_), 1.);
  test_sys_.ExpectCount(5, 3, 2, 2, 4);
  EXPECT_EQ(test_sys_.EvalKineticEnergy(*context_), 2.);
  test_sys_.ExpectCount(5, 3, 3, 2, 4);
  EXPECT_EQ(test_sys_.EvalConservativePower(*context_), 3.);
  test_sys_.ExpectCount(5, 3, 3, 3, 4);
  EXPECT_EQ(test_sys_.EvalNonConservativePower(*context_), 8.);  // Again.
  test_sys_.ExpectCount(5, 3, 3, 3, 5);

  // Check that the reported time derivatives cache entry is the right one.
  context_->SetTime(3.);  // Invalidate.
  const CacheEntry& entry = test_sys_.get_time_derivatives_cache_entry();
  const int64_t serial_number =
      entry.get_cache_entry_value(*context_).serial_number();
  test_sys_.EvalTimeDerivatives(*context_);
  EXPECT_NE(entry.get_cache_entry_value(*context_).serial_number(),
            serial_number);
}

}  // namespace
}  // namespace systems
}  // namespace drake
