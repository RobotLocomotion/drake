#include "drake/systems/framework/diagram_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/integrator.h"

namespace drake {
namespace systems {
namespace {

constexpr int kSize = 1;
constexpr int kNumSystems = 8;
constexpr double kTime = 12.0;

class SystemWithDiscreteState : public LeafSystem<double> {
 public:
  SystemWithDiscreteState() {
    DeclareDiscreteState(1);
  }
  ~SystemWithDiscreteState() override {}
};

class SystemWithAbstractState : public LeafSystem<double> {
 public:
  SystemWithAbstractState() {
    DeclareAbstractState(Value<int>(42));
  }
  ~SystemWithAbstractState() override {}
};

class SystemWithNumericParameters : public LeafSystem<double> {
 public:
  SystemWithNumericParameters() {
    DeclareNumericParameter(BasicVector<double>(2));
  }
  ~SystemWithNumericParameters() override {}
};

class SystemWithAbstractParameters : public LeafSystem<double> {
 public:
  SystemWithAbstractParameters() {
    DeclareAbstractParameter(Value<int>(2048));
  }
  ~SystemWithAbstractParameters() override {}
};

}  // namespace

// This class must be outside the anonymous namespace to permit the
// DiagramContext friend declaration to work.

class DiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    adder0_ = std::make_unique<Adder<double>>(2 /* inputs */, kSize);
    adder1_ = std::make_unique<Adder<double>>(2 /* inputs */, kSize);

    integrator0_.reset(new Integrator<double>(kSize));
    integrator1_.reset(new Integrator<double>(kSize));

    discrete_state_system_ = std::make_unique<SystemWithDiscreteState>();
    abstract_state_system_ = std::make_unique<SystemWithAbstractState>();
    system_with_numeric_parameters_ =
        std::make_unique<SystemWithNumericParameters>();
    system_with_abstract_parameters_ =
        std::make_unique<SystemWithAbstractParameters>();

    adder0_->set_name("adder0");
    adder1_->set_name("adder1");
    integrator0_->set_name("integrator0");
    integrator1_->set_name("integrator1");
    discrete_state_system_->set_name("discrete_state_system");
    abstract_state_system_->set_name("abstract_state_system");
    system_with_numeric_parameters_->set_name("system_with_numeric_parameters");
    system_with_abstract_parameters_->set_name(
        "system_with_abstract_parameters");

    // This chunk of code is partially mimicking Diagram::DoAllocateContext()
    // which is normally in charge of making DiagramContexts.
    context_ = std::make_unique<DiagramContext<double>>(kNumSystems);
    internal::SystemBaseContextBaseAttorney::set_system_id(
        context_.get(), internal::SystemId::get_new_id());

    // Don't change these indexes -- tests below depend on them.
    AddSystem(*adder0_, SubsystemIndex(0));
    AddSystem(*adder1_, SubsystemIndex(1));
    AddSystem(*integrator0_, SubsystemIndex(2));
    AddSystem(*integrator1_, SubsystemIndex(3));
    AddSystem(*discrete_state_system_, SubsystemIndex(4));
    AddSystem(*abstract_state_system_, SubsystemIndex(5));
    AddSystem(*system_with_numeric_parameters_, SubsystemIndex(6));
    AddSystem(*system_with_abstract_parameters_, SubsystemIndex(7));

    // Fake up some input ports for this diagram.
    context_->AddInputPort(InputPortIndex(0), DependencyTicket(100), {});
    context_->AddInputPort(InputPortIndex(1), DependencyTicket(101), {});

    context_->MakeState();
    context_->MakeParameters();
    context_->SubscribeDiagramCompositeTrackersToChildrens();

    context_->SetTime(kTime);
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc.get_mutable_vector()[0] = 42.0;
    xc.get_mutable_vector()[1] = 43.0;

    DiscreteValues<double>& xd = context_->get_mutable_discrete_state();
    xd.get_mutable_vector(0)[0] = 44.0;

    context_->get_mutable_numeric_parameter(0)[0] = 76.0;
    context_->get_mutable_numeric_parameter(0)[1] = 77.0;

    // Sanity checks: tests below count on these dimensions.
    EXPECT_EQ(context_->num_continuous_states(), 2);
    EXPECT_EQ(context_->num_discrete_state_groups(), 1);
    EXPECT_EQ(context_->num_abstract_states(), 1);
    EXPECT_EQ(context_->num_numeric_parameter_groups(), 1);
    EXPECT_EQ(context_->num_abstract_parameters(), 1);
    EXPECT_EQ(context_->num_subcontexts(), kNumSystems);
  }

  // Some tests below need to know which of the subsystems above have particular
  // resources. They use kNumSystems to identify the "parent" subsystem which
  // inherits all the resources of its children.
  static std::set<int> has_discrete_state() { return {4, kNumSystems}; }
  static std::set<int> has_abstract_state() { return {5, kNumSystems}; }
  static std::set<int> has_numeric_parameter() { return {6, kNumSystems}; }
  static std::set<int> has_abstract_parameter() { return {7, kNumSystems}; }
  static std::set<int> has_parameter() { return {6, 7, kNumSystems}; }

  void AddSystem(const System<double>& sys, SubsystemIndex index) {
    auto subcontext = sys.CreateDefaultContext();
    context_->AddSystem(index, std::move(subcontext));
  }

  void AttachInputPorts() {
    context_->FixInputPort(0, Value<BasicVector<double>>(
                                  Vector1<double>(128.0)));
    context_->FixInputPort(1, Value<BasicVector<double>>(
                                  Vector1<double>(256.0)));
  }

  // Reads a FixedInputPortValue connected to @p context at @p index.
  // Returns nullptr if the port is not connected.
  const BasicVector<double>* ReadVectorInputPort(const Context<double>& context,
                                                 int index) {
    const FixedInputPortValue* free_value =
        context.MaybeGetFixedInputPortValue(InputPortIndex(index));
    return free_value ? &free_value->get_vector_value<double>() : nullptr;
  }


  // Check that time is set as expected in the Diagram and all the subcontexts.
  void VerifyTimeValue(double expected_time) {
    // Check the Diagram.
    EXPECT_EQ(context_->get_time(), expected_time);

    // Make sure time got delivered to the subcontexts.
    for (SubsystemIndex i(0); i < kNumSystems; ++i) {
      const auto& subcontext = context_->GetSubsystemContext(i);
      EXPECT_EQ(subcontext.get_time(), expected_time);
    }
  }

  // Check that accuracy is set to the expected numerical value in the Diagram
  // and all the subcontexts. Don't call this for uninitialized (optional)
  // accuracy.
  void VerifyAccuracyValue(double expected_accuracy) {
    // Check the Diagram.
    ASSERT_TRUE(context_->get_accuracy());
    EXPECT_EQ(context_->get_accuracy().value(), expected_accuracy);

    // Make sure time got delivered to the subcontexts.
    for (SubsystemIndex i(0); i < kNumSystems; ++i) {
      const auto& subcontext = context_->GetSubsystemContext(i);
      ASSERT_TRUE(subcontext.get_accuracy());
      EXPECT_EQ(subcontext.get_accuracy().value(), expected_accuracy);
    }
  }

  // Check that the continuous state has the expected value, in both the
  // Diagram and its continuous-state-holding subcontexts, which are the two
  // integrators.
  void VerifyContinuousStateValue(const Eigen::Vector2d& expected) {
    // Make sure state is right in the Diagram.
    EXPECT_EQ(context_->get_continuous_state_vector().CopyToVector(), expected);

    // Make sure state got delivered to the integrators.
    EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(2))
                  .get_continuous_state_vector()[0],
              expected[0]);
    EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(3))
                  .get_continuous_state_vector()[0],
              expected[1]);
  }

  // Record the notification count for the given tracker in each of the
  // subcontexts, followed by the notification count for that same tracker in
  // the diagram context, so there are kNumSystems + 1 in the returned vector.
  std::vector<int64_t> SaveNotifications(DependencyTicket ticket) {
    std::vector<int64_t> notifications;
    for (SubsystemIndex i(0); i < kNumSystems; ++i) {
      notifications.push_back(
          NumNotifications(context_->GetSubsystemContext(i), ticket));
    }
    notifications.push_back(NumNotifications(*context_, ticket));
    return notifications;
  }

  // Verify that the current notification count is as expected, relative to the
  // given `before_count`. `should_have_been_notified` says which subsystems
  // should have received a notification, with kNumSystems treated as the index
  // of the diagram "subsystem". `before_count` is updated on return so it can
  // be used in a subsequent test.
  void VerifyNotifications(const std::string& which,
                           const std::set<int>& should_have_been_notified,
                           DependencyTicket ticket,
                           std::vector<int64_t>* before_count) {  // in/out
    auto count_now = SaveNotifications(ticket);
    ASSERT_EQ(count_now.size(), kNumSystems + 1);
    ASSERT_EQ(before_count->size(), kNumSystems + 1);
    for (int i = 0; i <= kNumSystems; ++i) {
      const int n = should_have_been_notified.count(i) ? 1 : 0;
      (*before_count)[i] += n;
      EXPECT_EQ(count_now[i], (*before_count)[i]) << which << " of system "
                                                  << i;
    }
  }

  // Verify that all subsystem trackers with this ticket were notified,
  // including the diagram, and updated the expected notification count.
  void VerifyNotifications(const std::string& which, DependencyTicket ticket,
                           std::vector<int64_t>* before_count) {  // in/out
    std::set<int> should_have_been_notified;
    for (int i = 0; i <= kNumSystems; ++i) should_have_been_notified.insert(i);
    VerifyNotifications(which, should_have_been_notified, ticket, before_count);
  }

  // Return the current notification count for the given tracker in the given
  // context. Don't count multiple notifications for the same change event.
  static int64_t NumNotifications(const ContextBase& context,
                                  DependencyTicket ticket) {
    const auto& tracker = context.get_tracker(ticket);
    return tracker.num_notifications_received()
        - tracker.num_ignored_notifications();
  }

  std::unique_ptr<DiagramContext<double>> context_;
  std::unique_ptr<Adder<double>> adder0_;
  std::unique_ptr<Adder<double>> adder1_;
  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;
  std::unique_ptr<SystemWithDiscreteState> discrete_state_system_;
  std::unique_ptr<SystemWithAbstractState> abstract_state_system_;
  std::unique_ptr<SystemWithNumericParameters> system_with_numeric_parameters_;
  std::unique_ptr<SystemWithAbstractParameters>
      system_with_abstract_parameters_;
};

namespace {

// Verifies that @p state is a clone of the state constructed in
// DiagramContextTest::SetUp.
void VerifyClonedState(const State<double>& clone) {
  // - Continuous
  const ContinuousState<double>& xc = clone.get_continuous_state();
  EXPECT_EQ(42.0, xc.get_vector()[0]);
  EXPECT_EQ(43.0, xc.get_vector()[1]);
  // - Discrete
  const DiscreteValues<double>& xd = clone.get_discrete_state();
  EXPECT_EQ(44.0, xd.get_vector(0)[0]);
  // - Abstract
  const AbstractValues& xa = clone.get_abstract_state();
  EXPECT_EQ(42, xa.get_value(0).get_value<int>());
}

// Verifies that the @p params are a clone of the params constructed in
// DiagramContextTest::SetUp.
void VerifyClonedParameters(const Parameters<double>& params) {
  ASSERT_EQ(1, params.num_numeric_parameter_groups());
  EXPECT_EQ(76.0, params.get_numeric_parameter(0)[0]);
  EXPECT_EQ(77.0, params.get_numeric_parameter(0)[1]);
  ASSERT_EQ(1, params.num_abstract_parameters());
  EXPECT_EQ(2048, UnpackIntValue(params.get_abstract_parameter(0)));
}

// Tests that subsystems have contexts in the DiagramContext.
TEST_F(DiagramContextTest, RetrieveConstituents) {
  // All of the subsystems should be leaf Systems.
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    auto context = dynamic_cast<const LeafContext<double>*>(
        &context_->GetSubsystemContext(i));
    EXPECT_TRUE(context != nullptr);
  }
}

// The notification tests here are verifying the "down" notification direction.
// That is, we want to see that if we modify a context value at the Diagram
// level, change notifications propagate out to the contained LeafSystems.
// The other direction, modifying a LeafContext value that should notify
// the Diagram, is verified in diagram_test.

// Tests that the time writes through to the subsystem contexts, and that all
// time trackers are notified.
TEST_F(DiagramContextTest, Time) {
  auto before = SaveNotifications(SystemBase::time_ticket());

  context_->SetTime(42.0);
  VerifyTimeValue(42.);
  VerifyNotifications("Time", SystemBase::time_ticket(), &before);
}

// Tests that the accuracy writes through to the subsystem contexts, and that
// all accuracy trackers are notified.
TEST_F(DiagramContextTest, Accuracy) {
  auto before = SaveNotifications(SystemBase::accuracy_ticket());

  const double new_accuracy = 1e-12;
  context_->SetAccuracy(new_accuracy);
  VerifyAccuracyValue(new_accuracy);
  VerifyNotifications("Accuracy", SystemBase::accuracy_ticket(), &before);
}

// Test that State notifications propagate down from the diagram to the leaves.
TEST_F(DiagramContextTest, MutableStateNotifications) {
  auto x_before = SaveNotifications(SystemBase::all_state_ticket());
  auto xc_before = SaveNotifications(SystemBase::xc_ticket());
  auto xd_before = SaveNotifications(SystemBase::xd_ticket());
  auto xa_before = SaveNotifications(SystemBase::xa_ticket());

  // Changing the whole state should affect all the substates.
  context_->get_mutable_state();  // Return value ignored.
  VerifyNotifications("get_mutable_state: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_state: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_state: xd", has_discrete_state(),
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("get_mutable_state: xa", has_abstract_state(),
                      SystemBase::xa_ticket(), &xa_before);

  // Changing continuous state should affect only x and xc.
  context_->get_mutable_continuous_state();  // Return value ignored.
  VerifyNotifications("get_mutable_continuous_state: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_continuous_state: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_continuous_state: xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("get_mutable_continuous_state: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  context_->get_mutable_continuous_state_vector();  // Return value ignored.
  VerifyNotifications("get_mutable_continuous_state_vector: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_continuous_state_vector: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_continuous_state_vector: xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("get_mutable_continuous_state_vector: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  const Eigen::Vector2d new_xc1(3.25, 5.5);
  context_->SetContinuousState(VectorX<double>(new_xc1));
  VerifyContinuousStateValue(new_xc1);
  VerifyNotifications("SetContinuousState: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("SetContinuousState: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("SetContinuousState: xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("SetContinuousState: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  // Changing time and continuous state should affect only t, x and xc.
  auto t_before = SaveNotifications(SystemBase::time_ticket());
  const double new_time = context_->get_time() + 1.;
  const Eigen::Vector2d new_xc2(1.25, 1.5);
  context_->SetTimeAndContinuousState(new_time, VectorX<double>(new_xc2));
  VerifyTimeValue(new_time);
  // Make sure state got delivered to the integrators.
  VerifyContinuousStateValue(new_xc2);
  // Make sure notifications got delivered.
  VerifyNotifications("SetTimeAndContinuousState: t",
                      SystemBase::time_ticket(), &t_before);
  VerifyNotifications("SetTimeAndContinuousState: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("SetTimeAndContinuousState: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("SetTimeAndContinuousState: xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("SetTimeAndContinuousState: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  // Changing discrete state should affect only x and xd, and those should only
  // get notified for systems that actually have discrete variables.

  context_->get_mutable_discrete_state();  // Return value ignored.
  VerifyNotifications("get_mutable_discrete_state: x", has_discrete_state(),
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_discrete_state: xd", has_discrete_state(),
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("get_mutable_discrete_state: xc", {},  // None.
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_discrete_state: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  context_->get_mutable_discrete_state_vector();  // Return value ignored.
  VerifyNotifications("get_mutable_discrete_state_vector: x",
                      has_discrete_state(),
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_discrete_state_vector: xd",
                      has_discrete_state(), SystemBase::xd_ticket(),
                      &xd_before);
  VerifyNotifications("get_mutable_discrete_state_vector: xc", {},  // None.
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_discrete_state_vector: xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  context_->get_mutable_discrete_state(0);  // Return value ignored.
  VerifyNotifications("get_mutable_discrete_state(0): x", has_discrete_state(),
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_discrete_state(0): xd", has_discrete_state(),
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("get_mutable_discrete_state(0): xc", {},  // None.
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_discrete_state(0): xa", {},  // None.
                      SystemBase::xa_ticket(), &xa_before);

  // Changing abstract state should affect only x and xa, and those should only
  // get notified for systems that actually have discrete variables.

  context_->get_mutable_abstract_state();  // Return value ignored.
  VerifyNotifications("get_mutable_abstract_state: x", has_abstract_state(),
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_abstract_state: xa", has_abstract_state(),
                      SystemBase::xa_ticket(), &xa_before);
  VerifyNotifications("get_mutable_abstract_state: xc", {},  // None.
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_abstract_state: xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);

  context_->get_mutable_abstract_state<int>(0);  // Return value ignored.
  VerifyNotifications("get_mutable_abstract_state(0): x", has_abstract_state(),
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("get_mutable_abstract_state(0): xa", has_abstract_state(),
                      SystemBase::xa_ticket(), &xa_before);
  VerifyNotifications("get_mutable_abstract_state(0): xc", {},  // None.
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("get_mutable_abstract_state(0): xd", {},  // None.
                      SystemBase::xd_ticket(), &xd_before);
}

TEST_F(DiagramContextTest, MutableParameterNotifications) {
  auto p_before =
      SaveNotifications(SystemBase::all_parameters_ticket());
  auto pn_before = SaveNotifications(SystemBase::pn_ticket());
  auto pa_before = SaveNotifications(SystemBase::pa_ticket());

  // Changing the whole set of parameters should affect all subcontexts that
  // have any parameters.
  context_->get_mutable_parameters();  // Return value ignored.
  VerifyNotifications("get_mutable_parameters: p", has_parameter(),
                      SystemBase::all_parameters_ticket(), &p_before);
  VerifyNotifications("get_mutable_parameters: pn", has_numeric_parameter(),
                      SystemBase::pn_ticket(), &pn_before);
  VerifyNotifications("get_mutable_parameters: pa", has_abstract_parameter(),
                      SystemBase::pa_ticket(), &pa_before);

  // Changing numeric or abstract should affect only subcontexts with that kind
  // of parameter.
  context_->get_mutable_numeric_parameter(0);  // Return value ignored.
  VerifyNotifications("get_mutable_numeric_parameter(0): p",
                      has_numeric_parameter(),
                      SystemBase::all_parameters_ticket(), &p_before);
  VerifyNotifications("get_mutable_numeric_parameter(0): pn",
                      has_numeric_parameter(),
                      SystemBase::pn_ticket(), &pn_before);
  VerifyNotifications("get_mutable_numeric_parameter(0): pa", {},  // None.
                      SystemBase::pa_ticket(), &pa_before);

  context_->get_mutable_abstract_parameter(0);  // Return value ignored.
  VerifyNotifications("get_mutable_abstract_parameter(0): p",
                      has_abstract_parameter(),
                      SystemBase::all_parameters_ticket(), &p_before);
  VerifyNotifications("get_mutable_abstract_parameter(0): pa",
                      has_abstract_parameter(),
                      SystemBase::pa_ticket(), &pa_before);
  VerifyNotifications("get_mutable_abstract_parameter(0): pn", {},  // None.
                      SystemBase::pn_ticket(), &pn_before);
}

// We have a method that copies everything from one context to another. That
// should produce a lot of notifications in the destination context.
TEST_F(DiagramContextTest, MutableEverythingNotifications) {
  auto clone = context_->Clone();
  ASSERT_TRUE(clone != nullptr);

  // Make sure clone's values are different from context's. These numbers are
  // arbitrary as long as they don't match the default context_ values.
  const double new_time = context_->get_time() + 1.;
  const double new_accuracy = 3e-12;
  const Eigen::Vector2d new_xc(0.125, 7.5);
  const double new_xd = -1.;
  const int new_xa = 12345;
  const double new_pn = -2;
  const int new_pa = 101;

  clone->SetTime(new_time);
  clone->SetAccuracy(new_accuracy);
  clone->SetContinuousState(new_xc);
  clone->get_mutable_discrete_state(0)[0] = new_xd;
  clone->get_mutable_abstract_state<int>(0) = new_xa;
  clone->get_mutable_numeric_parameter(0).SetAtIndex(0, new_pn);
  clone->get_mutable_abstract_parameter(0).set_value<int>(new_pa);

  auto t_before = SaveNotifications(SystemBase::time_ticket());
  auto a_before = SaveNotifications(SystemBase::accuracy_ticket());
  auto p_before =
      SaveNotifications(SystemBase::all_parameters_ticket());
  auto pn_before = SaveNotifications(SystemBase::pn_ticket());
  auto pa_before = SaveNotifications(SystemBase::pa_ticket());
  auto x_before = SaveNotifications(SystemBase::all_state_ticket());
  auto xc_before = SaveNotifications(SystemBase::xc_ticket());
  auto xd_before = SaveNotifications(SystemBase::xd_ticket());
  auto xa_before = SaveNotifications(SystemBase::xa_ticket());

  // This is the method under test.
  context_->SetTimeStateAndParametersFrom(*clone);

  // First make sure that all the values got passed through.
  VerifyTimeValue(new_time);
  VerifyAccuracyValue(new_accuracy);
  VerifyContinuousStateValue(new_xc);
  // Check that non-continuous states and parameter got set in diagram and
  // subcontext that has the resource.
  EXPECT_EQ(context_->get_discrete_state_vector()[0], new_xd);
  EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(4))  // discrete state
                .get_discrete_state_vector()[0],
            new_xd);
  EXPECT_EQ(context_->get_abstract_state<int>(0), new_xa);
  EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(5))  // abstract state
                .get_abstract_state<int>(0),
            new_xa);
  EXPECT_EQ(context_->get_numeric_parameter(0)[0], new_pn);
  EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(6))  // numeric param.
                .get_numeric_parameter(0)[0],
            new_pn);
  EXPECT_EQ(context_->get_abstract_parameter(0).get_value<int>(), new_pa);
  EXPECT_EQ(context_->GetSubsystemContext(SubsystemIndex(7))  // abstract param.
                .get_abstract_parameter(0).get_value<int>(),
            new_pa);

  VerifyNotifications("SetTimeStateAndParametersFrom: t",
                      SystemBase::time_ticket(), &t_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: a",
                      SystemBase::accuracy_ticket(), &a_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: p", has_parameter(),
                      SystemBase::all_parameters_ticket(),
                      &p_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: pn",
                      has_numeric_parameter(),
                      SystemBase::pn_ticket(), &pn_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: pa",
                      has_abstract_parameter(),
                      SystemBase::pa_ticket(), &pa_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: x",
                      SystemBase::all_state_ticket(), &x_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: xc",
                      SystemBase::xc_ticket(), &xc_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: xd", has_discrete_state(),
                      SystemBase::xd_ticket(), &xd_before);
  VerifyNotifications("SetTimeStateAndParametersFrom: xa", has_abstract_state(),
                      SystemBase::xa_ticket(), &xa_before);
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(DiagramContextTest, State) {
  // Each integrator has a single continuous state variable.
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(2, xc.size());
  EXPECT_EQ(0, xc.get_generalized_position().size());
  EXPECT_EQ(0, xc.get_generalized_velocity().size());
  EXPECT_EQ(2, xc.get_misc_continuous_state().size());

  // The discrete state vector has length 1.
  DiscreteValues<double>& xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(1, xd.num_groups());
  EXPECT_EQ(1, xd.get_vector(0).size());

  // Changes to the diagram state write through to constituent system states.
  // - Continuous
  ContinuousState<double>& integrator0_xc =
      context_->GetMutableSubsystemContext(SubsystemIndex(2))
          .get_mutable_continuous_state();
  ContinuousState<double>& integrator1_xc =
      context_->GetMutableSubsystemContext(SubsystemIndex(3))
          .get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc.get_vector()[0]);
  EXPECT_EQ(43.0, integrator1_xc.get_vector()[0]);
  // - Discrete
  DiscreteValues<double>& discrete_xd =
      context_->GetMutableSubsystemContext(SubsystemIndex(4))
          .get_mutable_discrete_state();
  EXPECT_EQ(44.0, discrete_xd.get_vector(0)[0]);

  // Changes to constituent system states appear in the diagram state.
  // - Continuous
  integrator1_xc.get_mutable_vector()[0] = 1000.0;
  EXPECT_EQ(1000.0, xc.get_vector()[1]);
  // - Discrete
  discrete_xd.get_mutable_vector(0)[0] = 1001.0;
  EXPECT_EQ(1001.0, xd.get_vector(0)[0]);
}

// Tests that the pointers to substates in the DiagramState are equal to the
// substates in the subsystem contexts.
TEST_F(DiagramContextTest, DiagramState) {
  auto diagram_state = dynamic_cast<DiagramState<double>*>(
      &context_->get_mutable_state());
  ASSERT_NE(nullptr, diagram_state);
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    EXPECT_EQ(&context_->GetMutableSubsystemContext(i).get_mutable_state(),
              &diagram_state->get_mutable_substate(i));
  }
}

// Tests that no exception is thrown when connecting a valid source
// and destination port.
TEST_F(DiagramContextTest, ConnectValid) {
  DRAKE_EXPECT_NO_THROW(context_->SubscribeInputPortToOutputPort(
      {SubsystemIndex(0) /* adder0_ */, OutputPortIndex(0)},
      {SubsystemIndex(1) /* adder1_ */, InputPortIndex(1)}));
}

// Tests that input ports can be assigned to the DiagramContext and then
// retrieved.
TEST_F(DiagramContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->num_input_ports());
  AttachInputPorts();
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
  EXPECT_EQ(256, ReadVectorInputPort(*context_, 1)->get_value()[0]);
}

TEST_F(DiagramContextTest, ToString) {
  const std::string str = context_->to_string();
  EXPECT_THAT(str, ::testing::HasSubstr("integrator0"));
  EXPECT_THAT(str, ::testing::HasSubstr("integrator1"));
  EXPECT_THAT(str, ::testing::HasSubstr("discrete_state_system"));
  EXPECT_THAT(str, ::testing::HasSubstr("abstract_state_system"));
  EXPECT_THAT(str, ::testing::HasSubstr("system_with_numeric_parameters"));
  EXPECT_THAT(str, ::testing::HasSubstr("system_with_abstract_parameters"));

  // Adders named adder0 and adder1 are part of the diagram, but don't have
  // any context that is useful to print, so are excluded.
  EXPECT_THAT(str, ::testing::Not(::testing::HasSubstr("adder0")));
  EXPECT_THAT(str, ::testing::Not(::testing::HasSubstr("adder1")));
}

// Test that start_next_change_event() returns a sequentially increasing
// number regardless of from where in the DiagramContext tree the request
// is initiated. Also, it should continue counting up after cloning.
TEST_F(DiagramContextTest, NextChangeEventNumber) {
  const int64_t first = context_->start_new_change_event();

  EXPECT_EQ(context_->start_new_change_event(), first + 1);
  EXPECT_EQ(context_->start_new_change_event(), first + 2);

  // Obtain a subcontext and verify we still count up.
  Context<double>& discrete_context =
      context_->GetMutableSubsystemContext(SubsystemIndex(4));
  EXPECT_EQ(discrete_context.start_new_change_event(), first + 3);

  // Now clone the context and make sure we're still counting up.
  auto clone = dynamic_pointer_cast<DiagramContext<double>>(context_->Clone());
  EXPECT_EQ(clone->start_new_change_event(), first + 4);
  EXPECT_EQ(context_->start_new_change_event(), first + 4);  // Sanity check.

  Context<double>& discrete_clone =
      clone->GetMutableSubsystemContext(SubsystemIndex(4));
  EXPECT_EQ(discrete_clone.start_new_change_event(), first + 5);
}

TEST_F(DiagramContextTest, Clone) {
  context_->SubscribeInputPortToOutputPort(
      {SubsystemIndex(0) /* adder0_ */, OutputPortIndex(0)},
      {SubsystemIndex(1) /* adder1_ */, InputPortIndex(1)});
  AttachInputPorts();

  auto clone = dynamic_pointer_cast<DiagramContext<double>>(context_->Clone());
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the system id was copied.
  EXPECT_TRUE(clone->get_system_id().is_valid());
  EXPECT_EQ(clone->get_system_id(), context_->get_system_id());
  const ContinuousState<double>& xc = clone->get_continuous_state();
  EXPECT_TRUE(xc.get_system_id().is_valid());
  EXPECT_EQ(xc.get_system_id(), context_->get_system_id());
  const DiscreteValues<double>& xd = clone->get_discrete_state();
  EXPECT_TRUE(xd.get_system_id().is_valid());
  EXPECT_EQ(xd.get_system_id(), context_->get_system_id());

  // Verify that the state has the same value.
  VerifyClonedState(clone->get_state());
  // Verify that the parameters have the same value.
  VerifyClonedParameters(clone->get_parameters());

  // Verify that changes to the state do not write through to the original
  // context.
  clone->get_mutable_continuous_state_vector()[0] = 1024.0;
  EXPECT_EQ(1024.0, clone->get_continuous_state()[0]);
  EXPECT_EQ(42.0, context_->get_continuous_state()[0]);

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(orig_port, clone_port);
    EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                                1e-8, MatrixCompareType::absolute));
  }
}

TEST_F(DiagramContextTest, CloneState) {
  std::unique_ptr<State<double>> state = context_->CloneState();
  // Verify that the state was copied.
  VerifyClonedState(*state);
  // Verify that the underlying type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<DiagramState<double>*>(state.get()));

  // Verify that the system id was copied.
  EXPECT_TRUE(state->get_system_id().is_valid());
  EXPECT_EQ(state->get_system_id(), context_->get_system_id());
  ContinuousState<double>& xc = state->get_mutable_continuous_state();
  EXPECT_TRUE(xc.get_system_id().is_valid());
  EXPECT_EQ(xc.get_system_id(), context_->get_system_id());
  const DiscreteValues<double>& xd = state->get_discrete_state();
  EXPECT_TRUE(xd.get_system_id().is_valid());
  EXPECT_EQ(xd.get_system_id(), context_->get_system_id());

  // Verify that changes to the state do not write through to the original
  // context.
  xc[1] = 1024.0;
  EXPECT_EQ(1024.0, state->get_continuous_state()[1]);
  EXPECT_EQ(43.0, context_->get_continuous_state()[1]);
}

// Verifies that accuracy is set properly during cloning.
TEST_F(DiagramContextTest, CloneAccuracy) {
  // Verify accuracy is not set by default.
  EXPECT_FALSE(context_->get_accuracy());

  // Verify that setting the accuracy is reflected in cloning.
  const double unity = 1.0;
  context_->SetAccuracy(unity);
  std::unique_ptr<Context<double>> clone = context_->Clone();
  EXPECT_EQ(clone->get_accuracy().value(), unity);
}

TEST_F(DiagramContextTest, SubcontextCloneIsError) {
  const auto& subcontext = context_->GetSubsystemContext(SubsystemIndex{0});
  DRAKE_EXPECT_THROWS_MESSAGE(
      subcontext.Clone(), std::logic_error,
      "Context::Clone..: Cannot clone a non-root Context; "
      "this Context was created by 'adder0'.");
}

TEST_F(DiagramContextTest, SubcontextSetTimeStateAndParametersFromIsError) {
  auto clone = dynamic_pointer_cast<DiagramContext<double>>(context_->Clone());
  ASSERT_TRUE(clone != nullptr);
  const auto& source_subcontext =
      context_->GetSubsystemContext(SubsystemIndex{0});
  auto& dest_subcontext = clone->GetMutableSubsystemContext(SubsystemIndex{0});
  DRAKE_EXPECT_THROWS_MESSAGE(
      dest_subcontext.SetTimeStateAndParametersFrom(source_subcontext),
      "SetTimeStateAndParametersFrom\\(\\): Time change allowed only in the "
      "root Context.");
}

TEST_F(DiagramContextTest, SubcontextSetStateAndParametersFrom) {
  auto clone = dynamic_pointer_cast<DiagramContext<double>>(context_->Clone());
  ASSERT_TRUE(clone != nullptr);
  // Set arbitrary time, accuracy, state and parameters to the cloned
  // context so that they are different from the original context.
  const double new_time = context_->get_time() + 1.;
  const double new_accuracy = 3e-12;
  const Eigen::Vector2d new_xc(0.125, 7.5);
  const double new_xd = -1.;
  const int new_xa = 12345;
  const double new_pn = -2;
  const int new_pa = 101;

  clone->SetTime(new_time);
  clone->SetAccuracy(new_accuracy);
  clone->SetContinuousState(new_xc);
  clone->get_mutable_discrete_state(0)[0] = new_xd;
  clone->get_mutable_abstract_state<int>(0) = new_xa;
  clone->get_mutable_numeric_parameter(0).SetAtIndex(0, new_pn);
  clone->get_mutable_abstract_parameter(0).set_value<int>(new_pa);

  // Call the method under test for the subcontexts of context_.
  for (int i = 0; i < kNumSystems; ++i) {
    const auto& source_subcontext =
        context_->GetSubsystemContext(SubsystemIndex{i});
    auto& dest_subcontext =
        clone->GetMutableSubsystemContext(SubsystemIndex{i});
    dest_subcontext.SetStateAndParametersFrom(source_subcontext);
  }

  // The state and the parameters should have been set to be the same
  // as the source context.
  VerifyClonedParameters(clone->get_parameters());
  VerifyClonedState(clone->get_state());

  // Verify time and accuracy did not change for this context and its
  // subcontexts.
  EXPECT_EQ(clone->get_time(), new_time);
  EXPECT_EQ(clone->get_accuracy(), new_accuracy);
  for (SubsystemIndex i(0); i < kNumSystems; ++i) {
    const auto& subcontext = clone->GetSubsystemContext(i);
    EXPECT_EQ(subcontext.get_time(), new_time);
    EXPECT_EQ(subcontext.get_accuracy(), new_accuracy);
  }
}
}  // namespace
}  // namespace systems
}  // namespace drake
