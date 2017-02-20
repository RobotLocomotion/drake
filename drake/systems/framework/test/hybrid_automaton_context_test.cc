#include "drake/systems/framework/hybrid_automaton_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;

constexpr int kSize{1};
constexpr double kZohSamplingPeriod{0.1};

// A system that is an amalgamation of continuous, discrete, and abstract
// states. Specifically, it is a diagram of an Integrator in series with a
// ZeroOrderHold element, in parallel with an AbstractTestSource system.
class ContinuousDiscreteAbstractSystem : public Diagram<double> {
 public:
  ContinuousDiscreteAbstractSystem() {
    DiagramBuilder<double> builder;

    integrator_ = builder.template AddSystem<Integrator<double>>(kSize);
    zoh_ = builder.template AddSystem<ZeroOrderHold<double>>(kZohSamplingPeriod,
                                                             kSize);
    // abstract_ = builder.template AddSystem<AbstractTestSource>();
    std::unique_ptr<AbstractValue>
        value(new Value<std::string>("I'm an abstract state."));
    abstract_ = builder.template
        AddSystem<ConstantValueSource<double>>(std::move(value));

    builder.Connect(integrator_->get_output_port(0), zoh_->get_input_port(0));

    builder.ExportInput(integrator_->get_input_port(0));
    builder.ExportOutput(zoh_->get_output_port(0));
    builder.ExportOutput(abstract_->get_output_port(0));

    builder.BuildInto(this);
  }

 private:
  Integrator<double>* integrator_ = nullptr;
  ZeroOrderHold<double>* zoh_ = nullptr;
  ConstantValueSource<double>* abstract_ = nullptr;
};

class ModalSubsystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator_.reset(new Integrator<double>(kSize));
    integrator_->set_name("Alice");

    // Instantiate a new modal subsystem. Note that, by default, the
    // ModalSubsystem mirrors the input and output port ids of the Integrator
    // system (which, in this case, is SISO).
    mss_.reset(new ModalSubsystem<double>(mode_id_, integrator_));
  }

  const int mode_id_ = 42;
  unique_ptr<ModalSubsystem<double>> mss_;
  shared_ptr<Integrator<double>> integrator_;
};

// Tests that we can correctly select among exported input and output ports
// those that are inputs and outputs of the HA.
TEST_F(ModalSubsystemTest, Attributes) {
  EXPECT_EQ(mode_id_, mss_->get_mode_id());
  EXPECT_EQ("Alice", mss_->get_system()->get_name());

  EXPECT_EQ(1, mss_->get_num_input_ports());
  EXPECT_EQ(1, mss_->get_num_output_ports());

  EXPECT_EQ(0, mss_->get_input_port_ids()[0]);
  EXPECT_EQ(0, mss_->get_output_port_ids()[0]);
}

// Tests the validity of a ModalSubsystem constructed with explicit ports.
GTEST_TEST(ModalSubsystemPortTest, ExplicitPortIds) {
  shared_ptr<System<double>> example_system(
      new ContinuousDiscreteAbstractSystem());
  const ModalSubsystem<double>& mss =
      ModalSubsystem<double>(0, example_system, {0}, {1, 0});

  // Verify the number and identities of the ports have passed into the object.
  EXPECT_EQ(1, mss.get_num_input_ports());
  EXPECT_EQ(2, mss.get_num_output_ports());

  EXPECT_EQ(0, mss.get_input_port_ids()[0]);
  EXPECT_EQ(1, mss.get_output_port_ids()[0]);
  EXPECT_EQ(0, mss.get_output_port_ids()[1]);
}

// Tests that a ModalSubsystem cannot be constructed with invalid ports.
GTEST_TEST(ModalSubsystemPortTest, InvalidPortIds) {
  shared_ptr<System<double>> example_system(
      new ContinuousDiscreteAbstractSystem());

  // Throws if any of the port ids are invalid.
  EXPECT_THROW(ModalSubsystem<double>(0, example_system,
    {example_system->get_num_output_ports()}, {0}), std::runtime_error);
  EXPECT_THROW(ModalSubsystem<double>(0, example_system, {0},
    {example_system->get_num_output_ports()}), std::runtime_error);
}

class HybridAutomatonContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    example_system_.reset(new ContinuousDiscreteAbstractSystem());
    example_system_->set_name("Bob");

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    mss_.reset(new ModalSubsystem<double>(mode_id_, example_system_));
  }

  const int mode_id_ = 6;
  unique_ptr<ModalSubsystem<double>> mss_;
  shared_ptr<ContinuousDiscreteAbstractSystem> example_system_;
};

// Tests the ability to specify symbolic expressions and evaluate them.
TEST_F(HybridAutomatonContextTest, ModalSubsystemSymbolicExpressions) {
  // Fetch the symbolic variables and mutable expressions for this context.
  std::vector<symbolic::Variable> xcs = mss_->get_symbolic_continuous_states();
  std::vector<symbolic::Variable> xds =
      mss_->get_symbolic_discrete_states_at(0);
  std::vector<symbolic::Expression>* example_invariant =
      mss_->get_mutable_invariant();
  std::vector<symbolic::Expression>* example_ic =
      mss_->get_mutable_initial_conditions();

  // Define symbols for the continuous state (@p p) and discrete state (@p q)
  // upon which to construct symbolic expressions.
  const symbolic::Expression p{xcs[0]};
  const symbolic::Expression q{xds[0]};

  // Check that the generated keys are indeed correctly labeled "xc0" and "xd0".
  EXPECT_EQ("xc0", p.to_string());
  EXPECT_EQ("xd0", q.to_string());

  // Create examples of invariants and initial conditions in terms of the
  // symbols @p p and @p q.
  // N.B. This is a mock-up of how a user would create these expressions.
  (*example_invariant).push_back({pow(p, 2.) - cos(p) + tanh(q)});
  (*example_ic).push_back({pow(p, 3.)});
  (*example_ic).push_back({-p + 3.});
  (*example_ic).push_back({q});

  // Check that the symbolic variables obtained from ModalSubsystem are
  // compatible with the constructed invariants and initial conditions.
  // N.B. This is a mock-up of code that will exist in HybridAutomatonContext to
  // process the expressions.
  symbolic::Environment x_env{{xcs[0], 3.}, {xds[0], -2.}};
  EXPECT_NO_THROW((*example_invariant)[0].Evaluate(x_env));
  EXPECT_NO_THROW((*example_ic)[0].Evaluate(x_env));
  EXPECT_NO_THROW((*example_ic)[1].Evaluate(x_env));
  EXPECT_NO_THROW((*example_ic)[2].Evaluate(x_env));
}

// Tests the ability to create a clone of a ModalSubsystem.
TEST_F(HybridAutomatonContextTest, CloneModalSubsystem) {
  // Set fictitious invariants and initial conditions.
  const symbolic::Expression y{symbolic::Variable{"x"}};
  symbolic::Expression expression{y};
  (*mss_->get_mutable_invariant()).push_back(y);
  (*mss_->get_mutable_initial_conditions()).push_back(y);

  // Retrieve a clone.
  unique_ptr<ModalSubsystem<double>> mss_new = mss_->Clone();

  // Verify that the data survives the clone.
  EXPECT_EQ(mode_id_, mss_new->get_mode_id());
  EXPECT_EQ("Bob", mss_new->get_system()->get_name());
  EXPECT_EQ(1, mss_new->get_invariant().size());
  EXPECT_EQ(1, mss_new->get_initial_conditions().size());
  EXPECT_EQ(1, mss_new->get_num_input_ports());
  EXPECT_EQ(2, mss_new->get_num_output_ports());
}

// TODO(jadecastro): Include unit tests for HybridAutomatonContext.

}  // namespace
}  // namespace systems
}  // namespace drake
