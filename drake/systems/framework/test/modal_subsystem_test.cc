#include "drake/systems/framework/modal_subsystem.h"

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

class AbstractTestSource : public LeafSystem<double> {
 public:
  AbstractTestSource() { this->DeclareOutputPort(kVectorValued, 1); }

  unique_ptr<AbstractState> AllocateAbstractState() const override {
    std::vector<unique_ptr<AbstractValue>> values;
    values.push_back({PackValue<std::string>("I'm an abstract state.")});
    return make_unique<AbstractState>(move(values));
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));

    System<double>::GetMutableOutputVector(output, 0) =
        System<double>::CopyContinuousStateVector(context);
  }
};

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
    abstract_ = builder.template AddSystem<AbstractTestSource>();

    builder.Connect(*integrator_, *zoh_);

    builder.ExportInput(integrator_->get_input_port());
    builder.ExportOutput(zoh_->get_output_port());
    builder.ExportOutput(abstract_->get_output_port(0));

    builder.BuildInto(this);
  }

 private:
  Integrator<double>* integrator_ = nullptr;
  ZeroOrderHold<double>* zoh_ = nullptr;
  AbstractTestSource* abstract_ = nullptr;
};

class ModalSubsystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator_.reset(new Integrator<double>(kSize));
    integrator_->set_name("Alice");

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    const int mode_id = 42;
    mss_.reset(new ModalSubsystem<double>(mode_id, integrator_));
  }

  unique_ptr<ModalSubsystem<double>> mss_;

  shared_ptr<Integrator<double>> integrator_;
};

class ModalSubsystemStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    example_system_.reset(new ContinuousDiscreteAbstractSystem());
    example_system_->set_name("Bob");

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    const int mode_id = 6;
    mss_.reset(new ModalSubsystem<double>(mode_id, example_system_));
  }

  unique_ptr<ModalSubsystem<double>> mss_;

  shared_ptr<ContinuousDiscreteAbstractSystem> example_system_;
};

// Tests that we can correctly select among exported input and output ports
// those that are inputs and outputs of the HA.
TEST_F(ModalSubsystemTest, ModalSubsystemPortIds) {
  EXPECT_EQ(1, mss_->get_num_input_ports());
  EXPECT_EQ(1, mss_->get_num_output_ports());

  EXPECT_EQ(0, mss_->get_input_port_ids()[0]);
  EXPECT_EQ(0, mss_->get_output_port_ids()[0]);

  // Explicitly specify the ports for a non-trivial example via the constructor.
  shared_ptr<System<double>> example_system(
      new ContinuousDiscreteAbstractSystem());
  const ModalSubsystem<double>& mss_new =
      ModalSubsystem<double>(0, example_system, {0}, {1, 0});

  // Verify the number and identities of the ports have passed into the object.
  EXPECT_EQ(1, mss_new.get_num_input_ports());
  EXPECT_EQ(2, mss_new.get_num_output_ports());

  EXPECT_EQ(0, mss_new.get_input_port_ids()[0]);
  EXPECT_EQ(1, mss_new.get_output_port_ids()[0]);
  EXPECT_EQ(0, mss_new.get_output_port_ids()[1]);
}

// Tests the ability to specify symbolic expressions and evaluate them.
TEST_F(ModalSubsystemStateTest, ModalSubsystemSymbolicExpressions) {
  // Fetch the symbolic variables and mutable expressions for this context.
  symbolic::Variable xc = mss_->get_symbolic_continuous_states()[0];
  symbolic::Variable xd = mss_->get_symbolic_discrete_states_at(0)[0];
  std::vector<symbolic::Expression>* example_invariant =
      mss_->get_mutable_invariant();
  std::vector<symbolic::Expression>* example_ic =
      mss_->get_mutable_initial_conditions();

  // Define the symbols @p p (continuous-state) and @p q (discrete-state).
  const symbolic::Expression p{xc};
  const symbolic::Expression q{xd};

  // Check that the keys are indeed labeled "xc0" and "xd0".
  EXPECT_EQ("xc0", p.to_string());
  EXPECT_EQ("xd0", q.to_string());

  // Create invariants and initial conditions in terms of the symbols @p p and
  // @p q.
  (*example_invariant).push_back({pow(p, 2.) - cos(p) + tanh(q)});
  (*example_ic).push_back({pow(p, 3.)});
  (*example_ic).push_back({-p + 3.});
  (*example_ic).push_back({q});

  // Evaluate the expressions for a given assignment.
  symbolic::Environment x_env{{xc, 3.}, {xd, -2.}};
  EXPECT_EQ(pow(3., 2.) - cos(3.) + tanh(-2.),
            (*example_invariant)[0].Evaluate(x_env));
  EXPECT_EQ(pow(3., 3.), (*example_ic)[0].Evaluate(x_env));
  EXPECT_EQ(-3. + 3., (*example_ic)[1].Evaluate(x_env));
  EXPECT_EQ(-2., (*example_ic)[2].Evaluate(x_env));
}

// Tests the ability to create a clone of a ModalSubsystem.
TEST_F(ModalSubsystemStateTest, CloneModalSubsystem) {
  // Set fictitious invariants and initial conditions.
  const symbolic::Expression y{symbolic::Variable{"x"}};
  symbolic::Expression expression{y};
  (*mss_->get_mutable_invariant()).push_back(y);
  (*mss_->get_mutable_initial_conditions()).push_back(y);

  // Retrieve a clone.
  unique_ptr<ModalSubsystem<double>> mss_new = mss_->Clone();

  // Verify that the data survives the clone.
  EXPECT_EQ(6, mss_new->get_mode_id());
  EXPECT_EQ("Bob", mss_new->get_system()->get_name());
  EXPECT_EQ(1, mss_new->get_invariant().size());
  EXPECT_EQ(1, mss_new->get_initial_conditions().size());
  EXPECT_EQ(1, mss_new->get_num_input_ports());
  EXPECT_EQ(2, mss_new->get_num_output_ports());
}

}  // namespace
}  // namespace systems
}  // namespace drake
