#include "drake/systems/framework/vector_system.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

using Eigen::VectorXd;

// For all of the output / xdot / x[n+1] functions, the result is set to be
// either (1) equal to the input, when there is no declared state or (2) equal
// to the input plus the state, when there is declared state.  Unlike most
// Systems, this System allows test code to configure its ports and state even
// beyond what the constructor does by default.  For all of the Siso overrides,
// a call count is retained for tests to examine.
class TestVectorSystem : public VectorSystem<double> {
 public:
  static constexpr int kSize = 2;

  TestVectorSystem() : VectorSystem<double>(kSize, kSize) {}

  // Let test code abuse these by making them public.
  using VectorSystem<double>::DeclareContinuousState;
  using VectorSystem<double>::DeclareDiscreteState;
  using VectorSystem<double>::DeclareAbstractInputPort;
  using VectorSystem<double>::DeclareAbstractOutputPort;
  using VectorSystem<double>::EvalVectorInput;
  using VectorSystem<double>::GetVectorState;

  // VectorSystem override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorOutput(const Context<double>& context,
                          const Eigen::VectorBlock<const VectorXd>& input,
                          const Eigen::VectorBlock<const VectorXd>& state,
                          Eigen::VectorBlock<VectorXd>* output) const override {
    ++output_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *output = input;
    } else {
      *output = input + state;
    }
  }

  // VectorSystem override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorTimeDerivatives(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorXd>& input,
      const Eigen::VectorBlock<const VectorXd>& state,
      Eigen::VectorBlock<VectorXd>* derivatives) const override {
    ++time_derivatives_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *derivatives = input;
    } else {
      *derivatives = input + state;
    }
  }

  // VectorSystem override.
  // N.B. This method signature might be used by many downstream projects.
  // Change it only with good reason and with a deprecation period first.
  void DoCalcVectorDiscreteVariableUpdates(
      const Context<double>& context,
      const Eigen::VectorBlock<const VectorXd>& input,
      const Eigen::VectorBlock<const VectorXd>& state,
      Eigen::VectorBlock<VectorXd>* next_state) const override {
    ++discrete_variable_updates_count_;
    last_context_ = &context;
    if (state.size() == 0) {
      *next_state = input;
    } else {
      *next_state = input + state;
    }
  }

  // LeafSystem override.
  std::unique_ptr<DiscreteValues<double>> AllocateDiscreteState()
      const override {
    return prototype_discrete_state_->Clone();
  }

  // LeafSystem override.
  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
    return prototype_abstract_state_->Clone();
  }

  // Use the given number of discrete state groups for AllocateDiscreteState;
  // when this has not been called, there will be no discrete state.
  void set_prototype_discrete_state_count(int count) {
    std::vector<std::unique_ptr<BasicVector<double>>> vec;
    for (int i = 0; i < count; ++i) {
      vec.emplace_back(std::make_unique<BasicVector<double>>(kSize));
    }
    prototype_discrete_state_ =
        std::make_unique<DiscreteValues<double>>(std::move(vec));
  }

  // Use a single Value<S>(value) for AllocateAbstractState; when this has not
  // been called, there will be no abstract state.
  template <typename S>
  void set_prototype_abstract_state(const S& value) {
    std::vector<std::unique_ptr<AbstractValue>> vec;
    vec.emplace_back(std::move(std::make_unique<Value<S>>(value)));
    prototype_abstract_state_ =
        std::make_unique<AbstractValues>(std::move(vec));
  }

  // Testing accessors.
  const Context<double>* get_last_context() const { return last_context_; }
  int get_output_count() const { return output_count_; }
  int get_time_derivatives_count() const { return time_derivatives_count_; }
  int get_discrete_variable_updates_count() const {
    return discrete_variable_updates_count_;
  }

 private:
  std::unique_ptr<DiscreteValues<double>> prototype_discrete_state_{
      std::make_unique<DiscreteValues<double>>()};
  std::unique_ptr<AbstractValues> prototype_abstract_state_{
      std::make_unique<AbstractValues>()};
  mutable const Context<double>* last_context_{nullptr};
  mutable int output_count_{0};
  mutable int time_derivatives_count_{0};
  mutable int discrete_variable_updates_count_{0};
};
constexpr int TestVectorSystem::kSize;

class VectorSystemTest : public ::testing::Test {
  // Not yet needed, but placeholder for future common code.
};

// The system has an appropriate topology.
TEST_F(VectorSystemTest, Topology) {
  // The device-under-test ("dut").
  TestVectorSystem dut;

  // One input port.
  ASSERT_EQ(dut.get_num_input_ports(), 1);
  const InputPortDescriptor<double>& descriptor_in = dut.get_input_port();
  EXPECT_EQ(descriptor_in.get_data_type(), kVectorValued);
  EXPECT_EQ(descriptor_in.size(), TestVectorSystem::kSize);

  // One output port.
  ASSERT_EQ(dut.get_num_output_ports(), 1);
  const OutputPort<double>& output_port = dut.get_output_port();
  EXPECT_EQ(output_port.get_data_type(), kVectorValued);
  EXPECT_EQ(output_port.size(), TestVectorSystem::kSize);

  // No state by default ...
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(context->get_num_input_ports(), 1);
  EXPECT_TRUE(context->is_stateless());

  // ... but subclasses may declare it.
  dut.DeclareContinuousState(TestVectorSystem::kSize);
  context = dut.CreateDefaultContext();
  EXPECT_FALSE(context->is_stateless());
}

// Subclasses that violate the VectorSystem premise fail-fast.
TEST_F(VectorSystemTest, TopologyFailFast) {
  {  // A second input.
    TestVectorSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.DeclareAbstractInputPort();
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  {  // A second output.
    TestVectorSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.DeclareAbstractOutputPort(nullptr, nullptr);  // No alloc or calc.
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  {  // Abstract state.
    // TODO(jwnimmer-tri) Conceptually, allowing abstract state could be
    // consistent with the VectorSystem contract.  However, if we widened the
    // VectorSystem contract to support that, then we would have add code to
    // fail-fast if the subclass neglected to override the unrestricted
    // updates API.  That doesn't seem worth it yet, but if we ever want
    // abstract state in VectorSystem, then it would be okay to write the
    // code and tests to support it.
    TestVectorSystem dut;
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_abstract_state<double>(1.0);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  {  // More than one discrete state group.
    TestVectorSystem dut;
    dut.set_prototype_discrete_state_count(1);
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_discrete_state_count(2);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }

  {  // Both continuous and discrete state.
    TestVectorSystem dut;
    dut.DeclareContinuousState(1);
    EXPECT_NO_THROW(dut.CreateDefaultContext());
    dut.set_prototype_discrete_state_count(1);
    EXPECT_THROW(dut.CreateDefaultContext(), std::exception);
  }
}

// Forwarding of CalcOutput with no state.
TEST_F(VectorSystemTest, OutputStateless) {
  TestVectorSystem dut;
  auto context = dut.CreateDefaultContext();
  auto& output_port = dut.get_output_port();
  std::unique_ptr<AbstractValue> output = output_port.Allocate();
  context->FixInputPort(0, {1.0, 2.0});
  output_port.Calc(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  const auto& basic = output->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(basic.GetAtIndex(0), 1.0);
  EXPECT_EQ(basic.GetAtIndex(1), 2.0);

  const auto& input = dut.EvalVectorInput(*context);
  EXPECT_EQ(input.size(), 2);
  EXPECT_EQ(input[0], 1.0);
  EXPECT_EQ(input[1], 2.0);
  const auto& state = dut.GetVectorState(*context);
  EXPECT_EQ(state.size(), 0);
}

// Forwarding of CalcOutput with continuous state.
TEST_F(VectorSystemTest, OutputContinuous) {
  TestVectorSystem dut;
  dut.DeclareContinuousState(TestVectorSystem::kSize);
  auto context = dut.CreateDefaultContext();
  auto& output_port = dut.get_output_port();
  std::unique_ptr<AbstractValue> output = output_port.Allocate();
  context->FixInputPort(0, {1.0, 2.0});
  context->get_mutable_continuous_state_vector().SetFromVector(
      Eigen::Vector2d::Ones());
  output_port.Calc(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  const auto& basic = output->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(basic.GetAtIndex(0), 2.0);
  EXPECT_EQ(basic.GetAtIndex(1), 3.0);

  const auto& state = dut.GetVectorState(*context);
  EXPECT_EQ(state.size(), 2);
  EXPECT_EQ(state[0], 1.0);
  EXPECT_EQ(state[1], 1.0);
}

// Forwarding of CalcOutput with discrete state.
TEST_F(VectorSystemTest, OutputDiscrete) {
  TestVectorSystem dut;
  dut.set_prototype_discrete_state_count(1);
  auto context = dut.CreateDefaultContext();
  auto& output_port = dut.get_output_port();
  std::unique_ptr<AbstractValue> output = output_port.Allocate();
  context->FixInputPort(0, {1.0, 2.0});
  context->get_mutable_discrete_state(0).SetFromVector(
      Eigen::Vector2d::Ones());
  output_port.Calc(*context, output.get());
  EXPECT_EQ(dut.get_output_count(), 1);
  EXPECT_EQ(dut.get_last_context(), context.get());
  const auto& basic = output->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(basic.GetAtIndex(0), 2.0);
  EXPECT_EQ(basic.GetAtIndex(1), 3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);

  const auto& state = dut.GetVectorState(*context);
  EXPECT_EQ(state.size(), 2);
  EXPECT_EQ(state[0], 1.0);
  EXPECT_EQ(state[1], 1.0);
}

// Forwarding of CalcTimeDerivatives works.
TEST_F(VectorSystemTest, TimeDerivatives) {
  TestVectorSystem dut;
  auto context = dut.CreateDefaultContext();
  std::unique_ptr<ContinuousState<double>> derivatives =
      dut.AllocateTimeDerivatives();

  // There's no state declared yet, so the VectorSystem base shouldn't call our
  // DUT.
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);

  // Now we have state, so the VectorSystem base should call our DUT.
  dut.DeclareContinuousState(TestVectorSystem::kSize);
  context = dut.CreateDefaultContext();
  context->FixInputPort(0, {1.0, 2.0});
  context->get_mutable_continuous_state_vector().SetFromVector(
      Eigen::Vector2d::Ones());
  derivatives = dut.AllocateTimeDerivatives();
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(dut.get_time_derivatives_count(), 1);
  EXPECT_EQ(derivatives->get_vector().GetAtIndex(0), 2.0);
  EXPECT_EQ(derivatives->get_vector().GetAtIndex(1), 3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);
  EXPECT_EQ(dut.get_output_count(), 0);
}

// Forwarding of CalcDiscreteVariableUpdates works.
TEST_F(VectorSystemTest, DiscreteVariableUpdates) {
  TestVectorSystem dut;
  auto context = dut.CreateDefaultContext();
  auto discrete_updates = dut.AllocateDiscreteVariables();

  // There's no state declared yet, so the VectorSystem base shouldn't call our
  // DUT.
  dut.CalcDiscreteVariableUpdates(*context, discrete_updates.get());
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 0);

  // Now we have state, so the VectorSystem base should call our DUT.
  dut.set_prototype_discrete_state_count(1);
  context = dut.CreateDefaultContext();
  context->FixInputPort(0, {1.0, 2.0});
  context->get_mutable_discrete_state(0).SetFromVector(
      Eigen::Vector2d::Ones());
  discrete_updates = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(*context, discrete_updates.get());
  EXPECT_EQ(dut.get_last_context(), context.get());
  EXPECT_EQ(dut.get_discrete_variable_updates_count(), 1);
  EXPECT_EQ(discrete_updates->get_vector(0).GetAtIndex(0), 2.0);
  EXPECT_EQ(discrete_updates->get_vector(0).GetAtIndex(1), 3.0);

  // Nothing else weird happened.
  EXPECT_EQ(dut.get_time_derivatives_count(), 0);
  EXPECT_EQ(dut.get_output_count(), 0);
}

class NoFeedthroughContinuousTimeSystem : public VectorSystem<double> {
 public:
  NoFeedthroughContinuousTimeSystem() : VectorSystem<double>(1, 1) {
    this->DeclareContinuousState(1);
  }

  optional<bool> DoHasDirectFeedthrough(int, int) const final { return false; }

 private:
  void DoCalcVectorOutput(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* output) const override {
    EXPECT_EQ(input.size(), 0);
    *output = state;
  }
};

class NoInputContinuousTimeSystem : public VectorSystem<double> {
 public:
  NoInputContinuousTimeSystem() : VectorSystem<double>(0, 1) {
    this->DeclareContinuousState(1);
  }

  // Let test code abuse this by making it public.
  using VectorSystem<double>::EvalVectorInput;

 private:
  virtual void DoCalcVectorTimeDerivatives(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* derivatives) const {
    *derivatives = -state;
  }

  virtual void DoCalcVectorOutput(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* output) const {
    *output = state;
  }
};

// Input is not provided to DoCalcOutput when non-direct-feedthrough.
TEST_F(VectorSystemTest, NoFeedthroughContinuousTimeSystemTest) {
  NoFeedthroughContinuousTimeSystem dut;

  // The non-connected input is never evaluated.
  auto context = dut.CreateDefaultContext();
  auto output = dut.get_output_port().Allocate();
  dut.get_output_port().Calc(*context, output.get());
  EXPECT_EQ(output->GetValueOrThrow<BasicVector<double>>().GetAtIndex(0), 0.0);
}

// Derivatives and Output methods still work when input size is zero.
TEST_F(VectorSystemTest, NoInputContinuousTimeSystemTest) {
  NoInputContinuousTimeSystem dut;
  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state().SetFromVector(Vector1d::Ones());

  // Ensure that time derivatives get calculated correctly.
  std::unique_ptr<ContinuousState<double>> derivatives =
      dut.AllocateTimeDerivatives();
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_EQ(derivatives->get_vector().GetAtIndex(0), -1.0);

  auto output = dut.get_output_port().Allocate();
  dut.get_output_port().Calc(*context, output.get());
  EXPECT_EQ(output->GetValueOrThrow<BasicVector<double>>().GetAtIndex(0), 1.0);

  const auto& input = dut.EvalVectorInput(*context);
  EXPECT_EQ(input.size(), 0);
}

class NoInputNoOutputDiscreteTimeSystem : public VectorSystem<double> {
 public:
  NoInputNoOutputDiscreteTimeSystem() : VectorSystem<double>(0, 0) {
    this->DeclarePeriodicDiscreteUpdate(1.0);
    this->DeclareDiscreteState(1);
  }

 private:
  // x[n+1] = x[n]^3
  virtual void DoCalcVectorDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& input,
      const Eigen::VectorBlock<const Eigen::VectorXd>& state,
      Eigen::VectorBlock<Eigen::VectorXd>* next_state) const {
    (*next_state)[0] = std::pow(state[0], 3.0);
  }
};

// Discrete updates still work when input size is zero.
// No output ports are created when the output size is zero.
TEST_F(VectorSystemTest, NoInputNoOutputDiscreteTimeSystemTest) {
  NoInputNoOutputDiscreteTimeSystem dut;
  auto context = dut.CreateDefaultContext();
  context->get_mutable_discrete_state().get_mutable_vector().SetFromVector(
      Vector1d::Constant(2.0));

  auto discrete_updates = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(*context, discrete_updates.get());
  EXPECT_EQ(discrete_updates->get_vector(0).GetAtIndex(0), 8.0);

  EXPECT_EQ(dut.get_num_output_ports(), 0);
}

/// A system that can use any scalar type: AutoDiff, symbolic form, etc.
template <typename T>
class OpenScalarTypeSystem : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OpenScalarTypeSystem);

  explicit OpenScalarTypeSystem(int some_number)
      : VectorSystem<T>(SystemTypeTag<systems::OpenScalarTypeSystem>{}, 1, 1),
        some_number_(some_number) {}

  template <typename U>
  explicit OpenScalarTypeSystem(const OpenScalarTypeSystem<U>& other)
      : OpenScalarTypeSystem<T>(other.some_number_) {}

  int get_some_number() const { return some_number_; }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class OpenScalarTypeSystem;

  const int some_number_{};
};

TEST_F(VectorSystemTest, ToAutoDiffXdTest) {
  // The member field remains intact.
  const OpenScalarTypeSystem<double> dut{22};
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [](const auto& converted) {
    EXPECT_EQ(converted.get_some_number(), 22);
  }));
}

TEST_F(VectorSystemTest, ToSymbolicTest) {
  // The member field remains intact.
  const OpenScalarTypeSystem<double> dut{22};
  EXPECT_TRUE(is_symbolic_convertible(dut, [](const auto& converted) {
    EXPECT_EQ(converted.get_some_number(), 22);
  }));
}

/// A system that passes a custom SystemScalarConverter object to VectorSystem,
/// rather than a SystemTypeTag.
template <typename T>
class DirectScalarTypeConversionSystem : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectScalarTypeConversionSystem);

  DirectScalarTypeConversionSystem()
      : VectorSystem<T>(MakeConverter(), 0, 0) {
    // This will fail at compile-time if T is ever symbolic::Expression.
    const T neg_one = test::copysign_int_to_non_symbolic_scalar(-1, T{1.0});
    DRAKE_DEMAND(neg_one == T{-1.0});
  }

  explicit DirectScalarTypeConversionSystem(
      const DirectScalarTypeConversionSystem<double>&, int dummy = 0)
      : DirectScalarTypeConversionSystem<T>() {}

 private:
  static SystemScalarConverter MakeConverter() {
    // Only support double => AutoDiffXd.
    SystemScalarConverter result;
    result.AddIfSupported<
      systems::DirectScalarTypeConversionSystem, AutoDiffXd, double>();
    return result;
  }
};

TEST_F(VectorSystemTest, DirectToAutoDiffXdTest) {
  DirectScalarTypeConversionSystem<double> dut;

  // Convert to AutoDiffXd.
  EXPECT_TRUE(is_autodiffxd_convertible(dut));

  // Convert to Symbolic (expected fail).
  EXPECT_EQ(dut.ToSymbolicMaybe(), nullptr);
}

// This system declares an output and continuous state, but does not define
// the required methods.
class MissingMethodsContinuousTimeSystem : public VectorSystem<double> {
 public:
  MissingMethodsContinuousTimeSystem() : VectorSystem<double>(0, 1) {
    this->DeclareContinuousState(1);
  }
};

TEST_F(VectorSystemTest, MissingMethodsContinuousTimeSystemTest) {
  MissingMethodsContinuousTimeSystem dut;
  auto context = dut.CreateDefaultContext();

  std::unique_ptr<ContinuousState<double>> derivatives =
      dut.AllocateTimeDerivatives();
  EXPECT_THROW(dut.CalcTimeDerivatives(*context, derivatives.get()),
               std::exception);

  auto output = dut.get_output_port().Allocate();
  EXPECT_THROW(dut.get_output_port().Calc(*context, output.get()),
               std::exception);
}

// This system declares an output and discrete state, but does not define
// the required methods.
class MissingMethodsDiscreteTimeSystem : public VectorSystem<double> {
 public:
  MissingMethodsDiscreteTimeSystem() : VectorSystem<double>(0, 1) {
    this->DeclarePeriodicDiscreteUpdate(1.0);
    this->DeclareDiscreteState(1);
  }
};

TEST_F(VectorSystemTest, MissingMethodsDiscreteTimeSystemTest) {
  MissingMethodsDiscreteTimeSystem dut;
  auto context = dut.CreateDefaultContext();

  context->get_mutable_discrete_state().get_mutable_vector().SetFromVector(
      Vector1d::Constant(2.0));
  auto discrete_updates = dut.AllocateDiscreteVariables();
  EXPECT_THROW(
      dut.CalcDiscreteVariableUpdates(*context, discrete_updates.get()),
      std::exception);

  auto output = dut.get_output_port().Allocate();
  EXPECT_THROW(dut.get_output_port().Calc(*context, output.get()),
               std::exception);
}

}  // namespace
}  // namespace systems
}  // namespace drake
