#include "drake/systems/framework/system_symbolic_inspector.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 2;

class SparseSystem : public LeafSystem<symbolic::Expression> {
 public:
  SparseSystem() {
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareInputPort(kVectorValued, kSize);

    this->DeclareVectorOutputPort(BasicVector<symbolic::Expression>(kSize),
                                  &SparseSystem::CalcY0);
    this->DeclareVectorOutputPort(BasicVector<symbolic::Expression>(kSize),
                                  &SparseSystem::CalcY1);
    this->DeclareAbstractOutputPort(42, &SparseSystem::CalcNothing);

    this->DeclareContinuousState(kSize);
    this->DeclareDiscreteState(kSize);

    this->DeclareEqualityConstraint(&SparseSystem::CalcConstraint, kSize,
                                    "equality constraint");
    this->DeclareInequalityConstraint(&SparseSystem::CalcConstraint, kSize,
                                      "inequality constraint");
  }

  void AddAbstractInputPort() { this->DeclareAbstractInputPort(); }

  ~SparseSystem() override {}

 private:
  // Calculation function for output port 0.
  void CalcY0(const Context<symbolic::Expression>& context,
              BasicVector<symbolic::Expression>* y0) const {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    const auto& xc = context.get_continuous_state_vector();

    // Output 0 depends on input 0 and the continuous state.  Input 1 appears in
    // an intermediate computation, but is ultimately cancelled out.
    y0->set_value(u1.get_value());
    y0->PlusEqScaled(1, u0);
    y0->PlusEqScaled(-1, u1);
    y0->PlusEqScaled(12, xc);
  }

  // Calculation function for output port 1.
  void CalcY1(const Context<symbolic::Expression>& context,
              BasicVector<symbolic::Expression>* y1) const {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    const auto& xd = context.get_discrete_state(0);

    // Output 1 depends on both inputs and the discrete state.
    y1->set_value(u0.get_value() + u1.get_value() + xd.get_value());
  }

  void CalcNothing(const Context<symbolic::Expression>& context, int*) const {}

  void CalcConstraint(const Context<symbolic::Expression>& context,
                      VectorX<symbolic::Expression>* value) const {
    *value = context.get_continuous_state_vector().CopyToVector();
  }

  // Implements a time-varying affine dynamics.
  void DoCalcTimeDerivatives(
      const Context<symbolic::Expression>& context,
      ContinuousState<symbolic::Expression>* derivatives) const override {
    const auto u0 = this->EvalVectorInput(context, 0)->CopyToVector();
    const auto u1 = this->EvalVectorInput(context, 1)->CopyToVector();
    const auto& t = context.get_time();
    const Vector2<symbolic::Expression> x =
        context.get_continuous_state_vector().CopyToVector();
    const Eigen::Matrix2d A = 2 * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d B1 = 3 * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d B2 = 4 * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d f0(5.0, 6.0);
    const Vector2<symbolic::Expression> xdot =
        A * t * x + B1 * u0 + B2 * u1 + f0;
    derivatives->SetFromVector(xdot);
  }

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<symbolic::Expression>& context,
      const std::vector<
          const systems::DiscreteUpdateEvent<symbolic::Expression>*>&,
      systems::DiscreteValues<symbolic::Expression>* discrete_state)
      const override {
    const auto u0 = this->EvalVectorInput(context, 0)->CopyToVector();
    const auto u1 = this->EvalVectorInput(context, 1)->CopyToVector();
    const Vector2<symbolic::Expression> xd =
        context.get_discrete_state(0).get_value();
    const Eigen::Matrix2d A = 7 * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d B1 = 8 * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d B2 = 9 * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d f0(10.0, 11.0);
    const Vector2<symbolic::Expression> next_xd =
        A * xd + B1 * u0 + B2 * u1 + f0;
    discrete_state->get_mutable_vector(0).SetFromVector(next_xd);
  }
};

class SystemSymbolicInspectorTest : public ::testing::Test {
 public:
  SystemSymbolicInspectorTest() : system_() {}

 protected:
  void SetUp() override {
    inspector_ = std::make_unique<SystemSymbolicInspector>(system_);
  }

  SparseSystem system_;
  std::unique_ptr<SystemSymbolicInspector> inspector_;
};

class PendulumInspectorTest : public ::testing::Test {
 public:
  PendulumInspectorTest() : system_() {}

 protected:
  void SetUp() override {
    inspector_ = std::make_unique<SystemSymbolicInspector>(system_);
  }

  examples::pendulum::PendulumPlant<symbolic::Expression> system_;
  std::unique_ptr<SystemSymbolicInspector> inspector_;
};

// Tests that the SystemSymbolicInspector infers, from the symbolic equations of
// the System, that input 1 does not affect output 0.
TEST_F(SystemSymbolicInspectorTest, InputToOutput) {
  // Only input 0 affects output 0.
  EXPECT_TRUE(inspector_->IsConnectedInputToOutput(0, 0));
  EXPECT_FALSE(inspector_->IsConnectedInputToOutput(1, 0));
  // Both inputs affect output 1.
  EXPECT_TRUE(inspector_->IsConnectedInputToOutput(0, 1));
  EXPECT_TRUE(inspector_->IsConnectedInputToOutput(1, 1));
  // All inputs are presumed to affect output 2, since it is abstract.
  EXPECT_TRUE(inspector_->IsConnectedInputToOutput(0, 2));
  EXPECT_TRUE(inspector_->IsConnectedInputToOutput(1, 2));
}

// Tests that, if the System has an abstract input, the SystemSymbolicInspector
// conservatively reports that every output might depend on every input.
TEST_F(SystemSymbolicInspectorTest, AbstractContextThwartsSparsity) {
  system_.AddAbstractInputPort();
  inspector_ = std::make_unique<SystemSymbolicInspector>(system_);
  for (int i = 0; i < system_.get_num_input_ports(); ++i) {
    for (int j = 0; j < system_.get_num_output_ports(); ++j) {
      EXPECT_TRUE(inspector_->IsConnectedInputToOutput(i, j));
    }
  }
}

TEST_F(SystemSymbolicInspectorTest, ConstraintTest) {
  EXPECT_EQ(inspector_->constraints().size(), 4);

  int equality_constraint_count{0};
  for (const auto& formula : inspector_->constraints()) {
    if (is_equal_to(formula)) {
      equality_constraint_count++;
    }
    EXPECT_EQ(get_rhs_expression(formula), 0);
    EXPECT_TRUE(is_variable(get_lhs_expression(formula)));
  }
  EXPECT_EQ(equality_constraint_count, 2);
}

TEST_F(SystemSymbolicInspectorTest, IsTimeInvariant) {
  // The derivatives depends on t.
  EXPECT_FALSE(inspector_->IsTimeInvariant());
}

TEST_F(PendulumInspectorTest, IsTimeInvariant) {
  EXPECT_TRUE(inspector_->IsTimeInvariant());
}

TEST_F(SystemSymbolicInspectorTest, HasAffineDynamics) {
  EXPECT_TRUE(inspector_->HasAffineDynamics());
}

TEST_F(PendulumInspectorTest, HasAffineDynamics) {
  EXPECT_FALSE(inspector_->HasAffineDynamics());
}

TEST_F(PendulumInspectorTest, SymbolicParameters) {
  auto params = inspector_->numeric_parameters(0);

  using examples::pendulum::PendulumParamsIndices;
  EXPECT_EQ(params.size(), PendulumParamsIndices::kNumCoordinates);

  // Test that the damping parameter appears with the correct order in the
  // derivatives and output methods.
  symbolic::Variables v({params[PendulumParamsIndices::kDamping]});

  auto derivatives = inspector_->derivatives();
  EXPECT_EQ(symbolic::Polynomial(derivatives[0], v).TotalDegree(), 0);
  EXPECT_EQ(symbolic::Polynomial(derivatives[1], v).TotalDegree(), 1);

  auto output = inspector_->output(0);
  EXPECT_EQ(symbolic::Polynomial(output[0], v).TotalDegree(), 0);
  EXPECT_EQ(symbolic::Polynomial(output[1], v).TotalDegree(), 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
