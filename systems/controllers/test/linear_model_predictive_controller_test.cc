#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

using math::DiscreteAlgebraicRiccatiEquation;

class TestMpcWithDoubleIntegrator : public ::testing::Test {
 protected:
  void SetUp() override {
    const double kTimeStep = 0.1;     // discrete time step.
    const double kTimeHorizon = 10.;  // Time horizon.

    // A discrete approximation of a double integrator.
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 1, 0.1, 0, 1;
    B << 0.005, 0.1;
    const auto C = Eigen::Matrix<double, 2, 2>::Identity();
    const auto D = Eigen::Matrix<double, 2, 1>::Zero();

    std::unique_ptr<LinearSystem<double>> system =
        std::make_unique<LinearSystem<double>>(A, B, C, D, kTimeStep);

    // Nominal, fixed reference condition.
    const Eigen::Vector2d x0 = Eigen::Vector2d::Zero();
    const Vector1d u0 = Vector1d::Zero();

    std::unique_ptr<Context<double>> system_context =
        system->CreateDefaultContext();
    system->get_input_port().FixValue(system_context.get(), u0);
    system_context->SetDiscreteState(0, x0);

    dut_.reset(new LinearModelPredictiveController<double>(
        std::move(system), std::move(system_context), Q_, R_, kTimeStep,
        kTimeHorizon));

    // Store another copy of the linear plant model.
    system_.reset(new LinearSystem<double>(A, B, C, D, kTimeStep));
  }

  // Cost matrices.
  const Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
  const Vector1d R_ = Vector1d::Constant(1.);

  std::unique_ptr<LinearModelPredictiveController<double>> dut_;
  std::unique_ptr<LinearSystem<double>> system_;
};

TEST_F(TestMpcWithDoubleIntegrator, TestAgainstInfiniteHorizonSolution) {
  const double kTolerance = 1e-5;

  const Eigen::Matrix2d A = system_->A();
  const Eigen::Matrix<double, 2, 1> B = system_->B();

  // Analytical solution to the LQR problem.
  const Eigen::Matrix2d S = DiscreteAlgebraicRiccatiEquation(A, B, Q_, R_);
  const Eigen::Matrix<double, 1, 2> K =
      -(R_ + B.transpose() * S * B).inverse() * (B.transpose() * S * A);

  const Eigen::Matrix<double, 2, 1> x0 = Eigen::Vector2d::Ones();

  auto context = dut_->CreateDefaultContext();
  dut_->get_input_port(0).FixValue(context.get(), x0);
  std::unique_ptr<SystemOutput<double>> output = dut_->AllocateOutput();

  dut_->CalcOutput(*context, output.get());

  EXPECT_TRUE(CompareMatrices(K * x0, output->get_vector_data(0)->get_value(),
                              kTolerance));
}

namespace {

// A discrete-time cubic polynomial system.
template <typename T>
class CubicPolynomialSystem final : public LeafSystem<T> {
 public:
  explicit CubicPolynomialSystem(double time_step)
      : LeafSystem<T>(SystemTypeTag<CubicPolynomialSystem>{}),
        time_step_(time_step) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 1);
    this->DeclareVectorOutputPort(kUseDefaultName, 2,
                                  &CubicPolynomialSystem::OutputState,
                                  {this->all_state_ticket()});
    this->DeclareDiscreteState(2);
    this->DeclarePeriodicDiscreteUpdate(time_step);
  }

  template <typename U>
  CubicPolynomialSystem(const CubicPolynomialSystem<U>& other)
      : CubicPolynomialSystem(other.time_step_) {}

 private:
  template <typename> friend class CubicPolynomialSystem;

  // x1(k+1) = u(k)
  // x2(k+1) = -x1³(k)
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* next_state) const final {
    using std::pow;
    const T& x1 = context.get_discrete_state(0).get_value()[0];
    const T& u = this->get_input_port(0).Eval(context)[0];
    next_state->set_value(0, Vector2<T>{u, pow(x1, 3.)});
  }

  void OutputState(const systems::Context<T>& context,
                   BasicVector<T>* output) const {
    output->set_value(context.get_discrete_state(0).get_value());
  }

  const double time_step_{0.};
};

}  // namespace

class TestMpcWithCubicSystem : public ::testing::Test {
 protected:
  const System<double>& GetSystemByName(std::string name,
                                        const Diagram<double>& diagram) {
    const System<double>* result{nullptr};
    for (const System<double>* system : diagram.GetSystems()) {
      if (system->get_name() == name) result = system;
    }
    return *result;
  }

  void MakeTimeInvariantMpcController() {
    EXPECT_NE(nullptr, system_);
    auto context = system_->CreateDefaultContext();

    // Set nominal input to zero.
    system_->get_input_port(0).FixValue(context.get(), 0.);

    // Set the nominal state.
    BasicVector<double>& x =
        context->get_mutable_discrete_state().get_mutable_vector();
    x.SetFromVector(Eigen::Vector2d::Zero());  // Fixed point is zero.

    dut_.reset(new LinearModelPredictiveController<double>(
        std::move(system_), std::move(context), Q_, R_, time_step_,
        time_horizon_));
  }

  void MakeControlledSystem(bool is_time_varying) {
    EXPECT_FALSE(is_time_varying);  // TODO(jadecastro) Introduce tests for the
                                    // time-varying case.
    EXPECT_EQ(nullptr, diagram_);

    system_.reset(new CubicPolynomialSystem<double>(time_step_));
    EXPECT_FALSE(system_->HasAnyDirectFeedthrough());

    DiagramBuilder<double> builder;
    auto cubic_system = builder.AddSystem<CubicPolynomialSystem>(time_step_);
    cubic_system->set_name("cubic_system");

    MakeTimeInvariantMpcController();
    EXPECT_NE(nullptr, dut_);
    auto controller = builder.AddSystem(std::move(dut_));
    controller->set_name("controller");

    builder.Connect(cubic_system->get_output_port(0),
                    controller->get_state_port());
    builder.Connect(controller->get_control_port(),
                    cubic_system->get_input_port(0));

    diagram_ = builder.Build();
  }

  void Simulate() {
    EXPECT_NE(nullptr, diagram_);
    EXPECT_EQ(nullptr, simulator_);

    simulator_.reset(new Simulator<double>(*diagram_));

    const auto& cubic_system = GetSystemByName("cubic_system", *diagram_);
    Context<double>& cubic_system_context =
        diagram_->GetMutableSubsystemContext(
            cubic_system, &simulator_->get_mutable_context());
    BasicVector<double>& x0 =
        cubic_system_context.get_mutable_discrete_state().get_mutable_vector();

    // Set an initial condition near the fixed point.
    x0.SetFromVector(10. * Eigen::Vector2d::Ones());

    simulator_->set_target_realtime_rate(1.);
    simulator_->Initialize();
    simulator_->AdvanceTo(time_horizon_);
  }

  const double time_step_ = 0.1;
  const double time_horizon_ = 0.2;

  // Set up the quadratic cost matrices.
  const Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
  const Vector1d R_ = Vector1d::Constant(1.);

  std::unique_ptr<Simulator<double>> simulator_;

 private:
  std::unique_ptr<LinearModelPredictiveController<double>> dut_;
  std::unique_ptr<CubicPolynomialSystem<double>> system_;
  std::unique_ptr<Diagram<double>> diagram_;
};

TEST_F(TestMpcWithCubicSystem, TimeInvariantMpc) {
  const double kTolerance = 1e-10;
  MakeControlledSystem(false /* is NOT time-varying */);
  Simulate();

  // Result should be deadbeat; expect convergence to within a tiny tolerance in
  // one step.
  Eigen::Vector2d result =
      simulator_->get_mutable_context().get_discrete_state(0).get_value();
  EXPECT_TRUE(CompareMatrices(result, Eigen::Vector2d::Zero(), kTolerance));
}

GTEST_TEST(TestMpcConstructor, ThrowIfRNotStrictlyPositiveDefinite) {
  const auto A = Eigen::Matrix<double, 2, 2>::Identity();
  const auto B = Eigen::Matrix<double, 2, 2>::Identity();
  const auto C = Eigen::Matrix<double, 2, 2>::Identity();
  const auto D = Eigen::Matrix<double, 2, 2>::Zero();

  std::unique_ptr<LinearSystem<double>> system =
      std::make_unique<LinearSystem<double>>(A, B, C, D, 1.);

  std::unique_ptr<Context<double>> context = system->CreateDefaultContext();
  system->get_input_port().FixValue(context.get(), Eigen::Vector2d::Zero());

  const Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();

  // Provide a positive-definite matrix (but not strict).
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  R(0, 0) = 0.;

  // Expect the constructor to throw since R is not strictly positive definite.
  EXPECT_THROW(LinearModelPredictiveController<double>(
      std::move(system), std::move(context), Q, R, 1., 1.),
               std::runtime_error);
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
