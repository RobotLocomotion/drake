#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/estimators/gaussian_state_observer.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/estimators/test_utilities/stochastic_linear_system.h"
#include "drake/systems/estimators/test_utilities/sum_matrix_columns_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

using estimators_test::StochasticLinearSystem;
using estimators_test::SumMatrixColumnsSystem;

template <typename OptionsType>
concept support_sqrt_method = requires { OptionsType::use_square_root_method; };

template <typename OptionsType>
class NonlinearKalmanFilterTestBase : public ::testing::Test {
 protected:
  virtual std::unique_ptr<GaussianStateObserver<double>> MakeObserver(
      const System<double>& observed_system,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const OptionsType& options) const = 0;

  static OptionsType MakeOptions(bool use_sqrt_method, int num_states = 2) {
    OptionsType options;
    if constexpr (support_sqrt_method<OptionsType>) {
      options.use_square_root_method = use_sqrt_method;
    } else if (use_sqrt_method) {
      throw std::logic_error("Square root method not implemented!");
    }

    Eigen::VectorXd xhat_init = Eigen::VectorXd::Ones(num_states) * 1.23;
    Eigen::MatrixXd Phat_init =
        Eigen::MatrixXd::Identity(num_states, num_states);
    Phat_init(0, num_states - 1) = 0.5;
    Phat_init(num_states - 1, 0) = 0.5;
    Phat_init(num_states - 1, num_states - 1) = 2.0;
    options.initial_state_estimate = xhat_init;
    options.initial_state_covariance = Phat_init;

    return options;
  }

  static void ExtractSquareMatrix(
      const Eigen::Ref<const Eigen::VectorXd>& concatenated,
      Eigen::Ref<Eigen::MatrixXd> square_matrix) {
    const int size = square_matrix.rows();
    DRAKE_ASSERT(square_matrix.rows() == square_matrix.cols());
    DRAKE_ASSERT(concatenated.size() == size + size * size);
    square_matrix = Eigen::Map<const Eigen::MatrixXd>(
        concatenated.data() + size, size, size);
  }

  template <typename T>
  static void TestInputOutputPorts(const GaussianStateObserver<T>& observer) {
    EXPECT_EQ(observer.GetInputPort("observed_system_input").get_index(),
              observer.get_observed_system_input_input_port().get_index());
    EXPECT_EQ(observer.GetInputPort("observed_system_output").get_index(),
              observer.get_observed_system_output_input_port().get_index());
    EXPECT_EQ(observer.GetOutputPort("estimated_state").get_index(),
              observer.get_estimated_state_output_port().get_index());
  }
};

template <typename OptionsType>
class DiscreteTimeNonlinearKalmanFilterTest
    : public NonlinearKalmanFilterTestBase<OptionsType> {
 private:
  void SetUp() override {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;
    A << 1, h_, 0, 1;
    B << 0.5 * h_ * h_, h_;
    C << 1, 0;
    D << 0;
    linear_sys_ = std::make_shared<LinearSystem<double>>(A, B, C, D, h_);
  }

  std::shared_ptr<LinearSystem<double>> linear_sys_;
  const double h_ = 0.01;
  const Eigen::MatrixXd W_ = Eigen::MatrixXd::Identity(2, 2) * h_;
  const Eigen::MatrixXd V_ = Eigen::MatrixXd::Identity(1, 1) / h_;

  void CheckErrorDynamics(
      const GaussianStateObserver<double>& observer, Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& u,
      const Eigen::Ref<const Eigen::VectorXd>& y,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> G_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> H_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> W_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> V_opt =
          std::nullopt) const {
    const Eigen::MatrixXd A = linear_sys_->A();
    const Eigen::MatrixXd B = linear_sys_->B();
    const Eigen::MatrixXd C = linear_sys_->C();
    const Eigen::MatrixXd D = linear_sys_->D();
    const Eigen::MatrixXd G = G_opt ? *G_opt : Eigen::MatrixXd::Identity(2, 2);
    const Eigen::MatrixXd H = H_opt ? *H_opt : Eigen::MatrixXd::Identity(1, 1);

    const Eigen::MatrixXd W = W_opt ? Eigen::MatrixXd(*W_opt) : W_;
    const Eigen::MatrixXd V = V_opt ? Eigen::MatrixXd(*V_opt) : V_;

    Eigen::VectorXd xhat = observer.GetStateEstimate(*context);
    Eigen::MatrixXd Phat = observer.GetStateCovariance(*context);

    // Measurement update.
    // K = P̂C'(CP̂C' + HVH')⁻¹
    // P̂[n|n] = (I - KC)P̂[n|n-1]
    // x̂[n|n] = x̂[n|n-1] + K(y - ŷ)
    Eigen::MatrixXd K =
        Phat * C.transpose() *
        (C * Phat * C.transpose() + H * V * H.transpose()).inverse();
    Phat = (Eigen::Matrix2d::Identity() - K * C) * Phat;
    xhat = xhat + K * (y - C * xhat - D * u);
    // Prediction update.
    // P̂[n+1|n] = AP̂[n|n-1]A' + GWG'
    // x̂[n+1|n] = Ax̂[n|n] + Bu[n]
    Phat = A * Phat * A.transpose() + G * W * G.transpose();
    xhat = A * xhat + B * u;

    context->SetDiscreteState(
        observer.EvalUniquePeriodicDiscreteUpdate(*context));
    const double kTol = 1e-12;
    EXPECT_TRUE(
        CompareMatrices(observer.GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer.GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer.get_estimated_state_output_port().Eval(*context), xhat, kTol));
  }

 protected:
  void TestConstruction(bool use_sqrt_method) {
    auto plant = linear_sys_;

    Eigen::VectorXd xhat = Eigen::Vector2d::Ones();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;

    auto observer = this->MakeObserver(*plant, W_, V_, options);
    EXPECT_TRUE(observer->IsDifferenceEquationSystem());

    this->TestInputOutputPorts(*observer);

    auto context = observer->CreateDefaultContext();
    const double kTol = 1e-14;
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));

    xhat << 1.23, 4.56;
    Phat << 2, 1, 1, 2;
    observer->SetStateEstimateAndCovariance(context.get(), xhat, Phat);
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));
  }

  void TestVectorInputDynamics(bool use_sqrt_method) {
    auto plant = linear_sys_;
    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y);
  }

  void TestNoInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto source = builder.AddSystem<ConstantVectorSource>(5.67);
    builder.Connect(source->get_output_port(), sys->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * 5.67, y);
  }

  void TestAbstractInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto mid = builder.AddSystem<SumMatrixColumnsSystem<double>>(1, 5);
    builder.Connect(mid->get_output_port(), sys->get_input_port());
    builder.ExportInput(mid->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::MatrixXd u = Eigen::MatrixXd::Ones(1, 5);
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(),
                                                              Value(u));
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * u.sum(), y);
  }

  void TestProcessNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2);
    G << 1 * h_, 2 * h_, 3 * h_, 4 * h_;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 1);
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(1));
    builder.Connect(source->get_output_port(), sys->get_v_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_w_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity() * h_;
    Eigen::MatrixXd V = V_;

    auto options = this->MakeOptions(use_sqrt_method);
    options.process_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, G, std::nullopt, W, V);
  }

  void TestMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd H(1, 2);
    H << 5 * h_, 6 * h_;
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(2));
    builder.Connect(source->get_output_port(), sys->get_w_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_v_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = W_;
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity() / h_;

    auto options = this->MakeOptions(use_sqrt_method);
    options.measurement_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, std::nullopt, H, W, V);
  }

  void TestProcessAndMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2), H(1, 2);
    G << 1 * h_, 2 * h_, 3 * h_, 4 * h_;
    H << 5 * h_, 6 * h_;
    StochasticLinearSystem plant(*linear_sys_, G, H);

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity() * h_;
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity() / h_;

    auto options = this->MakeOptions(use_sqrt_method);
    options.process_noise_input_port_index =
        plant.get_w_input_port().get_index();
    options.measurement_noise_input_port_index =
        plant.get_v_input_port().get_index();

    auto observer = this->MakeObserver(plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, G, H, W, V);
  }

  void TestSteadyState(bool use_sqrt_method) {
    Eigen::VectorXd xhat = Eigen::Vector2d::Ones();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;

    DiagramBuilder<double> builder;
    auto observer =
        builder.AddSystem(this->MakeObserver(*linear_sys_, W_, V_, options));
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Ones(1));
    auto plant = builder.AddSystem(linear_sys_);
    builder.Connect(source->get_output_port(), plant->get_input_port());
    builder.Connect(source->get_output_port(),
                    observer->get_observed_system_input_input_port());
    builder.Connect(plant->get_output_port(),
                    observer->get_observed_system_output_input_port());
    auto diagram = builder.Build();

    Simulator<double> simulator(*diagram);
    simulator.AdvanceTo(10);

    // Steady state observer gain.
    // x̂[n+1|n] = Ax̂[n|n-1] + Bu[n] + L(y - ŷ)
    auto& A = plant->A();
    auto& C = plant->C();
    Eigen::MatrixXd L1 = DiscreteTimeSteadyStateKalmanFilter(A, C, W_, V_);

    // Discrete-time observer dynamics.
    // K = P̂C'(CP̂C' + V)⁻¹
    // x̂[n|n] = x̂[n|n-1] + K(y - ŷ)
    // x̂[n+1|n] = Ax̂[n|n] + Bu[n]
    auto& observer_context =
        dynamic_cast<const DiagramContext<double>&>(simulator.get_context())
            .GetSubsystemContext(diagram->GetSystemIndexOrAbort(observer));
    Phat = observer->GetStateCovariance(observer_context);
    Eigen::MatrixXd L2 =
        A * Phat * C.transpose() * (C * Phat * C.transpose() + V_).inverse();

    EXPECT_TRUE(CompareMatrices(L1, L2, 1e-5));
  }
};

template <typename OptionsType>
class ContinuousDiscreteNonlinearKalmanFilterTest
    : public NonlinearKalmanFilterTestBase<OptionsType> {
 private:
  void SetUp() override {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;
    A << 0, 1, 0, 0;
    B << 0, 1;
    C << 1, 0;
    D << 0;
    linear_sys_ = std::make_shared<LinearSystem<double>>(A, B, C, D);
  }

  std::shared_ptr<LinearSystem<double>> linear_sys_;
  const Eigen::MatrixXd W_ = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd V_ = Eigen::MatrixXd::Identity(1, 1);
  const double measurement_time_period_ = 0.01;

  void CheckErrorDynamics(
      const GaussianStateObserver<double>& observer, Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& u,
      const Eigen::Ref<const Eigen::VectorXd>& y, bool use_sqrt_method,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> G_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> H_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> W_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> V_opt =
          std::nullopt) const {
    const Eigen::MatrixXd A = linear_sys_->A();
    const Eigen::MatrixXd B = linear_sys_->B();
    const Eigen::MatrixXd C = linear_sys_->C();
    const Eigen::MatrixXd D = linear_sys_->D();
    const Eigen::MatrixXd G = G_opt ? *G_opt : Eigen::MatrixXd::Identity(2, 2);
    const Eigen::MatrixXd H = H_opt ? *H_opt : Eigen::MatrixXd::Identity(1, 1);

    const Eigen::MatrixXd W = W_opt ? Eigen::MatrixXd(*W_opt) : W_;
    const Eigen::MatrixXd V = V_opt ? Eigen::MatrixXd(*V_opt) : V_;

    Eigen::VectorXd xhat = observer.GetStateEstimate(*context);
    Eigen::MatrixXd Phat = observer.GetStateCovariance(*context);

    // Continuous-time process update.
    // dx̂/dt = Ax̂ + Bu
    // dP̂/dt = AP̂ + P̂A' + GWG'
    Eigen::VectorXd xhat_dot = A * xhat + B * u;
    Eigen::MatrixXd Phat_dot =
        A * Phat + Phat * A.transpose() + G * W * G.transpose();

    const Eigen::VectorXd derivatives =
        observer.EvalTimeDerivatives(*context).CopyToVector();
    const int num_states = A.rows();
    const double kTol = 1e-12;
    EXPECT_TRUE(CompareMatrices(derivatives.head(num_states), xhat_dot, kTol));

    if (!use_sqrt_method) {
      Eigen::MatrixXd Phat_dot_sim(num_states, num_states);
      this->ExtractSquareMatrix(derivatives, Phat_dot_sim);
      EXPECT_TRUE(CompareMatrices(Phat_dot_sim, Phat_dot, kTol));
    } else {
      Eigen::MatrixXd Shat_sim(num_states, num_states);
      this->ExtractSquareMatrix(context->get_continuous_state().CopyToVector(),
                                Shat_sim);
      Eigen::MatrixXd Shat_dot_sim(num_states, num_states);
      this->ExtractSquareMatrix(derivatives, Shat_dot_sim);
      Eigen::MatrixXd Phat_dot_sim = Shat_dot_sim * Shat_sim.transpose() +
                                     Shat_sim * Shat_dot_sim.transpose();
      EXPECT_TRUE(CompareMatrices(Phat_dot_sim, Phat_dot, kTol));
    }

    // Discrete-time measurement update.
    // K = P̂C'(CP̂C' + HVH')⁻¹
    // P̂ ← (I - KC)P̂
    // x̂ ← x̂ + K(y - ŷ)
    Eigen::MatrixXd K =
        Phat * C.transpose() *
        (C * Phat * C.transpose() + H * V * H.transpose()).inverse();
    Phat = (Eigen::Matrix2d::Identity() - K * C) * Phat;
    xhat = xhat + K * (y - C * xhat - D * u);

    auto event_collection = observer.AllocateCompositeEventCollection();
    observer.GetPeriodicEvents(*context, event_collection.get());
    EXPECT_TRUE(event_collection->get_unrestricted_update_events().HasEvents());
    EXPECT_TRUE(observer
                    .CalcUnrestrictedUpdate(
                        *context,
                        event_collection->get_unrestricted_update_events(),
                        &context->get_mutable_state())
                    .succeeded());
    EXPECT_TRUE(
        CompareMatrices(observer.GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer.GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer.get_estimated_state_output_port().Eval(*context), xhat, kTol));
  }

 protected:
  void TestConstruction(bool use_sqrt_method) {
    auto plant = linear_sys_;

    Eigen::VectorXd xhat = Eigen::Vector2d::Ones();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;
    options.discrete_measurement_time_period = measurement_time_period_;

    auto observer = this->MakeObserver(*plant, W_, V_, options);

    this->TestInputOutputPorts(*observer);

    auto context = observer->CreateDefaultContext();
    const double kTol = 1e-14;
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));

    xhat << 1.23, 4.56;
    Phat << 2, 1, 1, 2;
    observer->SetStateEstimateAndCovariance(context.get(), xhat, Phat);
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));
  }

  void TestVectorInputDynamics(bool use_sqrt_method) {
    auto plant = linear_sys_;
    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method);
  }

  void TestNoInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto source = builder.AddSystem<ConstantVectorSource>(5.67);
    builder.Connect(source->get_output_port(), sys->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * 5.67, y, use_sqrt_method);
  }

  void TestAbstractInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto mid = builder.AddSystem<SumMatrixColumnsSystem<double>>(1, 5);
    builder.Connect(mid->get_output_port(), sys->get_input_port());
    builder.ExportInput(mid->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::MatrixXd u = Eigen::MatrixXd::Ones(1, 5);
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(),
                                                              Value(u));
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * u.sum(), y, use_sqrt_method);
  }

  void TestProcessNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2);
    G << 1.0, 2.0, 3.0, 4.0;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 1);
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(1));
    builder.Connect(source->get_output_port(), sys->get_v_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_w_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd V = V_;

    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    options.process_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method, G,
                       std::nullopt, W, V);
  }

  void TestMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd H(1, 2);
    H << 5.0, 6.0;
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(2));
    builder.Connect(source->get_output_port(), sys->get_w_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_v_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = W_;
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity();

    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    options.measurement_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method,
                       std::nullopt, H, W, V);
  }

  void TestProcessAndMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2), H(1, 2);
    G << 1.0, 2.0, 3.0, 4.0;
    H << 5.0, 6.0;
    StochasticLinearSystem plant(*linear_sys_, G, H);

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity();

    auto options = this->MakeOptions(use_sqrt_method);
    options.discrete_measurement_time_period = measurement_time_period_;
    options.process_noise_input_port_index =
        plant.get_w_input_port().get_index();
    options.measurement_noise_input_port_index =
        plant.get_v_input_port().get_index();

    auto observer = this->MakeObserver(plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method, G, H, W,
                       V);
  }

  void TestSimulation(bool use_sqrt_method) {
    Eigen::VectorXd xhat = Eigen::Vector2d::Ones();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;
    options.discrete_measurement_time_period = measurement_time_period_;

    Eigen::VectorXd u(1);
    u << 1.2;
    Eigen::VectorXd y(1);
    y << 3.4;

    DiagramBuilder<double> builder;
    auto observer =
        builder.AddSystem(this->MakeObserver(*linear_sys_, W_, V_, options));
    auto u_source = builder.AddSystem<ConstantVectorSource>(u);
    auto y_source = builder.AddSystem<ConstantVectorSource>(y);
    builder.Connect(u_source->get_output_port(),
                    observer->get_observed_system_input_input_port());
    builder.Connect(y_source->get_output_port(),
                    observer->get_observed_system_output_input_port());
    auto diagram = builder.Build();

    Simulator<double> simulator(*diagram);
    auto& observer_context =
        dynamic_cast<const DiagramContext<double>&>(simulator.get_context())
            .GetSubsystemContext(diagram->GetSystemIndexOrAbort(observer));

    // System matrices.
    auto& C = linear_sys_->C();
    auto& D = linear_sys_->D();

    // Discrete-time measurement update.
    // K = P̂C'(CP̂C' + V)⁻¹
    // P̂ ← (I - KC)P̂
    // x̂ ← x̂ + K(y - ŷ)
    Eigen::MatrixXd K =
        Phat * C.transpose() * (C * Phat * C.transpose() + V_).inverse();
    Phat = (Eigen::Matrix2d::Identity() - K * C) * Phat;
    xhat = xhat + K * (y - C * xhat - D * u);

    simulator.AdvanceTo(0);
    EXPECT_TRUE(CompareMatrices(observer->GetStateEstimate(observer_context),
                                xhat, 1e-12));
    EXPECT_TRUE(CompareMatrices(observer->GetStateCovariance(observer_context),
                                Phat, 1e-12));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(observer_context),
        xhat, 1e-12));

    // Hardcoded solution for integrating the continuous-time process update:
    // dx̂/dt = Ax̂ + Bu,  dP̂/dt = AP̂ + P̂A' + W.
    xhat << 2.216060000000000, 1.612000000000000;
    // clang-format off
    Phat << 0.515087833333333, 0.258800000000000,
            0.258800000000000, 0.885000000000000;
    // clang-format on
    simulator.AdvanceTo(measurement_time_period_ -
                        std::numeric_limits<double>::epsilon());
    EXPECT_TRUE(CompareMatrices(observer->GetStateEstimate(observer_context),
                                xhat, 1e-12));
    EXPECT_TRUE(CompareMatrices(observer->GetStateCovariance(observer_context),
                                Phat, !use_sqrt_method ? 1e-12 : 1e-9));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(observer_context),
        xhat, 1e-12));
  }
};

template <typename OptionsType>
class ContinuousTimeNonlinearKalmanFilterTest
    : public NonlinearKalmanFilterTestBase<OptionsType> {
 private:
  void SetUp() override {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;
    A << 0, 1, 0, 0;
    B << 0, 1;
    C << 1, 0;
    D << 0;
    linear_sys_ = std::make_shared<LinearSystem<double>>(A, B, C, D);
  }

  std::shared_ptr<LinearSystem<double>> linear_sys_;
  const Eigen::MatrixXd W_ = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd V_ = Eigen::MatrixXd::Identity(1, 1);

  void CheckErrorDynamics(
      const GaussianStateObserver<double>& observer, Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& u,
      const Eigen::Ref<const Eigen::VectorXd>& y, bool use_sqrt_method,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> G_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> H_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> W_opt = std::nullopt,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> V_opt =
          std::nullopt) const {
    const Eigen::MatrixXd A = linear_sys_->A();
    const Eigen::MatrixXd B = linear_sys_->B();
    const Eigen::MatrixXd C = linear_sys_->C();
    const Eigen::MatrixXd D = linear_sys_->D();
    const Eigen::MatrixXd G = G_opt ? *G_opt : Eigen::MatrixXd::Identity(2, 2);
    const Eigen::MatrixXd H = H_opt ? *H_opt : Eigen::MatrixXd::Identity(1, 1);

    const Eigen::MatrixXd W = W_opt ? Eigen::MatrixXd(*W_opt) : W_;
    const Eigen::MatrixXd V = V_opt ? Eigen::MatrixXd(*V_opt) : V_;

    Eigen::VectorXd xhat = observer.GetStateEstimate(*context);
    Eigen::MatrixXd Phat = observer.GetStateCovariance(*context);

    // dx̂/dt = Ax̂ + Bu + P̂C'(HVH')⁻¹(y - Cx̂ - Du)
    // dP̂/dt = AP̂ + P̂A' + GWG' - P̂C'(HVH')⁻¹CP̂
    Eigen::VectorXd xhat_dot = A * xhat + B * u +
                               Phat * C.transpose() *
                                   (H * V * H.transpose()).inverse() *
                                   (y - C * xhat - D * u);
    Eigen::MatrixXd Phat_dot =
        A * Phat + Phat * A.transpose() + G * W * G.transpose() -
        Phat * C.transpose() * (H * V * H.transpose()).inverse() * C * Phat;

    const Eigen::VectorXd derivatives =
        observer.EvalTimeDerivatives(*context).CopyToVector();
    const int num_states = A.rows();
    const double kTol = 1e-12;
    EXPECT_TRUE(CompareMatrices(derivatives.head(num_states), xhat_dot, kTol));

    if (!use_sqrt_method) {
      Eigen::MatrixXd Phat_dot_sim(num_states, num_states);
      this->ExtractSquareMatrix(derivatives, Phat_dot_sim);
      EXPECT_TRUE(CompareMatrices(Phat_dot_sim, Phat_dot, kTol));
    } else {
      Eigen::MatrixXd Shat_sim(num_states, num_states);
      this->ExtractSquareMatrix(context->get_continuous_state().CopyToVector(),
                                Shat_sim);
      Eigen::MatrixXd Shat_dot_sim(num_states, num_states);
      this->ExtractSquareMatrix(derivatives, Shat_dot_sim);
      Eigen::MatrixXd Phat_dot_sim = Shat_dot_sim * Shat_sim.transpose() +
                                     Shat_sim * Shat_dot_sim.transpose();
      EXPECT_TRUE(CompareMatrices(Phat_dot_sim, Phat_dot, kTol));
    }
  }

 protected:
  void TestConstruction(bool use_sqrt_method) {
    auto plant = linear_sys_;

    Eigen::VectorXd xhat = Eigen::Vector2d::Ones();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;

    auto observer = this->MakeObserver(*plant, W_, V_, options);
    EXPECT_TRUE(observer->IsDifferentialEquationSystem());

    this->TestInputOutputPorts(*observer);

    auto context = observer->CreateDefaultContext();
    const double kTol = 1e-14;
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));

    xhat << 1.23, 4.56;
    Phat << 2, 1, 1, 2;
    observer->SetStateEstimateAndCovariance(context.get(), xhat, Phat);
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateEstimate(*context), xhat, kTol));
    EXPECT_TRUE(
        CompareMatrices(observer->GetStateCovariance(*context), Phat, kTol));
    EXPECT_TRUE(CompareMatrices(
        observer->get_estimated_state_output_port().Eval(*context), xhat,
        kTol));
  }

  void TestVectorInputDynamics(bool use_sqrt_method) {
    auto plant = linear_sys_;
    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method);
  }

  void TestNoInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto source = builder.AddSystem<ConstantVectorSource>(5.67);
    builder.Connect(source->get_output_port(), sys->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * 5.67, y, use_sqrt_method);
  }

  void TestAbstractInputDynamics(bool use_sqrt_method) {
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem(linear_sys_);
    auto mid = builder.AddSystem<SumMatrixColumnsSystem<double>>(1, 5);
    builder.Connect(mid->get_output_port(), sys->get_input_port());
    builder.ExportInput(mid->get_input_port());
    builder.ExportOutput(sys->get_output_port());
    auto plant = builder.Build();

    auto options = this->MakeOptions(use_sqrt_method);
    auto observer = this->MakeObserver(*plant, W_, V_, options);
    auto context = observer->CreateDefaultContext();

    Eigen::MatrixXd u = Eigen::MatrixXd::Ones(1, 5);
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(),
                                                              Value(u));
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(),
                       Eigen::VectorXd::Ones(1) * u.sum(), y, use_sqrt_method);
  }

  void TestProcessNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2);
    G << 1.0, 2.0, 3.0, 4.0;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 1);
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(1));
    builder.Connect(source->get_output_port(), sys->get_v_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_w_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd V = V_;

    auto options = this->MakeOptions(use_sqrt_method);
    options.process_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method, G,
                       std::nullopt, W, V);
  }

  void TestMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd H(1, 2);
    H << 5.0, 6.0;
    DiagramBuilder<double> builder;
    auto sys = builder.AddSystem<StochasticLinearSystem>(*linear_sys_, G, H);
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Zero(2));
    builder.Connect(source->get_output_port(), sys->get_w_input_port());
    builder.ExportInput(sys->get_u_input_port());
    builder.ExportInput(sys->get_v_input_port());
    builder.ExportOutput(sys->get_y_output_port());
    auto plant = builder.Build();

    Eigen::MatrixXd W = W_;
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity();

    auto options = this->MakeOptions(use_sqrt_method);
    options.measurement_noise_input_port_index = InputPortIndex(1);

    auto observer = this->MakeObserver(*plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method,
                       std::nullopt, H, W, V);
  }

  void TestProcessAndMeasurementNoiseInputDynamics(bool use_sqrt_method) {
    Eigen::MatrixXd G(2, 2), H(1, 2);
    G << 1.0, 2.0, 3.0, 4.0;
    H << 5.0, 6.0;
    StochasticLinearSystem plant(*linear_sys_, G, H);

    Eigen::MatrixXd W = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd V = Eigen::Matrix2d::Identity();

    auto options = this->MakeOptions(use_sqrt_method);
    options.process_noise_input_port_index =
        plant.get_w_input_port().get_index();
    options.measurement_noise_input_port_index =
        plant.get_v_input_port().get_index();

    auto observer = this->MakeObserver(plant, W, V, options);
    auto context = observer->CreateDefaultContext();

    Eigen::VectorXd u = Eigen::VectorXd::Ones(1) * 1.2;
    Eigen::VectorXd y = Eigen::VectorXd::Ones(1) * 3.4;
    observer->get_observed_system_input_input_port().FixValue(context.get(), u);
    observer->get_observed_system_output_input_port().FixValue(context.get(),
                                                               y);

    CheckErrorDynamics(*observer, context.get(), u, y, use_sqrt_method, G, H, W,
                       V);
  }

  void TestSteadyState(bool use_sqrt_method) {
    Eigen::VectorXd xhat = Eigen::Vector2d::Zero();
    Eigen::MatrixXd Phat(2, 2);
    Phat << 1, 0.5, 0.5, 1;
    auto options = this->MakeOptions(use_sqrt_method);
    options.initial_state_estimate = xhat;
    options.initial_state_covariance = Phat;

    DiagramBuilder<double> builder;
    auto observer =
        builder.AddSystem(this->MakeObserver(*linear_sys_, W_, V_, options));
    auto source =
        builder.AddSystem<ConstantVectorSource>(Eigen::VectorXd::Ones(1));
    auto plant = builder.AddSystem(linear_sys_);
    builder.Connect(source->get_output_port(), plant->get_input_port());
    builder.Connect(source->get_output_port(),
                    observer->get_observed_system_input_input_port());
    builder.Connect(plant->get_output_port(),
                    observer->get_observed_system_output_input_port());
    auto diagram = builder.Build();

    Simulator<double> simulator(*diagram);
    simulator.AdvanceTo(10);

    // Steady state observer gain.
    // dx̂/dt = Ax̂ + Bu + L(y - ŷ)
    auto& A = plant->A();
    auto& C = plant->C();
    Eigen::MatrixXd L1 = SteadyStateKalmanFilter(A, C, W_, V_);

    // Continuous-time observer dynamics.
    // dx̂/dt = Ax̂ + Bu + P̂C'V⁻¹(y - ŷ)
    auto& observer_context =
        dynamic_cast<const DiagramContext<double>&>(simulator.get_context())
            .GetSubsystemContext(diagram->GetSystemIndexOrAbort(observer));
    Phat = observer->GetStateCovariance(observer_context);
    Eigen::MatrixXd L2 = Phat * C.transpose() * V_.inverse();

    EXPECT_TRUE(CompareMatrices(L1, L2, !use_sqrt_method ? 1e-5 : 1e-4));
  }
};

}  // namespace estimators
}  // namespace systems
}  // namespace drake
