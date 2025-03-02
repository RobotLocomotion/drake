#include "drake/systems/estimators/unscented_kalman_filter.h"

#include <exception>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

#include <fmt/format.h>

#include "drake/math/matrix_util.h"
#include "drake/systems/estimators/extended_kalman_filter.h"

namespace drake {
namespace systems {
namespace estimators {

// This implementation is mostly based on:
// [1] S. Sarkka, "On unscented Kalman filtering for state estimation of
//     continuous-time nonlinear systems," IEEE Transactions on Automatic
//     Control, vol. 52, no. 9, pp. 1631-1641, 2007.

namespace {

using internal::JointGaussian;
using internal::UnscentedTransform;

// GaussianStateObserver<double>          [Inheritance map]
//           ↓
// UnscentedKalmanFilterBase --------→ UnscentedKalmanFilterCont
//           ↓                          ↓                     ↓
// UnscentedKalmanFilterDD  UnscentedKalmanFilterCD  UnscentedKalmanFilterCC

// Base class for all unscented Kalman filters.
class UnscentedKalmanFilterBase : public GaussianStateObserver<double> {
 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnscentedKalmanFilterBase);

  UnscentedKalmanFilterBase(
      std::shared_ptr<const System<double>> observed_system,
      const Context<double>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const UnscentedKalmanFilterOptions& options)
      : observed_system_(std::move(observed_system)),
        num_states_(observed_system_context.num_total_states()),
        W_(W),
        V_(V),
        unscented_transform_params_(options.unscented_transform_parameters) {
    observed_system_->ValidateContext(observed_system_context);
    DRAKE_THROW_UNLESS(num_states_ > 0);  // Or else we don't need an observer.
    DRAKE_THROW_UNLESS(
        observed_system_context.has_only_continuous_state() ||
        (observed_system_context.has_only_discrete_state() &&
         observed_system_context.num_discrete_state_groups() == 1));

    // Copy the observed system context into a cache entry where we can safely
    // modify it without runtime reallocation or a (non-thread-safe) mutable
    // member.
    observed_system_context_cache_entry_ = &this->DeclareCacheEntry(
        "observed system context",
        ValueProducer(observed_system_context, &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});

    // Check the observed system input/output ports and set the
    // observed_system_(.*)put_port_ member variables.
    internal::CheckObservedSystemInputOutputPorts(
        *observed_system_, options, &observed_system_actuation_input_port_,
        &observed_system_measurement_output_port_,
        &observed_system_process_noise_input_port_,
        &observed_system_measurement_noise_input_port_);
    DRAKE_THROW_UNLESS(observed_system_measurement_output_port_ != nullptr);

    // First input port is the output of the observed system.
    const int y_size = observed_system_measurement_output_port_->size();
    this->DeclareVectorInputPort("observed_system_output", y_size);

    // Second input port is the input to the observed system (if exists).
    if (observed_system_actuation_input_port_) {
      const auto& port = *observed_system_actuation_input_port_;
      if (port.get_data_type() == kVectorValued) {
        this->DeclareVectorInputPort("observed_system_input", port.size());
      } else if (port.get_data_type() == kAbstractValued) {
        this->DeclareAbstractInputPort("observed_system_input",
                                       /* model value = */ *port.Allocate());
      }
    }

    // Output port is the estimated state.
    this->DeclareVectorOutputPort(
        "estimated_state", num_states_,
        [this](const Context<double>& context, BasicVector<double>* out) {
          out->SetFromVector(this->GetStateEstimate(context));
        },
        {SystemBase::all_state_ticket()});

    // Check W≻0, V≻0, x̂₀, and P̂₀≻0.
    const int w_size = observed_system_process_noise_input_port_ != nullptr
                           ? observed_system_process_noise_input_port_->size()
                           : num_states_;
    DRAKE_THROW_UNLESS(W.rows() == w_size && W.cols() == w_size);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
        W, std::numeric_limits<double>::epsilon(), kSymmetryTolerance));
    const int v_size =
        observed_system_measurement_noise_input_port_ != nullptr
            ? observed_system_measurement_noise_input_port_->size()
            : y_size;
    DRAKE_THROW_UNLESS(V.rows() == v_size && V.cols() == v_size);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
        V, std::numeric_limits<double>::epsilon(), kSymmetryTolerance));

    if (options.initial_state_estimate.has_value()) {
      DRAKE_THROW_UNLESS(options.initial_state_estimate->size() == num_states_);
    }
    if (options.initial_state_covariance.has_value()) {
      DRAKE_THROW_UNLESS(
          options.initial_state_covariance->rows() == num_states_ &&
          options.initial_state_covariance->cols() == num_states_);
      DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
          *options.initial_state_covariance,
          std::numeric_limits<double>::epsilon(), kSymmetryTolerance));
    }

    // Check the unscented transform parameters.
    for (int n : {num_states_, num_states_ + w_size, num_states_ + v_size,
                  num_states_ + w_size + v_size}) {
      internal::CheckUnscentedTransformParams(n, unscented_transform_params_);
    }
  }

  ~UnscentedKalmanFilterBase() override = default;

  const InputPort<double>& get_observed_system_input_input_port()
      const override {
    return this->get_input_port(1);
  }

  const InputPort<double>& get_observed_system_output_input_port()
      const override {
    return this->get_input_port(0);
  }

  const OutputPort<double>& get_estimated_state_output_port() const override {
    return this->get_output_port(0);
  }

  // Returns the observed system context stored in the cache in the context.
  Context<double>& get_mutable_observed_system_context(
      const Context<double>& context) const {
    Context<double>& observed_system_context =
        observed_system_context_cache_entry_
            ->get_mutable_cache_entry_value(context)
            .GetMutableValueOrThrow<Context<double>>();
    observed_system_context.SetTime(context.get_time());
    return observed_system_context;
  }

  // Calculate the dynamics of the observed system:
  // ẋ = f(x,u,w) or x[n+1] = f(x[n],u,w).
  Eigen::VectorXd CalcDynamics(
      Context<double>* observed_system_context,
      const Eigen::Ref<const Eigen::VectorXd>& x, const AbstractValue* u,
      std::optional<Eigen::Ref<const Eigen::VectorXd>> w = std::nullopt) const {
    const bool is_continuous =
        observed_system_context->has_only_continuous_state();

    const bool use_additive_w =
        observed_system_process_noise_input_port_ == nullptr;

    // dyn = f(x,u,w)
    is_continuous ? observed_system_context->SetContinuousState(x)
                  : observed_system_context->SetDiscreteState(x);
    if (u) {
      observed_system_context->FixInputPort(
          observed_system_actuation_input_port_->get_index(), *u);
    }
    if (!use_additive_w) {
      observed_system_process_noise_input_port_->FixValue(
          observed_system_context, w.value());
    }

    Eigen::VectorXd dyn =
        is_continuous
            ? observed_system_->EvalTimeDerivatives(*observed_system_context)
                  .CopyToVector()
            : observed_system_
                  ->EvalUniquePeriodicDiscreteUpdate(*observed_system_context)
                  .value();
    if (use_additive_w && w.has_value()) {
      dyn += w.value();
    }

    return dyn;
  }

  // Calculate the measurement output of the observed system: y = g(x,u,v).
  Eigen::VectorXd CalcMeasurement(
      Context<double>* observed_system_context,
      const Eigen::Ref<const Eigen::VectorXd>& x, const AbstractValue* u,
      std::optional<Eigen::Ref<const Eigen::VectorXd>> v = std::nullopt) const {
    const bool is_continuous =
        observed_system_context->has_only_continuous_state();

    const bool use_additive_v =
        observed_system_measurement_noise_input_port_ == nullptr;

    // y = g(x,u,v)
    is_continuous ? observed_system_context->SetContinuousState(x)
                  : observed_system_context->SetDiscreteState(x);
    if (u) {
      observed_system_context->FixInputPort(
          observed_system_actuation_input_port_->get_index(), *u);
    }
    if (!use_additive_v) {
      observed_system_measurement_noise_input_port_->FixValue(
          observed_system_context, v.value());
    }

    Eigen::VectorXd y = observed_system_measurement_output_port_->Eval(
        *observed_system_context);
    if (use_additive_v && v.has_value()) {
      y += v.value();
    }

    return y;
  }

  // Batch calculate the dynamics of the observed system:
  // ẋᵢ = f(xᵢ,u,wᵢ) or xᵢ[n+1] = f(xᵢ[n],u,wᵢ).
  Eigen::MatrixXd BatchCalcDynamics(
      Context<double>* observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& X, const AbstractValue* u,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> W = std::nullopt) const {
    if (W) {
      DRAKE_ASSERT(X.cols() == W->cols());
    }
    Eigen::MatrixXd Dyn(X.rows(), X.cols());
    for (int i = 0; i < X.cols(); ++i) {
      Dyn.col(i) =
          !W ? CalcDynamics(observed_system_context, X.col(i), u)
             : CalcDynamics(observed_system_context, X.col(i), u, W->col(i));
    }
    return Dyn;
  }

  // Batch calculate the measurement output of the observed system:
  // yᵢ = g(xᵢ,u,vᵢ).
  Eigen::MatrixXd BatchCalcMeasurement(
      Context<double>* observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& X, const AbstractValue* u,
      std::optional<Eigen::Ref<const Eigen::MatrixXd>> V = std::nullopt) const {
    if (V) {
      DRAKE_ASSERT(X.cols() == V->cols());
    }
    Eigen::MatrixXd Y(observed_system_measurement_output_port_->size(),
                      X.cols());
    for (int i = 0; i < X.cols(); ++i) {
      Y.col(i) =
          !V ? CalcMeasurement(observed_system_context, X.col(i), u)
             : CalcMeasurement(observed_system_context, X.col(i), u, V->col(i));
    }
    return Y;
  }

  // The Kalman discrete-time measurement update.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> MeasurementUpdate(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Phat) const {
    auto u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;
    const Eigen::VectorXd& y =
        this->get_observed_system_output_input_port().Eval(context);

    Eigen::VectorXd mu_y;
    Eigen::MatrixXd Sigma_yy, Sigma_xy;
    if (!observed_system_has_measurement_noise_input_port()) {
      auto [X, w_m, W_c] =
          UnscentedTransform(xhat, Phat, unscented_transform_params_);
      auto Y = BatchCalcMeasurement(
          &this->get_mutable_observed_system_context(context), X, u);

      mu_y = Y * w_m;
      Sigma_yy = Y * W_c * Y.transpose() + V_;
      Sigma_xy = X * W_c * Y.transpose();
    } else {
      const int v_size = V_.rows();
      auto [Z, w_m, W_c] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(v_size), V_),
          unscented_transform_params_);

      Eigen::MatrixXd X = Z.topRows(num_states_);
      Eigen::MatrixXd Y = BatchCalcMeasurement(
          &this->get_mutable_observed_system_context(context), X, u,
          Z.bottomRows(v_size));

      mu_y = Y * w_m;
      Sigma_yy = Y * W_c * Y.transpose();
      Sigma_xy = X * W_c * Y.transpose();
    }

    Eigen::MatrixXd K = Sigma_xy * Sigma_yy.inverse();
    Eigen::MatrixXd Phat_new = Phat - K * Sigma_xy.transpose();
    Eigen::VectorXd xhat_new = xhat + K * (y - mu_y);

    return {std::move(xhat_new), std::move(Phat_new)};
  }

  bool observed_system_has_actuation_input_port() const {
    return observed_system_actuation_input_port_ != nullptr;
  }

  bool observed_system_has_process_noise_input_port() const {
    return observed_system_process_noise_input_port_ != nullptr;
  }

  bool observed_system_has_measurement_noise_input_port() const {
    return observed_system_measurement_noise_input_port_ != nullptr;
  }

  const std::shared_ptr<const System<double>> observed_system_;
  const int num_states_;
  const Eigen::MatrixXd W_;
  const Eigen::MatrixXd V_;
  const UnscentedKalmanFilterOptions::UnscentedTransformParameters
      unscented_transform_params_;
  static constexpr double kSymmetryTolerance = 1e-8;

 private:
  const CacheEntry* observed_system_context_cache_entry_{};
  // The following observed system output port will not be nullptr.
  const OutputPort<double>* observed_system_measurement_output_port_{};
  // The following observed system input ports may be nullptr.
  const InputPort<double>* observed_system_actuation_input_port_{};
  const InputPort<double>* observed_system_process_noise_input_port_{};
  const InputPort<double>* observed_system_measurement_noise_input_port_{};
};

// Unscented Kalman filter with discrete-time observed system dynamics and
// discrete-time measurements.
class UnscentedKalmanFilterDD final : public UnscentedKalmanFilterBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnscentedKalmanFilterDD);

  UnscentedKalmanFilterDD(std::shared_ptr<const System<double>> observed_system,
                          const Context<double>& observed_system_context,
                          const Eigen::Ref<const Eigen::MatrixXd>& W,
                          const Eigen::Ref<const Eigen::MatrixXd>& V,
                          const UnscentedKalmanFilterOptions& options)
      : UnscentedKalmanFilterBase(std::move(observed_system),
                                  observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(observed_system_context.has_only_discrete_state() &&
                       observed_system_context.num_discrete_state_groups() ==
                           1);

    // Initial state estimate and covariance.
    Eigen::VectorXd initial_state_estimate =
        options.initial_state_estimate.has_value()
            ? options.initial_state_estimate.value()
            : Eigen::VectorXd::Zero(num_states_);
    Eigen::MatrixXd initial_state_covariance =
        options.initial_state_covariance.has_value()
            ? options.initial_state_covariance.value()
            : Eigen::MatrixXd::Identity(num_states_, num_states_) *
                  std::numeric_limits<double>::epsilon();

    //  We declare only one discrete state containing both the estimated state
    //  and variance.
    this->DeclareDiscreteState(
        internal::ConcatenateVectorAndSquareMatrix<double>(
            initial_state_estimate, initial_state_covariance));

    // Declare periodic update for the state estimate and covaraiance.
    DRAKE_THROW_UNLESS(
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute()
            .has_value());
    auto discrete_attr =
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute().value();
    this->DeclarePeriodicDiscreteUpdateEvent(
        discrete_attr.period_sec(), discrete_attr.offset_sec(),
        &UnscentedKalmanFilterDD::PeriodicDiscreteUpdate);

    if ((options.discrete_measurement_time_period.has_value() &&
         options.discrete_measurement_time_period.value() !=
             discrete_attr.period_sec()) ||
        (options.discrete_measurement_time_offset != 0.0 &&
         options.discrete_measurement_time_offset !=
             discrete_attr.offset_sec())) {
      throw std::logic_error(
          "Discrete-time unscented Kalman filter does not use the "
          "`discrete_measurement_time_period` and "
          "`discrete_measurement_time_offset` options.");
    }
  }

  ~UnscentedKalmanFilterDD() override = default;

  // Implements GaussianStateObserver interface.
  void SetStateEstimateAndCovariance(
      Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& state_estimate,
      const Eigen::Ref<const Eigen::MatrixXd>& state_covariance)
      const override {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(state_estimate.size() == num_states_);
    DRAKE_THROW_UNLESS(state_covariance.rows() == num_states_ &&
                       state_covariance.cols() == num_states_);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
        state_covariance, std::numeric_limits<double>::epsilon(),
        kSymmetryTolerance));
    context->SetDiscreteState(internal::ConcatenateVectorAndSquareMatrix(
        state_estimate, state_covariance));
  }

  // Implements GaussianStateObserver interface.
  Eigen::VectorXd GetStateEstimate(
      const Context<double>& context) const override {
    this->ValidateContext(context);
    return context.get_discrete_state_vector().value().head(num_states_);
  }

  // Implements GaussianStateObserver interface.
  Eigen::MatrixXd GetStateCovariance(
      const Context<double>& context) const override {
    this->ValidateContext(context);
    Eigen::MatrixXd state_covariance(num_states_, num_states_);
    internal::ExtractSquareMatrix<double>(
        context.get_discrete_state_vector().value(), state_covariance);
    return state_covariance;
  }

 private:
  // Callback for discrete update of the state estimate and covariance.
  void PeriodicDiscreteUpdate(const Context<double>& context,
                              DiscreteValues<double>* discrete_state) const {
    // Get the current state estimate and covariance.
    Eigen::VectorXd xhat = GetStateEstimate(context);
    Eigen::MatrixXd Phat = GetStateCovariance(context);

    // Measurement and process update.
    std::tie(xhat, Phat) = MeasurementUpdate(context, xhat, Phat);
    std::tie(xhat, Phat) = ProcessUpdate(context, xhat, Phat);

    discrete_state->set_value(
        internal::ConcatenateVectorAndSquareMatrix<double>(xhat, Phat));
  }

  // The discrete-time Kalman process update.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> ProcessUpdate(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Phat) const {
    auto u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;

    if (!observed_system_has_process_noise_input_port()) {
      auto [X, wm, Wc] =
          UnscentedTransform(xhat, Phat, unscented_transform_params_);

      Eigen::MatrixXd Xnext = BatchCalcDynamics(
          &this->get_mutable_observed_system_context(context), X, u);

      Eigen::VectorXd xhat_next = Xnext * wm;
      Eigen::MatrixXd Phat_next = Xnext * Wc * Xnext.transpose() + W_;
      return {std::move(xhat_next), std::move(Phat_next)};
    } else {
      const int w_size = W_.rows();
      auto [Z, wm, Wc] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(w_size), W_),
          unscented_transform_params_);

      Eigen::MatrixXd Xnext =
          BatchCalcDynamics(&this->get_mutable_observed_system_context(context),
                            Z.topRows(num_states_), u, Z.bottomRows(w_size));

      Eigen::VectorXd xhat_next = Xnext * wm;
      Eigen::MatrixXd Phat_next = Xnext * Wc * Xnext.transpose();
      return {std::move(xhat_next), std::move(Phat_next)};
    }
  }
};

// Base class for unscented Kalman filter with continuous-time obsereved system
// dynamics.
class UnscentedKalmanFilterCont : public UnscentedKalmanFilterBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnscentedKalmanFilterCont);

  UnscentedKalmanFilterCont(
      std::shared_ptr<const System<double>> observed_system,
      const Context<double>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const UnscentedKalmanFilterOptions& options)
      : UnscentedKalmanFilterBase(std::move(observed_system),
                                  observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(observed_system_context.has_only_continuous_state());

    // Initial state estimate and covariance.
    Eigen::VectorXd initial_state_estimate =
        options.initial_state_estimate.has_value()
            ? options.initial_state_estimate.value()
            : Eigen::VectorXd::Zero(num_states_);
    Eigen::MatrixXd initial_state_covariance =
        options.initial_state_covariance.has_value()
            ? options.initial_state_covariance.value()
            : Eigen::MatrixXd::Identity(num_states_, num_states_) *
                  std::numeric_limits<double>::epsilon();

    // Declare estimated state and variance.
    const auto& xc = observed_system_context.get_continuous_state();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    this->DeclareContinuousState(
        BasicVector<double>(internal::ConcatenateVectorAndSquareMatrix<double>(
            initial_state_estimate, initial_state_covariance)),
        num_q, num_v, num_z + num_states_ * num_states_);
  }

  ~UnscentedKalmanFilterCont() override = default;

  // Implements GaussianStateObserver interface.
  void SetStateEstimateAndCovariance(
      Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& state_estimate,
      const Eigen::Ref<const Eigen::MatrixXd>& state_covariance)
      const override {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(state_estimate.size() == num_states_);
    DRAKE_THROW_UNLESS(state_covariance.rows() == num_states_ &&
                       state_covariance.cols() == num_states_);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
        state_covariance, std::numeric_limits<double>::epsilon(),
        kSymmetryTolerance));
    context->SetContinuousState(internal::ConcatenateVectorAndSquareMatrix(
        state_estimate, state_covariance));
  }

  // Implements GaussianStateObserver interface.
  Eigen::VectorXd GetStateEstimate(
      const Context<double>& context) const override {
    this->ValidateContext(context);
    return context.get_continuous_state_vector().CopyToVector().head(
        num_states_);
  }

  // Implements GaussianStateObserver interface.
  Eigen::MatrixXd GetStateCovariance(
      const Context<double>& context) const override {
    this->ValidateContext(context);
    Eigen::MatrixXd state_covariance(num_states_, num_states_);
    internal::ExtractSquareMatrix<double>(
        context.get_continuous_state_vector().CopyToVector(), state_covariance);
    return state_covariance;
  }
};

// Unscented Kalman filter with continuous-time observed system dynamics and
// discrete-time measurements.
class UnscentedKalmanFilterCD final : public UnscentedKalmanFilterCont {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnscentedKalmanFilterCD);

  UnscentedKalmanFilterCD(std::shared_ptr<const System<double>> observed_system,
                          const Context<double>& observed_system_context,
                          const Eigen::Ref<const Eigen::MatrixXd>& W,
                          const Eigen::Ref<const Eigen::MatrixXd>& V,
                          const UnscentedKalmanFilterOptions& options)
      : UnscentedKalmanFilterCont(std::move(observed_system),
                                  observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(options.discrete_measurement_time_period.has_value() &&
                       options.discrete_measurement_time_period.value() > 0);
    this->DeclarePeriodicUnrestrictedUpdateEvent(
        options.discrete_measurement_time_period.value(),
        options.discrete_measurement_time_offset,
        &UnscentedKalmanFilterCD::PeriodicDiscreteUpdate);
  }

  ~UnscentedKalmanFilterCD() override = default;

 private:
  // Callback for computing the continuous-time process update.
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // Get the mutable observed system context from the cache.
    Context<double>* observed_system_context =
        &this->get_mutable_observed_system_context(context);
    // Get u.
    auto u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;

    // Get the current state estimate and covariance.
    Eigen::VectorXd xhat = GetStateEstimate(context);
    Eigen::MatrixXd Phat = GetStateCovariance(context);

    // Compute the derivatives.
    Eigen::VectorXd xhat_dot;
    Eigen::MatrixXd Phat_dot;
    if (!observed_system_has_process_noise_input_port()) {
      auto [X, wm, Wc] =
          UnscentedTransform(xhat, Phat, unscented_transform_params_);
      auto Xdot = BatchCalcDynamics(observed_system_context, X, u);

      xhat_dot = Xdot * wm;
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + W_;
    } else {
      const int w_size = W_.rows();
      auto [Z, wm, Wc] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(w_size), W_),
          unscented_transform_params_);

      Eigen::MatrixXd X = Z.topRows(num_states_);
      Eigen::MatrixXd Xdot = BatchCalcDynamics(observed_system_context, X, u,
                                               Z.bottomRows(w_size));
      Eigen::MatrixXd Xdot_noiseless =
          BatchCalcDynamics(observed_system_context, X, u,
                            Eigen::MatrixXd::Zero(w_size, Z.cols()));

      Eigen::MatrixXd GWG = Xdot * Wc * Xdot.transpose() -
                            Xdot_noiseless * Wc * Xdot_noiseless.transpose();

      xhat_dot = Xdot * wm;
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + GWG;
    }

    derivatives->SetFromVector(
        internal::ConcatenateVectorAndSquareMatrix<double>(xhat_dot, Phat_dot));
  }

  // Callback for computing the discrete-time measurement update.
  void PeriodicDiscreteUpdate(const Context<double>& context,
                              State<double>* state) const {
    // Get the current state estimate and covariance.
    Eigen::VectorXd xhat = GetStateEstimate(context);
    Eigen::MatrixXd Phat = GetStateCovariance(context);

    // Measurement update.
    std::tie(xhat, Phat) = MeasurementUpdate(context, xhat, Phat);

    state->get_mutable_continuous_state().SetFromVector(
        internal::ConcatenateVectorAndSquareMatrix<double>(xhat, Phat));
  }
};

// Unscented Kalman filter with continuous-time observed system dynamics and
// continuous-time measurements.
class UnscentedKalmanFilterCC final : public UnscentedKalmanFilterCont {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnscentedKalmanFilterCC);

  UnscentedKalmanFilterCC(std::shared_ptr<const System<double>> observed_system,
                          const Context<double>& observed_system_context,
                          const Eigen::Ref<const Eigen::MatrixXd>& W,
                          const Eigen::Ref<const Eigen::MatrixXd>& V,
                          const UnscentedKalmanFilterOptions& options)
      : UnscentedKalmanFilterCont(std::move(observed_system),
                                  observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
  }

  ~UnscentedKalmanFilterCC() override = default;

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // Get the mutable observed system context from the cache.
    Context<double>* observed_system_context =
        &this->get_mutable_observed_system_context(context);

    // Get the current state estimate and covariance.
    Eigen::VectorXd xhat = GetStateEstimate(context);
    Eigen::MatrixXd Phat = GetStateCovariance(context);

    // Get u and y.
    auto u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;
    const Eigen::VectorXd& y =
        this->get_observed_system_output_input_port().Eval(context);

    // Compute the derivatives.
    Eigen::VectorXd xhat_dot;
    Eigen::MatrixXd Phat_dot;
    if (!observed_system_has_process_noise_input_port() &&
        !observed_system_has_measurement_noise_input_port()) {
      auto [X, wm, Wc] =
          UnscentedTransform(xhat, Phat, unscented_transform_params_);
      auto Xdot = BatchCalcDynamics(observed_system_context, X, u);
      auto Y = BatchCalcMeasurement(observed_system_context, X, u);

      Eigen::MatrixXd Sigma_xy = X * Wc * Y.transpose();
      Eigen::MatrixXd K = Sigma_xy * V_.inverse();

      xhat_dot = Xdot * wm + K * (y - Y * wm);
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + W_ -
                 K * Sigma_xy.transpose();
    } else if (observed_system_has_process_noise_input_port() &&
               !observed_system_has_measurement_noise_input_port()) {
      const int w_size = W_.rows();
      auto [Z, wm, Wc] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(w_size), W_),
          unscented_transform_params_);

      Eigen::MatrixXd X = Z.topRows(num_states_);
      auto Xdot = BatchCalcDynamics(observed_system_context, X, u,
                                    Z.bottomRows(w_size));
      auto Xdot_noiseless =
          BatchCalcDynamics(observed_system_context, X, u,
                            Eigen::MatrixXd::Zero(w_size, Z.cols()));
      auto Y = BatchCalcMeasurement(observed_system_context, X, u);

      Eigen::MatrixXd GWG = Xdot * Wc * Xdot.transpose() -
                            Xdot_noiseless * Wc * Xdot_noiseless.transpose();

      Eigen::MatrixXd Sigma_xy = X * Wc * Y.transpose();
      Eigen::MatrixXd K = Sigma_xy * V_.inverse();

      xhat_dot = Xdot * wm + K * (y - Y * wm);
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + GWG -
                 K * Sigma_xy.transpose();
    } else if (!observed_system_has_process_noise_input_port() &&
               observed_system_has_measurement_noise_input_port()) {
      const int v_size = V_.rows();
      auto [Z, wm, Wc] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(v_size), V_),
          unscented_transform_params_);

      Eigen::MatrixXd X = Z.topRows(num_states_);
      auto Xdot = BatchCalcDynamics(observed_system_context, X, u);
      auto Y = BatchCalcMeasurement(observed_system_context, X, u,
                                    Z.bottomRows(v_size));
      auto Y_noiseless =
          BatchCalcMeasurement(observed_system_context, X, u,
                               Eigen::MatrixXd::Zero(v_size, Z.cols()));

      Eigen::MatrixXd HVH =
          Y * Wc * Y.transpose() - Y_noiseless * Wc * Y_noiseless.transpose();

      Eigen::MatrixXd Sigma_xy = X * Wc * Y.transpose();
      Eigen::MatrixXd K = Sigma_xy * HVH.inverse();

      xhat_dot = Xdot * wm + K * (y - Y * wm);
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + W_ -
                 K * Sigma_xy.transpose();
    } else {
      const int w_size = W_.rows();
      const int v_size = V_.rows();
      auto [Z, wm, Wc] = UnscentedTransform(
          JointGaussian(xhat, Phat, Eigen::VectorXd::Zero(w_size), W_,
                        Eigen::VectorXd::Zero(v_size), V_),
          unscented_transform_params_);

      Eigen::MatrixXd X = Z.topRows(num_states_);
      auto Xdot = BatchCalcDynamics(observed_system_context, X, u,
                                    Z.middleRows(num_states_, w_size));
      auto Xdot_noiseless =
          BatchCalcDynamics(observed_system_context, X, u,
                            Eigen::MatrixXd::Zero(w_size, Z.cols()));
      auto Y = BatchCalcMeasurement(observed_system_context, X, u,
                                    Z.bottomRows(v_size));
      auto Y_noiseless =
          BatchCalcMeasurement(observed_system_context, X, u,
                               Eigen::MatrixXd::Zero(v_size, Z.cols()));

      Eigen::MatrixXd GWG = Xdot * Wc * Xdot.transpose() -
                            Xdot_noiseless * Wc * Xdot_noiseless.transpose();
      Eigen::MatrixXd HVH =
          Y * Wc * Y.transpose() - Y_noiseless * Wc * Y_noiseless.transpose();

      Eigen::MatrixXd Sigma_xy = X * Wc * Y.transpose();
      Eigen::MatrixXd K = Sigma_xy * HVH.inverse();
      xhat_dot = Xdot * wm + K * (y - Y * wm);
      Phat_dot = X * Wc * Xdot.transpose() + Xdot * Wc * X.transpose() + GWG -
                 K * Sigma_xy.transpose();
    }

    derivatives->SetFromVector(
        internal::ConcatenateVectorAndSquareMatrix<double>(xhat_dot, Phat_dot));
  }
};

}  // namespace

UnscentedKalmanFilterOptions::UnscentedTransformParameters::
    UnscentedTransformParameters(
        double alpha_in, double beta_in,
        std::variant<double, std::function<double(int)>> kappa_in)
    : alpha(alpha_in), beta(beta_in), kappa(kappa_in) {}

std::unique_ptr<GaussianStateObserver<double>> UnscentedKalmanFilter(
    std::shared_ptr<const System<double>> observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const UnscentedKalmanFilterOptions& options) {
  observed_system->ValidateContext(observed_system_context);
  if (observed_system_context.has_only_discrete_state() &&
      observed_system_context.num_discrete_state_groups() == 1) {
    return std::make_unique<UnscentedKalmanFilterDD>(
        std::move(observed_system), observed_system_context, W, V, options);
  } else if (observed_system_context.has_only_continuous_state()) {
    if (options.discrete_measurement_time_period.has_value()) {
      return std::make_unique<UnscentedKalmanFilterCD>(
          std::move(observed_system), observed_system_context, W, V, options);
    } else {
      return std::make_unique<UnscentedKalmanFilterCC>(
          std::move(observed_system), observed_system_context, W, V, options);
    }
  } else {
    throw std::logic_error(
        "UnscentedKalmanFilter only supports systems with either only "
        "continuous states or only discrete states");
  }
}

std::unique_ptr<GaussianStateObserver<double>> UnscentedKalmanFilter(
    const System<double>& observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const UnscentedKalmanFilterOptions& options) {
  return UnscentedKalmanFilter(
      std::shared_ptr<const System<double>>(
          /* managed object = */ std::shared_ptr<void>{},
          /* stored pointer = */ &observed_system),
      observed_system_context, W, V, options);
}

namespace internal {

void CheckUnscentedTransformParams(
    int n,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params) {
  const double alpha = params.alpha;
  const double beta = params.beta;
  const double kappa = (std::get_if<0>(&params.kappa))
                           ? std::get<0>(params.kappa)
                           : std::get<1>(params.kappa)(n);
  if (!(0.001 <= alpha && alpha <= 1)) {
    throw std::logic_error(
        fmt::format("The unscented transform parameters must satisfy: 0.001 ≤ "
                    "α ≤ 1; currently α = {}.",
                    alpha));
  }
  if (!(0 <= beta && beta <= 3)) {
    throw std::logic_error(
        fmt::format("The unscented transform parameters must satisfy: 0 ≤ β ≤ "
                    "3; currently β = {}.",
                    beta));
  }
  if (!(kappa + n > 0)) {
    throw std::logic_error(fmt::format(
        "The unscented transform parameters must satisfy: κ + n > 0; currently "
        "κ = {}, n = {}.",
        kappa, n));
  }
  // TODO(wei-chen): Check `λ/(n + λ) + (1 - α² + β) > 0` if required.
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd>
UnscentedTransform(
    const Eigen::Ref<const Eigen::VectorXd>& mean,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params) {
  const int n = mean.size();
  const double alpha = params.alpha;
  const double beta = params.beta;
  const double kappa = (std::get_if<0>(&params.kappa))
                           ? std::get<0>(params.kappa)
                           : std::get<1>(params.kappa)(n);
  const double lambda = alpha * alpha * (n + kappa) - n;
  const Eigen::MatrixXd L = ((n + lambda) * covariance).llt().matrixL();

  // Compute sigma points.
  const int num_points = 2 * n + 1;
  Eigen::MatrixXd points(mean.size(), num_points);
  points.col(0) = mean;
  for (int i = 0; i < n; ++i) {
    points.col(i + 1) = mean + L.col(i);
    points.col(i + 1 + n) = mean - L.col(i);
  }

  // Compute weights for mean.
  Eigen::VectorXd w_m = 0.5 / (n + lambda) * Eigen::VectorXd::Ones(num_points);
  w_m(0) = lambda / (n + lambda);

  // Compute weights for covariance.
  Eigen::VectorXd w_c = w_m;
  w_c(0) += 1 - alpha * alpha + beta;

  Eigen::MatrixXd I_minus_wm =
      Eigen::MatrixXd::Identity(num_points, num_points).colwise() - w_m;
  Eigen::MatrixXd W_c = I_minus_wm * w_c.asDiagonal() * I_minus_wm.transpose();

  return {points, w_m, W_c};
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd>
UnscentedTransform(
    const std::pair<Eigen::VectorXd, Eigen::MatrixXd>& mean_and_covariance,
    const UnscentedKalmanFilterOptions::UnscentedTransformParameters& params) {
  return UnscentedTransform(mean_and_covariance.first,
                            mean_and_covariance.second, params);
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> JointGaussian(
    const Eigen::Ref<const Eigen::VectorXd>& mean1,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance1,
    const Eigen::Ref<const Eigen::VectorXd>& mean2,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance2) {
  DRAKE_ASSERT(covariance1.rows() == mean1.size() &&
               covariance1.cols() == mean1.size());
  DRAKE_ASSERT(covariance2.rows() == mean2.size() &&
               covariance2.cols() == mean2.size());

  int new_size = mean1.size() + mean2.size();
  Eigen::VectorXd joint_mean(new_size);
  joint_mean << mean1, mean2;

  Eigen::MatrixXd joint_covariance = Eigen::MatrixXd::Zero(new_size, new_size);
  joint_covariance.block(0, 0, mean1.size(), mean1.size()) = covariance1;
  joint_covariance.block(mean1.size(), mean1.size(), mean2.size(),
                         mean2.size()) = covariance2;

  return {joint_mean, joint_covariance};
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> JointGaussian(
    const Eigen::Ref<const Eigen::VectorXd>& mean1,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance1,
    const Eigen::Ref<const Eigen::VectorXd>& mean2,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance2,
    const Eigen::Ref<const Eigen::VectorXd>& mean3,
    const Eigen::Ref<const Eigen::MatrixXd>& covariance3) {
  DRAKE_ASSERT(covariance1.rows() == mean1.size() &&
               covariance1.cols() == mean1.size());
  DRAKE_ASSERT(covariance2.rows() == mean2.size() &&
               covariance2.cols() == mean2.size());
  DRAKE_ASSERT(covariance3.rows() == mean3.size() &&
               covariance3.cols() == mean3.size());

  int new_size = mean1.size() + mean2.size() + mean3.size();
  Eigen::VectorXd joint_mean(new_size);
  joint_mean << mean1, mean2, mean3;

  Eigen::MatrixXd joint_covariance = Eigen::MatrixXd::Zero(new_size, new_size);
  joint_covariance.block(0, 0, mean1.size(), mean1.size()) = covariance1;
  joint_covariance.block(mean1.size(), mean1.size(), mean2.size(),
                         mean2.size()) = covariance2;
  joint_covariance.block(mean1.size() + mean2.size(),
                         mean1.size() + mean2.size(), mean3.size(),
                         mean3.size()) = covariance3;

  return {joint_mean, joint_covariance};
}

}  // namespace internal

}  // namespace estimators
}  // namespace systems
}  // namespace drake
