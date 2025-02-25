#include "drake/systems/estimators/extended_kalman_filter.h"

#include <limits>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace systems {
namespace estimators {

namespace {

// GaussianStateObserver<double>        [Inheritance map]
//           ↓
// ExtendedKalmanFilterBase -------→ ExtendedKalmanFilterCont
//           ↓                        ↓                    ↓
// ExtendedKalmanFilterDD  ExtendedKalmanFilterCD  ExtendedKalmanFilterCC

// Base class for all extended Kalman filters.
class ExtendedKalmanFilterBase : public GaussianStateObserver<double> {
 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExtendedKalmanFilterBase);

  struct MemoryArena;

  ExtendedKalmanFilterBase(
      std::shared_ptr<const System<AutoDiffXd>> observed_system,
      const Context<AutoDiffXd>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options)
      : observed_system_(std::move(observed_system)),
        num_states_(observed_system_context.num_total_states()),
        use_sqrt_method_(options.use_square_root_method),
        W_(W),
        V_(V),
        Wsqrt_(use_sqrt_method_ ? Eigen::MatrixXd(W.llt().matrixL())
                                : Eigen::MatrixXd()),
        Vsqrt_(use_sqrt_method_ ? Eigen::MatrixXd(V.llt().matrixL())
                                : Eigen::MatrixXd()) {
    observed_system_->ValidateContext(observed_system_context);
    DRAKE_THROW_UNLESS(num_states_ > 0);  // Or else we don't need an observer.
    DRAKE_THROW_UNLESS(
        observed_system_context.has_only_continuous_state() ||
        (observed_system_context.has_only_discrete_state() &&
         observed_system_context.num_discrete_state_groups() == 1));

    // Copy the observed system context into a cache entry.
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

    // Check W≽0, V≻0, x̂₀, and P̂₀≽0. If use_sqrt_method_, check W≻0 and P̂₀≻0.
    const int w_size = observed_system_process_noise_input_port_ != nullptr
                           ? observed_system_process_noise_input_port_->size()
                           : num_states_;
    DRAKE_THROW_UNLESS(W.rows() == w_size && W.cols() == w_size);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
        W, !use_sqrt_method_ ? 0.0 : std::numeric_limits<double>::epsilon(),
        kSymmetryTolerance));
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
          !use_sqrt_method_ ? 0.0 : std::numeric_limits<double>::epsilon(),
          kSymmetryTolerance));
    } else if (use_sqrt_method_) {
      throw std::logic_error(
          "options.initial_state_covariance is required when "
          "options.use_square_root_method is set to 'true'.");
    }

    // Setup a memory arena in a cache entry.
    memory_arena_cache_entry_ = &this->DeclareCacheEntry(
        "memory arena",
        ValueProducer(MemoryArena(num_states_, y_size, w_size, v_size),
                      &ValueProducer::NoopCalc),
        {SystemBase::nothing_ticket()});
  }

  ~ExtendedKalmanFilterBase() override = default;

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
  Context<AutoDiffXd>& get_mutable_observed_system_context(
      const Context<double>& context) const {
    Context<AutoDiffXd>& observed_system_context =
        observed_system_context_cache_entry_
            ->get_mutable_cache_entry_value(context)
            .GetMutableValueOrThrow<Context<AutoDiffXd>>();
    observed_system_context.SetTime(context.get_time());
    return observed_system_context;
  }

  // Returns the memory arena stored in the cache in the context.
  MemoryArena& get_mutable_memory_arena(const Context<double>& context) const {
    return memory_arena_cache_entry_->get_mutable_cache_entry_value(context)
        .GetMutableValueOrThrow<MemoryArena>();
  }

  // Memory arena struct holding some preallocated memory.
  struct MemoryArena {
    MemoryArena(int x_size, int y_size, int w_size, int v_size) {
      const int num_derivs = x_size + std::max(w_size, v_size);
      x_ad = math::InitializeAutoDiff(Eigen::VectorXd(x_size), num_derivs);
      w_ad = math::InitializeAutoDiff(Eigen::VectorXd::Zero(w_size), num_derivs,
                                      num_derivs - w_size);
      v_ad = math::InitializeAutoDiff(Eigen::VectorXd::Zero(v_size), num_derivs,
                                      num_derivs - v_size);

      dyn_ad = math::InitializeAutoDiff(Eigen::VectorXd(x_size), num_derivs);
      dyn_value.resize(x_size);
      dyn_gradient.resize(x_size, num_derivs);

      y_ad = math::InitializeAutoDiff(Eigen::VectorXd(y_size), num_derivs);
      y_value.resize(y_size);
      y_gradient.resize(y_size, num_derivs);
    }

    void set_x_ad(const Eigen::Ref<const Eigen::VectorXd>& x) {
      DRAKE_ASSERT(this->x_ad.size() == x.size());
      for (int i = 0; i < x.size(); ++i) {
        this->x_ad[i].value() = x[i];
      }
    }

    Eigen::VectorX<AutoDiffXd> x_ad;
    Eigen::VectorX<AutoDiffXd> w_ad;
    Eigen::VectorX<AutoDiffXd> v_ad;

    Eigen::VectorX<AutoDiffXd> dyn_ad;
    Eigen::VectorXd dyn_value;
    Eigen::MatrixXd dyn_gradient;

    Eigen::VectorX<AutoDiffXd> y_ad;
    Eigen::VectorXd y_value;
    Eigen::MatrixXd y_gradient;
  };

  // Calculates the dynamics of the observed system:
  // ẋ = f(x,u,w=0) or x[n+1] = f(x[n],u,w=0).
  // Also calculates the Jacobians: ∂f/∂x and ∂f/∂w.
  std::tuple<Eigen::Ref<const Eigen::VectorXd>,
             Eigen::Ref<const Eigen::MatrixXd>,
             Eigen::Ref<const Eigen::MatrixXd>>
  CalcDynamicsAndLinearize(Context<AutoDiffXd>* observed_system_context,
                           const Eigen::Ref<const Eigen::VectorXd>& x,
                           const AbstractValue* u,
                           MemoryArena* memory_arena) const {
    MemoryArena& m = *memory_arena;

    const bool is_continuous =
        observed_system_context->has_only_continuous_state();
    const bool use_additive_w =
        observed_system_process_noise_input_port_ == nullptr;
    const int w_size = W_.rows();

    // dyn = f(x,u,w=0)
    m.set_x_ad(x);
    is_continuous ? observed_system_context->SetContinuousState(m.x_ad)
                  : observed_system_context->SetDiscreteState(m.x_ad);

    if (observed_system_has_actuation_input_port()) {
      DRAKE_ASSERT(u != nullptr);
      if (observed_system_actuation_input_port_->get_data_type() ==
          kVectorValued) {
        Eigen::VectorX<AutoDiffXd> u_vec =
            u->get_value<BasicVector<double>>().value();
        observed_system_actuation_input_port_->FixValue(observed_system_context,
                                                        u_vec);
      } else {
        observed_system_actuation_input_port_->FixValue(observed_system_context,
                                                        *u);
      }
    }

    if (!use_additive_w) {
      observed_system_process_noise_input_port_->FixValue(
          observed_system_context, m.w_ad);
    }

    m.dyn_ad =
        is_continuous
            ? observed_system_->EvalTimeDerivatives(*observed_system_context)
                  .CopyToVector()
            : observed_system_
                  ->EvalUniquePeriodicDiscreteUpdate(*observed_system_context)
                  .value();
    if (use_additive_w) {
      m.dyn_ad += m.w_ad;
    }

    math::ExtractValue(m.dyn_ad, &m.dyn_value);
    Eigen::Ref<const Eigen::VectorXd> dyn = m.dyn_value;

    // A = ∂f/∂x(x,u,w), G = ∂f/∂w(x,u,w).
    math::ExtractGradient(m.dyn_ad, std::nullopt, &m.dyn_gradient);
    Eigen::Ref<const Eigen::MatrixXd> A = m.dyn_gradient.leftCols(num_states_);
    Eigen::Ref<const Eigen::MatrixXd> G = m.dyn_gradient.rightCols(w_size);

    return {dyn, A, G};
  }

  // Calculates the measurement output of the observed system: y = g(x,u,v=0).
  // Also calculates the Jacobians: ∂g/∂x and ∂g/∂v.
  std::tuple<Eigen::Ref<const Eigen::VectorXd>,
             Eigen::Ref<const Eigen::MatrixXd>,
             Eigen::Ref<const Eigen::MatrixXd>>
  CalcMeasurementAndLinearize(Context<AutoDiffXd>* observed_system_context,
                              const Eigen::Ref<const Eigen::VectorXd>& x,
                              const AbstractValue* u,
                              MemoryArena* memory_arena) const {
    MemoryArena& m = *memory_arena;

    const bool is_continuous =
        observed_system_context->has_only_continuous_state();
    const bool use_additive_v =
        observed_system_measurement_noise_input_port_ == nullptr;
    const int v_size = V_.rows();

    // y = g(x,u,v=0)
    m.set_x_ad(x);
    is_continuous ? observed_system_context->SetContinuousState(m.x_ad)
                  : observed_system_context->SetDiscreteState(m.x_ad);

    if (observed_system_has_actuation_input_port()) {
      DRAKE_ASSERT(u != nullptr);
      if (observed_system_actuation_input_port_->get_data_type() ==
          kVectorValued) {
        Eigen::VectorX<AutoDiffXd> u_vec =
            u->get_value<BasicVector<double>>().value();
        observed_system_actuation_input_port_->FixValue(observed_system_context,
                                                        u_vec);
      } else {
        observed_system_actuation_input_port_->FixValue(observed_system_context,
                                                        *u);
      }
    }

    if (!use_additive_v) {
      observed_system_measurement_noise_input_port_->FixValue(
          observed_system_context, m.v_ad);
    }

    m.y_ad = observed_system_measurement_output_port_->Eval(
        *observed_system_context);
    if (use_additive_v) {
      m.y_ad += m.v_ad;
    }

    math::ExtractValue(m.y_ad, &m.y_value);
    Eigen::Ref<const Eigen::VectorXd> y = m.y_value;

    // C = ∂g/∂x(x,u,v), H = ∂g/∂v(x,u,v).
    math::ExtractGradient(m.y_ad, std::nullopt, &m.y_gradient);
    Eigen::Ref<const Eigen::MatrixXd> C = m.y_gradient.leftCols(num_states_);
    Eigen::Ref<const Eigen::MatrixXd> H = m.y_gradient.rightCols(v_size);

    return {y, C, H};
  }

  // The Kalman discrete-time measurement update.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> MeasurementUpdate(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Phat) const {
    MemoryArena& m = this->get_mutable_memory_arena(context);

    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;
    const Eigen::VectorXd& y =
        this->get_observed_system_output_input_port().Eval(context);

    // ŷ = g(x̂,u,v=0), C = ∂g/∂x, H = ∂g/∂v
    auto [yhat, C, H] = CalcMeasurementAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);

    // K = P̂C'(CP̂C' + HVH')⁻¹
    Eigen::MatrixXd K = (C * Phat * C.transpose() + H * V_ * H.transpose())
                            .llt()
                            .solve(C * Phat)
                            .transpose();

    // x̂ ← x̂ + K (y − ŷ)
    Eigen::VectorXd xhat_new = xhat + K * (y - yhat);

    // P̂ ← (I - KC) P̂
    Eigen::MatrixXd Phat_new =
        (Eigen::MatrixXd::Identity(num_states_, num_states_) - K * C) * Phat;

    return {std::move(xhat_new), std::move(Phat_new)};
  }

  // The Kalman discrete-time measurement update using square root method.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> MeasurementUpdateSqrt(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Shat /* P̂ = ŜŜ' */) const {
    DRAKE_ASSERT(use_sqrt_method_);
    const int v_size = V_.rows();
    const int y_size = observed_system_measurement_output_port_->size();
    if (!(v_size >= y_size)) {
      auto [xhat_new, Phat_new] =
          MeasurementUpdate(context, xhat, Shat * Shat.transpose());
      Eigen::MatrixXd Shat_new = Phat_new.llt().matrixL();
      return {std::move(xhat_new), std::move(Shat_new)};
    }

    MemoryArena& m = this->get_mutable_memory_arena(context);

    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;
    const Eigen::VectorXd& y =
        this->get_observed_system_output_input_port().Eval(context);

    // ŷ = g(x̂,u,v=0), C = ∂g/∂x, H = ∂g/∂v
    auto [yhat, C, H] = CalcMeasurementAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);

    // The following algorithm requires v_size≥y_size (M must be a tall matrix).
    Eigen::MatrixXd M(v_size + num_states_, y_size + num_states_);
    M.topLeftCorner(v_size, y_size) = Vsqrt_.transpose() * H.transpose();
    M.topRightCorner(v_size, num_states_).setZero();
    M.bottomLeftCorner(num_states_, y_size) = (C * Shat).transpose();
    M.bottomRightCorner(num_states_, num_states_) = Shat.transpose();

    Eigen::MatrixXd R = Eigen::HouseholderQR<Eigen::MatrixXd>(M)
                            .matrixQR()
                            .topRows(y_size + num_states_)
                            .triangularView<Eigen::Upper>();
    // R₁'R₁ = CP̂C' + HVH'
    // R₁'R₂ = CP̂
    // R₂'R₂ + R₃'R₃ = P̂
    auto R1 = R.topLeftCorner(y_size, y_size).triangularView<Eigen::Upper>();
    auto R2 = R.topRightCorner(y_size, num_states_);
    auto R3 = R.block(y_size, y_size, num_states_, num_states_)
                  .triangularView<Eigen::Upper>();

    // K = P̂C'(CP̂C' + HVH')⁻¹
    Eigen::MatrixXd K = R1.solve(R2).transpose();

    // x̂ ← x̂ + K (y − ŷ)
    Eigen::VectorXd xhat_new = xhat + K * (y - yhat);

    // P̂ ← P̂ - P̂C'(CP̂C' + HVH')⁻¹CP̂
    Eigen::MatrixXd Shat_new = R3.transpose();

    return {std::move(xhat_new), std::move(Shat_new)};
  }

  bool observed_system_has_actuation_input_port() const {
    return observed_system_actuation_input_port_ != nullptr;
  }

  const std::shared_ptr<const System<AutoDiffXd>> observed_system_;
  const int num_states_;
  const bool use_sqrt_method_;
  const Eigen::MatrixXd W_;
  const Eigen::MatrixXd V_;
  const Eigen::MatrixXd Wsqrt_;
  const Eigen::MatrixXd Vsqrt_;
  static constexpr double kSymmetryTolerance = 1e-8;

 private:
  const CacheEntry* observed_system_context_cache_entry_{};
  const CacheEntry* memory_arena_cache_entry_{};
  // The following observed system output port will not be nullptr.
  const OutputPort<AutoDiffXd>* observed_system_measurement_output_port_{};
  // The following observed system input ports may be nullptr.
  const InputPort<AutoDiffXd>* observed_system_actuation_input_port_{};
  const InputPort<AutoDiffXd>* observed_system_process_noise_input_port_{};
  const InputPort<AutoDiffXd>* observed_system_measurement_noise_input_port_{};
};

// Extended Kalman filter with discrete-time observed system dynamics and
// discrete-time measurements.
class ExtendedKalmanFilterDD final : public ExtendedKalmanFilterBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExtendedKalmanFilterDD);

  ExtendedKalmanFilterDD(
      std::shared_ptr<const System<AutoDiffXd>> observed_system,
      const Context<AutoDiffXd>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options)
      : ExtendedKalmanFilterBase(std::move(observed_system),
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
            : Eigen::MatrixXd::Zero(num_states_, num_states_);

    //  We declare only one discrete state containing both the estimated state
    //  and variance.
    if (!use_sqrt_method_) {
      this->DeclareDiscreteState(
          internal::ConcatenateVectorAndSquareMatrix<double>(
              initial_state_estimate, initial_state_covariance));
    } else {
      DRAKE_THROW_UNLESS(options.initial_state_covariance.has_value());
      DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
          initial_state_covariance, std::numeric_limits<double>::epsilon(),
          kSymmetryTolerance));
      this->DeclareDiscreteState(internal::ConcatenateVectorAndLowerTriMatrix(
          initial_state_estimate, initial_state_covariance.llt().matrixL()));
    }

    // Declare periodic update for the state estimate and covaraiance.
    DRAKE_THROW_UNLESS(
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute()
            .has_value());
    auto discrete_attr =
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute().value();
    this->DeclarePeriodicDiscreteUpdateEvent(
        discrete_attr.period_sec(), discrete_attr.offset_sec(),
        &ExtendedKalmanFilterDD::PeriodicDiscreteUpdate);

    if ((options.discrete_measurement_time_period.has_value() &&
         options.discrete_measurement_time_period.value() !=
             discrete_attr.period_sec()) ||
        (options.discrete_measurement_time_offset != 0.0 &&
         options.discrete_measurement_time_offset !=
             discrete_attr.offset_sec())) {
      throw std::logic_error(
          "Discrete-time extended Kalman filter does not use the "
          "`discrete_measurement_time_period` and "
          "`discrete_measurement_time_offset` options.");
    }
  }

  ~ExtendedKalmanFilterDD() override = default;

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
        state_covariance,
        !use_sqrt_method_ ? 0.0 : std::numeric_limits<double>::epsilon(),
        kSymmetryTolerance));
    if (!use_sqrt_method_) {
      context->SetDiscreteState(internal::ConcatenateVectorAndSquareMatrix(
          state_estimate, state_covariance));
    } else {
      context->SetDiscreteState(internal::ConcatenateVectorAndLowerTriMatrix(
          state_estimate, state_covariance.llt().matrixL()));
    }
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
    if (!use_sqrt_method_) {
      Eigen::MatrixXd state_covariance(num_states_, num_states_);
      internal::ExtractSquareMatrix<double>(
          context.get_discrete_state_vector().value(), state_covariance);
      return state_covariance;
    } else {
      Eigen::MatrixXd state_covariance_sqrt(num_states_, num_states_);
      internal::ExtractLowerTriMatrix<double>(
          context.get_discrete_state_vector().value(), state_covariance_sqrt);
      return state_covariance_sqrt * state_covariance_sqrt.transpose();
    }
  }

 private:
  // Callback for discrete update of the state estimate and covariance.
  void PeriodicDiscreteUpdate(const Context<double>& context,
                              DiscreteValues<double>* discrete_state) const {
    if (!use_sqrt_method_) {
      // Get the current state estimate and covariance.
      Eigen::VectorXd xhat = GetStateEstimate(context);
      Eigen::MatrixXd Phat = GetStateCovariance(context);

      // Measurement and process update.
      std::tie(xhat, Phat) = MeasurementUpdate(context, xhat, Phat);
      std::tie(xhat, Phat) = ProcessUpdate(context, xhat, Phat);

      discrete_state->set_value(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat, Phat));
    } else {
      // Get the current state estimate and covariance sqrt.
      Eigen::VectorXd xhat = GetStateEstimate(context);
      Eigen::MatrixXd Shat(num_states_, num_states_);
      internal::ExtractLowerTriMatrix<double>(
          context.get_discrete_state_vector().value(), Shat);

      // Measurement and process update.
      std::tie(xhat, Shat) = MeasurementUpdateSqrt(context, xhat, Shat);
      std::tie(xhat, Shat) = ProcessUpdateSqrt(context, xhat, Shat);

      discrete_state->set_value(
          internal::ConcatenateVectorAndLowerTriMatrix<double>(xhat, Shat));
    }
  }

  // The discrete-time Kalman process update.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> ProcessUpdate(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Phat) const {
    DRAKE_ASSERT(!use_sqrt_method_);
    MemoryArena& m = this->get_mutable_memory_arena(context);

    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;

    // x̂[n+1] = f(x̂[n],u,w=0), A = ∂f/∂x, G = ∂f/∂w
    auto [xhat_next, A, G] = CalcDynamicsAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);

    // P̂[n+1] = AP̂A' + GWG'
    Eigen::MatrixXd Phat_next =
        A * Phat * A.transpose() + G * W_ * G.transpose();

    return {std::move(xhat_next), std::move(Phat_next)};
  }

  // The discrete-time Kalman process update using square root method.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> ProcessUpdateSqrt(
      const Context<double>& context,
      const Eigen::Ref<const Eigen::VectorXd>& xhat,
      const Eigen::Ref<const Eigen::MatrixXd>& Shat /* P̂ = ŜŜ' */) const {
    DRAKE_ASSERT(use_sqrt_method_);
    MemoryArena& m = this->get_mutable_memory_arena(context);

    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;

    // x̂[n+1] = f(x̂[n],u,w=0), A = ∂f/∂x, G = ∂f/∂w
    auto [xhat_next, A, G] = CalcDynamicsAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);

    const int w_size = G.cols();
    Eigen::MatrixXd M(num_states_ + w_size, num_states_);
    M << (A * Shat).transpose(), (G * Wsqrt_).transpose();

    // R'R = AP̂A' + GWG'
    Eigen::MatrixXd R = Eigen::HouseholderQR<Eigen::MatrixXd>(M)
                            .matrixQR()
                            .topRows(num_states_)
                            .triangularView<Eigen::Upper>();

    // P̂[n+1] = AP̂A' + GWG'
    Eigen::MatrixXd Shat_next = R.transpose();

    return {std::move(xhat_next), std::move(Shat_next)};
  }
};

// Base class for extended Kalman filter with continuous-time obsereved system
// dynamics.
class ExtendedKalmanFilterCont : public ExtendedKalmanFilterBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExtendedKalmanFilterCont);

  ExtendedKalmanFilterCont(
      std::shared_ptr<const System<AutoDiffXd>> observed_system,
      const Context<AutoDiffXd>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options)
      : ExtendedKalmanFilterBase(std::move(observed_system),
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
            : Eigen::MatrixXd::Zero(num_states_, num_states_);

    // Declare estimated state and covariance.
    const auto& xc = observed_system_context.get_continuous_state();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    if (!use_sqrt_method_) {
      this->DeclareContinuousState(
          BasicVector<double>(
              internal::ConcatenateVectorAndSquareMatrix<double>(
                  initial_state_estimate, initial_state_covariance)),
          num_q, num_v, num_z + num_states_ * num_states_);
    } else {
      DRAKE_THROW_UNLESS(options.initial_state_covariance.has_value());
      DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
          initial_state_covariance, std::numeric_limits<double>::epsilon(),
          kSymmetryTolerance));
      // For continuous-time, the covariance needs to be stored in a square
      // matrix because the covariance time derivative is not lower triangle.
      this->DeclareContinuousState(
          BasicVector<double>(
              internal::ConcatenateVectorAndSquareMatrix<double>(
                  initial_state_estimate,
                  Eigen::MatrixXd(initial_state_covariance.llt().matrixL()))),
          num_q, num_v, num_z + num_states_ * num_states_);
    }
  }

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
        state_covariance,
        !use_sqrt_method_ ? 0.0 : std::numeric_limits<double>::epsilon(),
        kSymmetryTolerance));
    if (!use_sqrt_method_) {
      context->SetContinuousState(internal::ConcatenateVectorAndSquareMatrix(
          state_estimate, state_covariance));
    } else {
      context->SetContinuousState(
          internal::ConcatenateVectorAndSquareMatrix<double>(
              state_estimate,
              Eigen::MatrixXd(state_covariance.llt().matrixL())));
    }
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
    if (!use_sqrt_method_) {
      Eigen::MatrixXd state_covariance(num_states_, num_states_);
      internal::ExtractSquareMatrix<double>(
          context.get_continuous_state_vector().CopyToVector(),
          state_covariance);
      return state_covariance;
    } else {
      Eigen::MatrixXd state_covariance_sqrt = GetStateCovarianceSqrt(context);
      return state_covariance_sqrt * state_covariance_sqrt.transpose();
    }
  }

  ~ExtendedKalmanFilterCont() override = default;

 protected:
  Eigen::MatrixXd GetStateCovarianceSqrt(const Context<double>& context) const {
    DRAKE_ASSERT(use_sqrt_method_);
    Eigen::MatrixXd state_covariance_sqrt(num_states_, num_states_);
    internal::ExtractSquareMatrix<double>(
        context.get_continuous_state_vector().CopyToVector(),
        state_covariance_sqrt);
    return state_covariance_sqrt;
  }
};

// Extended Kalman filter with continuous-time observed system dynamics and
// discrete-time measurements.
class ExtendedKalmanFilterCD final : public ExtendedKalmanFilterCont {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExtendedKalmanFilterCD);

  ExtendedKalmanFilterCD(
      std::shared_ptr<const System<AutoDiffXd>> observed_system,
      const Context<AutoDiffXd>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options)
      : ExtendedKalmanFilterCont(std::move(observed_system),
                                 observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(options.discrete_measurement_time_period.has_value() &&
                       options.discrete_measurement_time_period.value() > 0);
    this->DeclarePeriodicUnrestrictedUpdateEvent(
        options.discrete_measurement_time_period.value(),
        options.discrete_measurement_time_offset,
        &ExtendedKalmanFilterCD::PeriodicDiscreteUpdate);
  }

  ~ExtendedKalmanFilterCD() override = default;

 private:
  // Callback for computing the continuous-time process update.
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    MemoryArena& m = this->get_mutable_memory_arena(context);

    // Get u.
    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;

    // Get the current state estimate.
    Eigen::VectorXd xhat = GetStateEstimate(context);

    // dx̂/dt = f(x̂,u,w=0), A = ∂f/∂x, G = ∂f/∂w
    auto [xhat_dot, A, G] = CalcDynamicsAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);

    if (!use_sqrt_method_) {
      // Get the current state covariance.
      Eigen::MatrixXd Phat = GetStateCovariance(context);

      // dP̂/dt = AP̂ + P̂A' + GWG'
      Eigen::MatrixXd Phat_dot =
          A * Phat + Phat * A.transpose() + G * W_ * G.transpose();

      derivatives->SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat_dot,
                                                             Phat_dot));
    } else {
      // Get the current state covariance sqrt.
      Eigen::MatrixXd Shat = GetStateCovarianceSqrt(context);

      // dŜ/dt = AŜ + GWG'Ŝ⁻ᵀ/2
      Eigen::MatrixXd Shat_dot =
          A * Shat + 0.5 * G * W_ * G.transpose() * Shat.transpose().inverse();

      derivatives->SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat_dot,
                                                             Shat_dot));
    }
  }

  // Callback for computing the discrete-time measurement update.
  void PeriodicDiscreteUpdate(const Context<double>& context,
                              State<double>* state) const {
    if (!use_sqrt_method_) {
      // Get the current state estimate and covariance.
      Eigen::VectorXd xhat = GetStateEstimate(context);
      Eigen::MatrixXd Phat = GetStateCovariance(context);

      // Measurement update.
      std::tie(xhat, Phat) = MeasurementUpdate(context, xhat, Phat);

      state->get_mutable_continuous_state().SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat, Phat));
    } else {
      // Get the current state estimate and covariance sqrt.
      Eigen::VectorXd xhat = GetStateEstimate(context);
      Eigen::MatrixXd Shat = GetStateCovarianceSqrt(context);

      // Measurement update.
      std::tie(xhat, Shat) = MeasurementUpdateSqrt(context, xhat, Shat);

      state->get_mutable_continuous_state().SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat, Shat));
    }
  }
};

// Extended Kalman filter with continuous-time observed system dynamics and
// continuous-time measurements.
class ExtendedKalmanFilterCC final : public ExtendedKalmanFilterCont {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExtendedKalmanFilterCC);

  ExtendedKalmanFilterCC(
      std::shared_ptr<const System<AutoDiffXd>> observed_system,
      const Context<AutoDiffXd>& observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& W,
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      const ExtendedKalmanFilterOptions& options)
      : ExtendedKalmanFilterCont(std::move(observed_system),
                                 observed_system_context, W, V, options) {
    DRAKE_THROW_UNLESS(!options.discrete_measurement_time_period.has_value());
  }

  ~ExtendedKalmanFilterCC() override = default;

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    MemoryArena& m = this->get_mutable_memory_arena(context);

    // Get u and y.
    const AbstractValue* u =
        observed_system_has_actuation_input_port()
            ? &this->get_observed_system_input_input_port().Eval<AbstractValue>(
                  context)
            : nullptr;
    const Eigen::VectorXd& y =
        this->get_observed_system_output_input_port().Eval(context);

    // Get the current state estimate.
    Eigen::VectorXd xhat = GetStateEstimate(context);

    // x̂dot = f(x̂,u,w=0), A = ∂f/∂x, G = ∂f/∂w
    auto [xhatdot, A, G] = CalcDynamicsAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);
    // ŷ = g(x̂,u,v=0), C = ∂g/∂x, H = ∂g/∂v
    auto [yhat, C, H] = CalcMeasurementAndLinearize(
        &this->get_mutable_observed_system_context(context), xhat, u, &m);
    Eigen::MatrixXd HVH_inv = (H * V_ * H.transpose()).inverse();

    if (!use_sqrt_method_) {
      // Get the current state covariance.
      Eigen::MatrixXd Phat = GetStateCovariance(context);

      // dx̂/dt = f(x̂,u) + P̂C'(HVH')⁻¹(y - g(x̂,u))
      Eigen::MatrixXd PhatC = Phat * C.transpose();
      Eigen::VectorXd xhat_deriv = xhatdot + PhatC * HVH_inv * (y - yhat);

      // dP̂/dt = AP̂ + P̂A' + GWG' - P̂C'(HVH')⁻¹CP̂
      Eigen::MatrixXd Phat_deriv = A * Phat + Phat * A.transpose() +
                                   G * W_ * G.transpose() -
                                   PhatC * HVH_inv * PhatC.transpose();

      derivatives->SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat_deriv,
                                                             Phat_deriv));
    } else {
      // Get the current state covariance.
      Eigen::MatrixXd Shat = GetStateCovarianceSqrt(context);

      // dx̂/dt = f(x̂,u) + P̂C'(HVH')⁻¹(y - g(x̂,u))
      Eigen::MatrixXd PhatC_HVH_inv =
          Shat * Shat.transpose() * C.transpose() * HVH_inv;
      Eigen::VectorXd xhat_deriv = xhatdot + PhatC_HVH_inv * (y - yhat);

      // dŜ/dt = AŜ + GWG'Ŝ⁻ᵀ/2 - P̂C'(HVH')⁻¹CŜ/2
      Eigen::MatrixXd Shat_deriv =
          A * Shat + 0.5 * G * W_ * G.transpose() * Shat.transpose().inverse() -
          0.5 * PhatC_HVH_inv * C * Shat;

      derivatives->SetFromVector(
          internal::ConcatenateVectorAndSquareMatrix<double>(xhat_deriv,
                                                             Shat_deriv));
    }
  }
};

}  // namespace

std::unique_ptr<GaussianStateObserver<double>> ExtendedKalmanFilter(
    std::shared_ptr<const System<AutoDiffXd>> observed_system,
    const Context<AutoDiffXd>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const ExtendedKalmanFilterOptions& options) {
  observed_system->ValidateContext(observed_system_context);
  if (observed_system_context.has_only_discrete_state() &&
      observed_system_context.num_discrete_state_groups() == 1) {
    return std::make_unique<ExtendedKalmanFilterDD>(
        std::move(observed_system), observed_system_context, W, V, options);
  } else if (observed_system_context.has_only_continuous_state()) {
    if (options.discrete_measurement_time_period.has_value()) {
      return std::make_unique<ExtendedKalmanFilterCD>(
          std::move(observed_system), observed_system_context, W, V, options);
    } else {
      return std::make_unique<ExtendedKalmanFilterCC>(
          std::move(observed_system), observed_system_context, W, V, options);
    }
  } else {
    throw std::logic_error(
        "ExtendedKalmanFilter only supports systems with either only "
        "continuous states or only discrete states");
  }
}

std::unique_ptr<GaussianStateObserver<double>> ExtendedKalmanFilter(
    const System<double>& observed_system,
    const Context<double>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V,
    const ExtendedKalmanFilterOptions& options) {
  observed_system.ValidateContext(observed_system_context);
  auto observed_system_ad = observed_system.ToAutoDiffXd();
  auto observed_system_context_ad = observed_system_ad->CreateDefaultContext();
  observed_system_context_ad->SetTimeStateAndParametersFrom(
      observed_system_context);
  return ExtendedKalmanFilter(std::move(observed_system_ad),
                              *observed_system_context_ad, W, V, options);
}

namespace internal {

template <typename T>
Eigen::VectorX<T> ConcatenateVectorAndSquareMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& vector,
    const Eigen::Ref<const Eigen::MatrixX<T>>& square_matrix) {
  DRAKE_ASSERT(square_matrix.rows() == vector.size());
  DRAKE_ASSERT(square_matrix.cols() == vector.size());
  Eigen::VectorX<T> concatenated(vector.size() + square_matrix.size());
  concatenated << vector, Eigen::Map<const Eigen::VectorX<T>>(
                              square_matrix.data(), square_matrix.size());
  return concatenated;
}

template <typename T>
Eigen::VectorX<T> ConcatenateVectorAndLowerTriMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& vector,
    const Eigen::Ref<const Eigen::MatrixX<T>>& lower_tri_matrix) {
  DRAKE_ASSERT((lower_tri_matrix.template triangularView<Eigen::StrictlyUpper>()
                    .toDenseMatrix()
                    .array() == 0.0)
                   .all());
  return ConcatenateVectorAndLowerTriMatrix(
      vector, lower_tri_matrix.template triangularView<Eigen::Lower>());
}

template <typename T>
void ExtractSquareMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& concatenated,
    Eigen::Ref<Eigen::MatrixX<T>> square_matrix) {
  const int size = square_matrix.rows();
  DRAKE_ASSERT(square_matrix.rows() == square_matrix.cols());
  DRAKE_ASSERT(concatenated.size() == size + size * size);
  square_matrix = Eigen::Map<const Eigen::MatrixX<T>>(
      concatenated.data() + size, size, size);
}

template <typename T>
void ExtractLowerTriMatrix(
    const Eigen::Ref<const Eigen::VectorX<T>>& concatenated,
    Eigen::Ref<Eigen::MatrixX<T>> lower_tri_matrix) {
  const int size = lower_tri_matrix.rows();
  DRAKE_ASSERT(lower_tri_matrix.rows() == lower_tri_matrix.cols());
  DRAKE_ASSERT(concatenated.size() == size + size * (size + 1) / 2);
  lower_tri_matrix.setZero();
  int idx = size;
  for (int j = 0; j < size; ++j) {
    for (int i = j; i < size; ++i) {
      lower_tri_matrix(i, j) = concatenated(idx++);
    }
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&ConcatenateVectorAndSquareMatrix<T>,
     static_cast<
         Eigen::VectorX<T> (*)(const Eigen::Ref<const Eigen::VectorX<T>>&,
                               const Eigen::Ref<const Eigen::MatrixX<T>>&)>(
         &ConcatenateVectorAndLowerTriMatrix<T>),
     &ExtractSquareMatrix<T>, &ExtractLowerTriMatrix<T>));

}  // namespace internal

}  // namespace estimators
}  // namespace systems
}  // namespace drake
