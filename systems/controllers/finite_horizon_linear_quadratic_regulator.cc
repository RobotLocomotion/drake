#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/trajectories/discrete_time_trajectory.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace systems {
namespace controllers {

using trajectories::DiscreteTimeTrajectory;
using trajectories::PiecewisePolynomial;
using trajectories::Trajectory;

namespace {

// Implements the *time-reversed* Riccati differential equation.  When this
// system evaluates the contained system/cost at time t, it will always replace
// t=-t.
class RiccatiSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RiccatiSystem);

  RiccatiSystem(const System<double>& system, const Context<double>& context,
                const Eigen::Ref<const Eigen::MatrixXd>& Q,
                const Eigen::Ref<const Eigen::MatrixXd>& R,
                const Trajectory<double>& x0, const Trajectory<double>& u0,
                const FiniteHorizonLinearQuadraticRegulatorOptions& options)
      : system_(System<double>::ToAutoDiffXd(system)),
        input_port_(
            system_->get_input_port_selection(options.input_port_index)),
        context_(system_->CreateDefaultContext()),
        num_states_(context_->num_total_states()),
        num_inputs_(input_port_->size()),
        Q_(Q),
        R_(R),
        Rinv_(R.inverse()),
        N_(options.N ? *options.N
                     : Eigen::MatrixXd::Zero(num_states_, num_inputs_)),
        x0_(x0),
        u0_(u0),
        options_(options) {
    if (input_port_->get_data_type() == PortDataType::kAbstractValued) {
      throw std::logic_error(
          "The specified input port is abstract-valued, but "
          "FiniteHorizonLinearQuadraticRegulator only supports vector-valued "
          "input ports.  Did you perhaps forget to pass a non-default "
          "`options.input_port_index`?");
    }
    DRAKE_DEMAND(input_port_->get_data_type() == PortDataType::kVectorValued);

    DRAKE_DEMAND(context_->has_only_continuous_state());
    DRAKE_DEMAND(num_states_ > 0);
    DRAKE_DEMAND(num_inputs_ > 0);

    const double kSymmetryTolerance = 1e-8;
    DRAKE_DEMAND(Q.rows() == num_states_ && Q.cols() == num_states_);
    DRAKE_DEMAND(math::IsPositiveDefinite(Q, 0.0, kSymmetryTolerance));
    DRAKE_DEMAND(R.rows() == num_inputs_ && R.cols() == num_inputs_);
    DRAKE_DEMAND(math::IsPositiveDefinite(
        R, std::numeric_limits<double>::epsilon(), kSymmetryTolerance));

    DRAKE_DEMAND(x0_.rows() == num_states_ && x0_.cols() == 1);
    DRAKE_DEMAND(u0_.rows() == num_inputs_ && u0_.cols() == 1);

    // If use_square_root_method = true, then the state is P (vectorized), then
    // sx, then s0, where Sxx = PP'.  Otherwise, the state is Sxx (vectorized),
    // followed by sx, then s0.
    this->DeclareContinuousState(num_states_ * num_states_ + num_states_ + 1);

    // Initialize autodiff.
    context_->SetTimeStateAndParametersFrom(context);
    system_->FixInputPortsFrom(system, context, context_.get());
  }

  // Implement the (time-reversed) Riccati equation, as described in
  // http://underactuated.mit.edu/lqr.html#finite_horizon_derivation .
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // Unpack the state.
    const Eigen::VectorXd S_vectorized =
        context.get_continuous_state_vector().CopyToVector();
    const Eigen::VectorBlock<const Eigen::VectorXd> sx =
        S_vectorized.segment(num_states_ * num_states_, num_states_);

    // Allocate a vectorized version of the derivatives, and map into it.
    Eigen::VectorXd minus_Sdot_vectorized = S_vectorized;
    Eigen::VectorBlock<Eigen::VectorXd> minus_sxdot =
        minus_Sdot_vectorized.segment(num_states_ * num_states_, num_states_);
    double& minus_s0dot = minus_Sdot_vectorized[S_vectorized.size() - 1];

    // Get (time-varying) linearization of the plant.
    const double system_time = -context.get_time();
    context_->SetTime(system_time);
    auto autodiff_args = math::InitializeAutoDiffTuple(x0_.value(system_time),
                                                       u0_.value(system_time));
    context_->SetContinuousState(std::get<0>(autodiff_args));
    input_port_->FixValue(context_.get(), std::get<1>(autodiff_args));
    const VectorX<AutoDiffXd> autodiff_xdot0 =
        system_->EvalTimeDerivatives(*context_).CopyToVector();
    const Eigen::MatrixXd AB = math::ExtractGradient(autodiff_xdot0);
    const Eigen::Ref<const Eigen::MatrixXd>& A = AB.leftCols(num_states_);
    const Eigen::Ref<const Eigen::MatrixXd>& B = AB.rightCols(num_inputs_);
    const Eigen::VectorXd c =
        math::ExtractValue(autodiff_xdot0) - x0_.EvalDerivative(system_time, 1);

    // Desired trajectories relative to the nominal.
    const Eigen::VectorXd xd0 =
        options_.xd
            ? (options_.xd->value(system_time) - x0_.value(system_time)).eval()
            : Eigen::VectorXd::Zero(num_states_);
    const Eigen::VectorXd ud0 =
        options_.ud
            ? (options_.ud->value(system_time) - u0_.value(system_time)).eval()
            : Eigen::VectorXd::Zero(num_inputs_);

    // Compute the Riccati dynamics.
    Eigen::MatrixXd Sxx;
    if (options_.use_square_root_method) {
      const Eigen::Map<const Eigen::MatrixXd> P(S_vectorized.data(),
                                                num_states_, num_states_);
      Sxx = P * P.transpose();
      const Eigen::MatrixXd PinvT = P.inverse().transpose();
      Eigen::Map<Eigen::MatrixXd> minus_Pdot(minus_Sdot_vectorized.data(),
                                             num_states_, num_states_);
      minus_Pdot = A.transpose() * P -
                   0.5 * (N_ + Sxx * B) * Rinv_ *
                       (B.transpose() * P + N_.transpose() * PinvT) +
                   0.5 * Q_ * PinvT;
    } else {
      Sxx = Eigen::Map<const Eigen::MatrixXd>(S_vectorized.data(), num_states_,
                                              num_states_);
      Eigen::Map<Eigen::MatrixXd> minus_Sxxdot(minus_Sdot_vectorized.data(),
                                               num_states_, num_states_);
      minus_Sxxdot = Sxx * A + A.transpose() * Sxx -
                     (N_ + Sxx * B) * Rinv_ * (N_ + Sxx * B).transpose() + Q_;
    }

    const Eigen::VectorXd qx = -Q_ * xd0 - N_ * ud0;
    const Eigen::VectorXd ru_plus_BTsx =
        -R_ * ud0 - N_.transpose() * xd0 + B.transpose() * sx;
    minus_sxdot = qx - (N_ + Sxx * B) * Rinv_ * ru_plus_BTsx +
                  A.transpose() * sx + Sxx * c;
    minus_s0dot = xd0.dot(Q_ * xd0) + ud0.dot(R_ * ud0) +
                  2 * xd0.dot(N_ * ud0) -
                  ru_plus_BTsx.dot(Rinv_ * ru_plus_BTsx) + 2.0 * sx.dot(c);

    derivatives->SetFromVector(minus_Sdot_vectorized);
  }

  // TODO(russt): It would be more elegant to think of S and K as an output of
  // the RiccatiSystem, but we don't (yet) have a way to get the dense
  // integration results of an output port.

  // Takes a time-reversed dense solution for this Riccati system, applies the
  // time-reversal, and unwraps the vectorized solution into its matrix
  // components.
  void MakeSTrajectories(
      const PiecewisePolynomial<double>& riccati_solution,
      FiniteHorizonLinearQuadraticRegulatorResult* result) const {
    auto Sxx = std::make_unique<PiecewisePolynomial<double>>(
        riccati_solution.Block(0, 0, num_states_ * num_states_, 1));
    Sxx->Reshape(num_states_, num_states_);
    if (options_.use_square_root_method) {
      result->S = std::make_unique<PiecewisePolynomial<double>>(
          *Sxx * (Sxx->Transpose()));
    } else {
      result->S = std::move(Sxx);
    }
    result->sx = std::make_unique<PiecewisePolynomial<double>>(
        riccati_solution.Block(num_states_ * num_states_, 0, num_states_, 1));
    result->s0 =
        std::make_unique<PiecewisePolynomial<double>>(riccati_solution.Block(
            num_states_ * num_states_ + num_states_, 0, 1, 1));
  }

  // Given a result structure already populated with the S and sx trajectories
  // (already time-reversed and reshaped), constructs the K and k0 trajectories.
  // Since derivatives are not easily available for B(t), we form a piecewise
  // cubic polynomial approximation of the elements of Kx and k0 using Lagrange
  // interpolating polynomials.  Cubic seems natural, as the default "dense
  // output" from the integrators is a cubic hermite spline; if we had
  // considered K as an output from a simulation it seems natural to interpolate
  // K with the same order approximation.
  void MakeKTrajectories(
      FiniteHorizonLinearQuadraticRegulatorResult* result) const {
    const std::vector<double>& breaks =
        dynamic_cast<PiecewisePolynomial<double>&>(*result->S)
            .get_segment_times();

    auto Kx = std::make_unique<PiecewisePolynomial<double>>();
    auto k0 = std::make_unique<PiecewisePolynomial<double>>();

    // TODO(russt): I believe there is a sampling theorem that should give me
    // the optimal sampling times for the fixed-degree polynomial over the
    // finite time interval. Find it and use it here.
    const std::vector<double> scaled_sample_times = {0.0, 1. / 3., 2. / 3., 1.};
    const int num_samples = scaled_sample_times.size();
    std::vector<double> times(num_samples);
    std::vector<Eigen::MatrixXd> Kx_samples(num_samples);
    std::vector<Eigen::MatrixXd> k0_samples(num_samples);
    for (size_t i = 0; i < breaks.size() - 1; ++i) {
      int j_start = 0;
      if (i > 0) {
        // Then my first sample is the last sample from the previous segment.
        times[0] = times[num_samples - 1];
        Kx_samples[0] = Kx_samples[num_samples - 1];
        k0_samples[0] = k0_samples[num_samples - 1];
        j_start = 1;
      }

      for (int j = j_start; j < num_samples; ++j) {
        const double time = scaled_sample_times[j] * breaks[i + 1] +
                            (1.0 - scaled_sample_times[j]) * breaks[i];
        times[j] = time;

        // Compute B.
        context_->SetTime(time);
        context_->SetContinuousState(x0_.value(time).cast<AutoDiffXd>().eval());
        input_port_->FixValue(context_.get(),
                              math::InitializeAutoDiff(u0_.value(time)));
        const VectorX<AutoDiffXd> autodiff_xdot0 =
            system_->EvalTimeDerivatives(*context_).CopyToVector();
        const Eigen::MatrixXd B = math::ExtractGradient(autodiff_xdot0);

        // Desired trajectories relative to the nominal.
        const Eigen::VectorXd xd0 =
            options_.xd ? (options_.xd->value(time) - x0_.value(time)).eval()
                        : Eigen::VectorXd::Zero(num_states_);
        const Eigen::VectorXd ud0 =
            options_.ud ? (options_.ud->value(time) - u0_.value(time)).eval()
                        : Eigen::VectorXd::Zero(num_inputs_);

        Kx_samples[j] = Rinv_ * (N_ + result->S->value(time) * B).transpose();
        k0_samples[j] =
            -ud0 + Rinv_ * (-N_.transpose() * xd0 +
                            B.transpose() * result->sx->value(time));
      }
      Kx->ConcatenateInTime(
          PiecewisePolynomial<double>::LagrangeInterpolatingPolynomial(
              times, Kx_samples));
      k0->ConcatenateInTime(
          PiecewisePolynomial<double>::LagrangeInterpolatingPolynomial(
              times, k0_samples));
    }

    result->K = std::move(Kx);
    result->k0 = std::move(k0);
  }

 private:
  const std::unique_ptr<const System<AutoDiffXd>> system_;
  const InputPort<AutoDiffXd>* const input_port_;

  // Note: Use a mutable context here to avoid reallocating on every call to
  // e.g. CalcTimeDerivatives. WARNING: this means that evaluation of the
  // RiccatiSystem is not thread safe, but since the implementation is internal
  // to this file, there are no entry points that would allow multi-threaded
  // execution with the same RiccatiSystem instance.
  // Note: This context_ is for system_, not `this`.
  const std::unique_ptr<Context<AutoDiffXd>> context_;

  const int num_states_;
  const int num_inputs_;
  const Eigen::Ref<const Eigen::MatrixXd>& Q_;
  const Eigen::Ref<const Eigen::MatrixXd>& R_;
  const Eigen::MatrixXd Rinv_;
  const Eigen::MatrixXd N_;
  const Trajectory<double>& x0_;
  const Trajectory<double>& u0_;
  const FiniteHorizonLinearQuadraticRegulatorOptions& options_;
};

FiniteHorizonLinearQuadraticRegulatorResult
ContinuousTimeFiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  // Most argument consistency checks are handled by RiccatiSystem, but that
  // System doesn't need to understand the time range, so we perform (only)
  // those checks here.
  system.ValidateContext(context);
  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(system.num_input_ports() > 0);
  DRAKE_DEMAND(tf > t0);
  const int num_states = context.num_total_states();
  std::unique_ptr<PiecewisePolynomial<double>> x0;
  if (options.x0) {
    DRAKE_DEMAND(options.x0->start_time() <= t0);
    DRAKE_DEMAND(options.x0->end_time() >= tf);
    DRAKE_DEMAND(options.x0->has_derivative());
  } else {
    // Make a constant trajectory with the state from context.
    x0 = std::make_unique<PiecewisePolynomial<double>>(
        context.get_continuous_state_vector().CopyToVector());
  }
  if (options.use_square_root_method && !options.Qf) {
    throw std::logic_error(
        "options.Qf is required when options.use_square_root_method is set to "
        "'true'.");
  }

  std::unique_ptr<PiecewisePolynomial<double>> u0;
  if (options.u0) {
    DRAKE_DEMAND(options.u0->start_time() <= t0);
    DRAKE_DEMAND(options.u0->end_time() >= tf);
  } else {
    // Make a constant trajectory with the input from context.
    u0 = std::make_unique<PiecewisePolynomial<double>>(
        system.get_input_port_selection(options.input_port_index)
            ->Eval(context));
  }

  RiccatiSystem riccati(system, context, Q, R, options.x0 ? *options.x0 : *x0,
                        options.u0 ? *options.u0 : *u0, options);

  // Simulator doesn't support integrating backwards in time, so simulate the
  // time-reversed Riccati equation from -tf to -t0, and reverse it after the
  // fact.
  Simulator<double> simulator(riccati);
  ApplySimulatorConfig(options.simulator_config, &simulator);

  // Set the initial conditions (the Riccati solution at tf).
  if (options.Qf) {
    const double kSymmetryTolerance = 1e-8;
    DRAKE_DEMAND(options.Qf->rows() == num_states &&
                 options.Qf->cols() == num_states);
    Eigen::VectorXd S_vectorized =
        Eigen::VectorXd::Zero(num_states * num_states + num_states + 1);
    if (options.use_square_root_method) {
      // We require Qf = Qfᵀ ≻ 0.
      DRAKE_DEMAND(math::IsSymmetric(*options.Qf, kSymmetryTolerance));
      const double kEigenvalueTolerance = 1e-8;

      // We use the SelfAdjointEigenSolver both to check PSD and to get the
      // sqrt. The eigenvalue check is adopted from math::IsPositiveDefinite.
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(*options.Qf);
      DRAKE_THROW_UNLESS(eigensolver.info() == Eigen::Success);
      const double max_abs_eigenvalue =
          eigensolver.eigenvalues().cwiseAbs().maxCoeff();
      if (eigensolver.eigenvalues().minCoeff() <
          kEigenvalueTolerance * std::max(1., max_abs_eigenvalue)) {
        throw std::logic_error(
            "The SQRT method solution to the Riccati equation requires that Qf "
            "is strictly positive definite.  To use a positive semi-definite "
            "Qf, set `options.use_square_root_method` to `false`.");
      }
      Eigen::Map<Eigen::MatrixXd> P(S_vectorized.data(), num_states,
                                    num_states);
      P = eigensolver.operatorSqrt();
    } else {
      // Qf = Qfᵀ ≽ 0 is sufficient.
      DRAKE_DEMAND(
          math::IsPositiveDefinite(*options.Qf, 0.0, kSymmetryTolerance));
      Eigen::Map<Eigen::MatrixXd> Sxx(S_vectorized.data(), num_states,
                                      num_states);
      Sxx = *options.Qf;
    }
    if (options.xd) {
      Eigen::VectorBlock<Eigen::VectorXd> sx =
          S_vectorized.segment(num_states * num_states, num_states);
      double& s0 = S_vectorized[S_vectorized.size() - 1];
      const Eigen::VectorXd xd0 =
          options.xd->value(tf) -
          (options.x0 ? options.x0->value(tf) : x0->value(tf));
      sx = -(*options.Qf) * xd0;
      s0 = xd0.dot((*options.Qf) * xd0);
    }
    simulator.get_mutable_context().SetContinuousState(S_vectorized);
  }

  simulator.get_mutable_context().SetTime(-tf);
  simulator.Initialize();
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.StartDenseIntegration();

  simulator.AdvanceTo(-t0);

  FiniteHorizonLinearQuadraticRegulatorResult result;
  result.x0 = options.x0 ? options.x0->Clone() : std::move(x0);
  result.u0 = options.u0 ? options.u0->Clone() : std::move(u0);
  std::unique_ptr<PiecewisePolynomial<double>> riccati_solution =
      integrator.StopDenseIntegration();
  riccati_solution->ReverseTime();
  riccati.MakeSTrajectories(*riccati_solution, &result);
  riccati.MakeKTrajectories(&result);

  return result;
}

FiniteHorizonLinearQuadraticRegulatorResult
DiscreteTimeFiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  system.ValidateContext(context);
  DRAKE_THROW_UNLESS(context.has_only_discrete_state() &&
                     context.num_discrete_state_groups() == 1);
  DRAKE_THROW_UNLESS(system.num_input_ports() > 0);
  DRAKE_THROW_UNLESS(tf > t0);

  if (options.use_square_root_method) {
    // TODO(wei-chen): Find if possible and implement square root method.
    throw std::logic_error(
        "Discrete-time finite-horizon LQR does not yet support the square root "
        "method.");
  }

  const Trajectory<double>* x0_traj;
  PiecewisePolynomial<double> x0_constant_traj;
  if (options.x0 != nullptr) {
    DRAKE_THROW_UNLESS(options.x0->start_time() <= t0);
    DRAKE_THROW_UNLESS(options.x0->end_time() >= tf);
    x0_traj = options.x0;
  } else {
    // Make a constant trajectory with the state from context.
    x0_constant_traj = PiecewisePolynomial<double>(
        context.get_discrete_state_vector().CopyToVector());
    x0_traj = &x0_constant_traj;
  }

  const Trajectory<double>* u0_traj;
  PiecewisePolynomial<double> u0_constant_traj;
  if (options.u0 != nullptr) {
    DRAKE_THROW_UNLESS(options.u0->start_time() <= t0);
    DRAKE_THROW_UNLESS(options.u0->end_time() >= tf);
    u0_traj = options.u0;
  } else {
    // Make a constant trajectory with the input from context.
    const InputPort<double>* input_port =
        system.get_input_port_selection(options.input_port_index);
    DRAKE_THROW_UNLESS(input_port != nullptr);
    u0_constant_traj = PiecewisePolynomial<double>(input_port->Eval(context));
    u0_traj = &u0_constant_traj;
  }

  // Create an autodiff version of the system.
  std::unique_ptr<System<AutoDiffXd>> autodiff_system =
      drake::systems::System<double>::ToAutoDiffXd(system);

  const InputPort<AutoDiffXd>* input_port =
      autodiff_system->get_input_port_selection(options.input_port_index);
  DRAKE_THROW_UNLESS(input_port != nullptr);

  // Initialize autodiff.
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);
  autodiff_system->FixInputPortsFrom(system, context, autodiff_context.get());

  const int num_states = autodiff_context->num_total_states();
  const int num_inputs = input_port->size();

  // Check arguments.
  if (input_port->get_data_type() == PortDataType::kAbstractValued) {
    throw std::logic_error(
        "The specified input port is abstract-valued, but "
        "FiniteHorizonLinearQuadraticRegulator only supports vector-valued "
        "input ports.  Did you perhaps forget to pass a non-default "
        "`input_port_index` argument?");
  }
  DRAKE_THROW_UNLESS(input_port->get_data_type() ==
                     PortDataType::kVectorValued);

  DRAKE_THROW_UNLESS(num_states > 0);
  DRAKE_THROW_UNLESS(num_inputs > 0);

  const double kSymmetryTolerance = 1e-8;
  DRAKE_THROW_UNLESS(Q.rows() == num_states && Q.cols() == num_states);
  DRAKE_THROW_UNLESS(math::IsPositiveDefinite(Q, 0.0, kSymmetryTolerance));

  DRAKE_THROW_UNLESS(R.rows() == num_inputs && R.cols() == num_inputs);
  DRAKE_THROW_UNLESS(math::IsPositiveDefinite(
      R, std::numeric_limits<double>::epsilon(), kSymmetryTolerance));

  const Eigen::MatrixXd Qf =
      options.Qf.value_or(Eigen::MatrixXd::Zero(num_states, num_states));
  DRAKE_THROW_UNLESS(Qf.rows() == num_states && Qf.cols() == num_states);
  DRAKE_THROW_UNLESS(math::IsPositiveDefinite(Q, 0.0, kSymmetryTolerance));

  const Eigen::MatrixXd N =
      options.N.value_or(Eigen::MatrixXd::Zero(num_states, num_inputs));
  DRAKE_THROW_UNLESS(N.rows() == num_states && N.cols() == num_inputs);

  // Compute the discrete times.
  const double kTimeCompTolerance = 1e-10;
  std::vector<double> times;
  DRAKE_THROW_UNLESS(
      system.GetUniquePeriodicDiscreteUpdateAttribute().has_value());
  auto discrete = system.GetUniquePeriodicDiscreteUpdateAttribute().value();
  DRAKE_THROW_UNLESS(std::abs(std::remainder(t0 - discrete.offset_sec(),
                                             discrete.period_sec())) <=
                     kTimeCompTolerance);
  DRAKE_THROW_UNLESS(std::abs(std::remainder(tf - discrete.offset_sec(),
                                             discrete.period_sec())) <=
                     kTimeCompTolerance);
  times.resize(std::round((tf - t0) / discrete.period_sec() + 1));
  for (int n = 0; n < ssize(times); ++n) {
    times[n] = t0 + discrete.period_sec() * n;
  }

  // Compute discrete-time finite horizon LQR,
  // K[n], k0[n] depends on S[n+1], sx[n+1], s0[n+1].
  std::vector<Eigen::MatrixXd> K(times.size() - 1), k0(times.size() - 1),
      S(times.size()), sx(times.size()), s0(times.size());

  Eigen::MatrixXd xd0f =
      (options.xd != nullptr)
          ? (options.xd->value(tf) - x0_traj->value(tf)).eval()
          : Eigen::VectorXd::Zero(num_states);
  S.back() = Qf;
  sx.back() = -Qf * xd0f;
  s0.back() = xd0f.transpose() * Qf * xd0f;

  for (int n = ssize(times) - 2; n >= 0; --n) {
    double t = times[n];

    Eigen::MatrixXd x0 = x0_traj->value(t);
    Eigen::MatrixXd u0 = u0_traj->value(t);

    Eigen::MatrixXd x0_next = x0_traj->value(times[n + 1]);

    // Desired trajectories relative to the nominal.
    Eigen::MatrixXd xd0 =
        ((options.xd != nullptr) ? options.xd->value(t) : x0) - x0;
    Eigen::MatrixXd ud0 =
        ((options.ud != nullptr) ? options.ud->value(t) : u0) - u0;

    auto autodiff_args = math::InitializeAutoDiffTuple(x0, u0);

    input_port->FixValue(autodiff_context.get(), std::get<1>(autodiff_args));

    autodiff_context->SetTime(t);
    autodiff_context->SetDiscreteState(std::get<0>(autodiff_args));

    auto autodiff_x0_next =
        autodiff_system->EvalUniquePeriodicDiscreteUpdate(*autodiff_context)
            .value();

    const Eigen::MatrixXd AB = math::ExtractGradient(autodiff_x0_next);

    Eigen::MatrixXd A = AB.leftCols(num_states);
    Eigen::MatrixXd B = AB.rightCols(num_inputs);
    Eigen::MatrixXd c = math::ExtractValue(autodiff_x0_next) - x0_next;

    Eigen::MatrixXd qx = -Q * xd0 - N * ud0;
    Eigen::MatrixXd q0 =
        xd0.transpose() * Q * xd0 + 2 * xd0.transpose() * N * ud0;
    Eigen::MatrixXd ru = -R * ud0 - N.transpose() * xd0;
    Eigen::MatrixXd r0 = ud0.transpose() * R * ud0;

    Eigen::MatrixXd R_BSB_inv = (R + B.transpose() * S[n + 1] * B).inverse();
    Eigen::MatrixXd N_BSA = N.transpose() + B.transpose() * S[n + 1] * A;
    Eigen::MatrixXd ru_BSc_Bsx =
        ru + B.transpose() * S[n + 1] * c + B.transpose() * sx[n + 1];

    K[n] = R_BSB_inv * N_BSA;
    k0[n] = R_BSB_inv * ru_BSc_Bsx;

    S[n] = Q + A.transpose() * S[n + 1] * A -
           N_BSA.transpose() * R_BSB_inv * N_BSA;
    sx[n] = qx + A.transpose() * S[n + 1] * c + A.transpose() * sx[n + 1] -
            N_BSA.transpose() * R_BSB_inv * ru_BSc_Bsx;
    s0[n] = c.transpose() * S[n + 1] * c + 2 * c.transpose() * sx[n + 1] +
            s0[n + 1] + q0 + r0 -
            ru_BSc_Bsx.transpose() * R_BSB_inv * ru_BSc_Bsx;
  }  // for loop

  FiniteHorizonLinearQuadraticRegulatorResult result;
  result.x0 = x0_traj->Clone();
  result.u0 = u0_traj->Clone();
  result.S = DiscreteTimeTrajectory(times, std::move(S));
  result.sx = DiscreteTimeTrajectory(times, std::move(sx));
  result.s0 = DiscreteTimeTrajectory(times, std::move(s0));
  times.pop_back();
  result.K = DiscreteTimeTrajectory(times, std::move(K));
  result.k0 = DiscreteTimeTrajectory(times, std::move(k0));

  return result;
}

}  // namespace

FiniteHorizonLinearQuadraticRegulatorResult
FiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  system.ValidateContext(context);
  if (context.has_only_continuous_state()) {
    return ContinuousTimeFiniteHorizonLinearQuadraticRegulator(
        system, context, t0, tf, Q, R, options);
  } else if (context.has_only_discrete_state() &&
             context.num_discrete_state_groups() == 1) {
    return DiscreteTimeFiniteHorizonLinearQuadraticRegulator(
        system, context, t0, tf, Q, R, options);
  } else {
    throw std::logic_error(
        "FiniteHorizonLinearQuadraticRegulator only supports systems with "
        "either only continuous states or only discrete states");
  }
}

namespace {

// Implements the (time-varying) linear controller described by a
// FiniteHorizonLinearQuadraticRegulatorResult.
// TODO(russt): Consider removing this class entirely and using
// TrajectoryAffineSystem instead, once we support enough Trajectory algebra
// (including adding and multiplying PiecewisePolynomial trajectories with
// different segment times).
template <typename T>
class Controller final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Controller);

  // Constructs the controller, and assumes ownership of the required components
  // of the FiniteHorizonLinearQuadraticRegulatorResult.  This is appropriate,
  // since the class is internal to this .cc file and is only ever constructed
  // via a Make*Regulator() workflow which will discard the result structure.
  Controller(std::unique_ptr<trajectories::Trajectory<double>> x0,
             std::unique_ptr<trajectories::Trajectory<double>> u0,
             std::unique_ptr<trajectories::Trajectory<double>> K,
             std::unique_ptr<trajectories::Trajectory<double>> k0)
      : LeafSystem<T>(SystemTypeTag<Controller>()),
        x0_(std::move(x0)),
        u0_(std::move(u0)),
        K_(std::move(K)),
        k0_(std::move(k0)) {
    this->DeclareVectorInputPort("plant_state", K_->cols());
    this->DeclareVectorOutputPort(
        "command", K_->rows(), &Controller::CalcOutput,
        {this->time_ticket(), this->input_port_ticket(InputPortIndex(0))});
  }

  // Scalar-type converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Controller(const Controller<U>& other)
      : Controller(other.x0_->Clone(), other.u0_->Clone(), other.K_->Clone(),
                   other.k0_->Clone()) {}

 private:
  template <typename U>
  friend class Controller;

  // Calculate the (time-varying) output.
  void CalcOutput(const Context<T>& context, BasicVector<T>* output) const {
    // Note: The stored trajectories are always double, even when T != double.
    const double t = ExtractDoubleOrThrow(context.get_time());
    const auto& x = this->get_input_port(0).Eval(context);
    output->get_mutable_value() =
        u0_->value(t) - K_->value(t) * (x - x0_->value(t)) - k0_->value(t);
  }

  std::unique_ptr<trajectories::Trajectory<double>> x0_;
  std::unique_ptr<trajectories::Trajectory<double>> u0_;
  std::unique_ptr<trajectories::Trajectory<double>> K_;
  std::unique_ptr<trajectories::Trajectory<double>> k0_;
};

}  // namespace

std::unique_ptr<System<double>> MakeFiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(system, context, t0, tf, Q, R,
                                            options);
  return std::make_unique<Controller<double>>(
      std::move(result.x0), std::move(result.u0), std::move(result.K),
      std::move(result.k0));
}

}  // namespace controllers

}  // namespace systems
}  // namespace drake
