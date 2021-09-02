#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace systems {
namespace controllers {

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

    // State is Sxx (vectorized), followed by sx, then s0.
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
    const Eigen::Map<const Eigen::MatrixXd> Sxx(S_vectorized.data(),
                                                num_states_, num_states_);
    const Eigen::VectorBlock<const Eigen::VectorXd> sx =
        S_vectorized.segment(num_states_ * num_states_, num_states_);

    // Allocate a vectorized version of the derivatives, and map into it.
    Eigen::VectorXd minus_Sdot_vectorized = S_vectorized;
    Eigen::Map<Eigen::MatrixXd> minus_Sxxdot(minus_Sdot_vectorized.data(),
                                             num_states_, num_states_);
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

    // Compute the Riccati dynamics
    minus_Sxxdot = Sxx * A + A.transpose() * Sxx -
                   (N_ + Sxx * B) * Rinv_ * (N_ + Sxx * B).transpose() + Q_;

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

  // TODO(russt): It would be more elegant to think of K as an output of the
  // RiccatiSystem, but we don't (yet) have a way to get the dense integration
  // results of an output port.

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

}  // namespace

FiniteHorizonLinearQuadraticRegulatorResult
FiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  // Most argument consistency checks are handled by RiccatiSystem, but that
  // System doesn't need to understand the time range, so we perform (only)
  // those checks here.
  system.ValidateContext(context);
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

  // Set the initial conditions (the Riccati solution at tf).
  if (options.Qf) {
    const double kSymmetryTolerance = 1e-8;
    DRAKE_DEMAND(options.Qf->rows() == num_states &&
                 options.Qf->cols() == num_states);
    DRAKE_DEMAND(
        math::IsPositiveDefinite(*options.Qf, 0.0, kSymmetryTolerance));
    Eigen::VectorXd S_vectorized =
        Eigen::VectorXd::Zero(num_states * num_states + num_states + 1);
    Eigen::Map<Eigen::MatrixXd> Sxx(S_vectorized.data(), num_states,
                                    num_states);
    Sxx = *options.Qf;
    if (options.xd) {
      Eigen::VectorBlock<Eigen::VectorXd> sx =
          S_vectorized.segment(num_states * num_states, num_states);
      double& s0 = S_vectorized[S_vectorized.size() - 1];
      const Eigen::VectorXd xd0 = options.xd->value(tf) - x0->value(tf);
      sx = -(*options.Qf) * xd0;
      s0 = xd0.dot((*options.Qf) * xd0);
    }
    simulator.get_mutable_context().SetContinuousState(S_vectorized);
  }

  simulator.get_mutable_context().SetTime(-tf);
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.StartDenseIntegration();

  simulator.AdvanceTo(-t0);

  FiniteHorizonLinearQuadraticRegulatorResult result;
  std::unique_ptr<PiecewisePolynomial<double>> S =
      integrator.StopDenseIntegration();
  S->ReverseTime();
  auto Sxx = std::make_unique<PiecewisePolynomial<double>>(
      S->Block(0, 0, num_states * num_states, 1));
  Sxx->Reshape(num_states, num_states);
  result.S = std::move(Sxx);
  result.sx = std::make_unique<PiecewisePolynomial<double>>(
      S->Block(num_states * num_states, 0, num_states, 1));
  result.s0 = std::make_unique<PiecewisePolynomial<double>>(
      S->Block(num_states * num_states + num_states, 0, 1, 1));
  result.x0 = options.x0 ? options.x0->Clone() : std::move(x0);
  result.u0 = options.u0 ? options.u0->Clone() : std::move(u0);
  riccati.MakeKTrajectories(&result);

  return result;
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Controller)

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
