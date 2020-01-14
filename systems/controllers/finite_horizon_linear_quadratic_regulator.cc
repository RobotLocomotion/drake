#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"  // noqa

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {

using trajectories::PiecewisePolynomial;

namespace internal {

// Implements the *time-reversed* Riccati differential equation.  When this
// system evaluates the contained system/cost at time t, it will always replace
// t=-t.
class RiccatiSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RiccatiSystem);

  RiccatiSystem(const System<double>& system, const Context<double>& context,
                const Eigen::Ref<const Eigen::MatrixXd>& Q,
                const Eigen::Ref<const Eigen::MatrixXd>& R,
                const FiniteHorizonLinearQuadraticRegulatorOptions& options)
      : system_(System<double>::ToAutoDiffXd(system)),
        input_port_(
            system_->get_input_port_selection(options.input_port_index)),
        context_(system_->CreateDefaultContext()),
        Q_(Q),
        Rinv_(R.inverse()),
        options_(options),
        num_states_(context.num_total_states()),
        num_inputs_(input_port_->size()) {
    DRAKE_DEMAND(input_port_->get_data_type() == PortDataType::kVectorValued);

    DRAKE_DEMAND(context.has_only_continuous_state());
    DRAKE_DEMAND(num_states_ > 0);
    DRAKE_DEMAND(num_inputs_ > 0);

    const double kSymmetryTolerance = 1e-8;
    DRAKE_DEMAND(Q.rows() == num_states_ && Q.cols() == num_states_);
    DRAKE_DEMAND(math::IsPositiveDefinite(Q, 0.0, kSymmetryTolerance));
    DRAKE_DEMAND(R.rows() == num_inputs_ && R.cols() == num_inputs_);
    DRAKE_DEMAND(math::IsPositiveDefinite(
        R, std::numeric_limits<double>::epsilon(), kSymmetryTolerance));

    if (options.x0) {
      DRAKE_DEMAND(options.x0->rows() == num_states_ &&
                   options.x0->cols() == 1);
    }
    if (options.u0) {
      DRAKE_DEMAND(options.u0->rows() == num_inputs_ &&
                   options.u0->cols() == 1);
    }

    this->DeclareContinuousState(num_states_ * num_states_);

    // Initialize autodiff.
    context_->SetTimeStateAndParametersFrom(context);
    system_->FixInputPortsFrom(system, context, context_.get());
  }

  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    const Eigen::VectorXd S_vectorized =
        context.get_continuous_state_vector().CopyToVector();
    const Eigen::Map<const Eigen::MatrixXd> S(S_vectorized.data(), num_states_,
                                              num_states_);

    const double time = -context.get_time();
    context_->SetTime(time);

    // Get (time-varying) linearization of the plant.
    auto autodiff_args = math::initializeAutoDiffTuple(
        options_.x0 ? options_.x0->value(time)
                    : Eigen::VectorXd::Zero(num_states_),
        options_.u0 ? options_.u0->value(time)
                    : Eigen::VectorXd::Zero(num_inputs_));
    context_->SetContinuousState(std::get<0>(autodiff_args));
    input_port_->FixValue(context_.get(), std::get<1>(autodiff_args));

    const VectorX<AutoDiffXd> autodiff_xdot0 =
        system_->EvalTimeDerivatives(*context_).CopyToVector();

    const Eigen::VectorXd xdot0 = math::autoDiffToValueMatrix(autodiff_xdot0);
    const Eigen::MatrixXd AB = math::autoDiffToGradientMatrix(autodiff_xdot0);
    const Eigen::Ref<const Eigen::MatrixXd>& A = AB.leftCols(num_states_);
    const Eigen::Ref<const Eigen::MatrixXd>& B = AB.rightCols(num_inputs_);

    const Eigen::MatrixXd minus_Sdot =
        S * A + A.transpose() * S - S * B * Rinv_ * B.transpose() * S + Q_;

    const Eigen::Map<const Eigen::VectorXd> minus_Sdot_vectorized(
        minus_Sdot.data(), num_states_ * num_states_);
    derivatives->SetFromVector(minus_Sdot_vectorized);
  }

  // Extract the control gains, K, from the solution trajectory.
  // TODO(russt): It would be more elegant to think of this as an output of the
  // RiccatiSystem, but we don't (yet) have a way to get the dense integration
  // results of an output port.
  std::unique_ptr<PiecewisePolynomial<double>> MakeKTrajectory(
      const PiecewisePolynomial<double>& S_trajectory) {
    // K = Rinv*B'*S.
    const std::vector<double>& breaks = S_trajectory.get_segment_times();

    std::vector<Eigen::MatrixXd> RinvBt(breaks.size());
    for (int i = 0; i < static_cast<int>(breaks.size()); i++) {
      const double time = breaks[i];
      context_->SetTime(time);
      context_->SetContinuousState(
          options_.x0 ? options_.x0->value(time).cast<AutoDiffXd>().eval()
                      : VectorX<AutoDiffXd>::Zero(num_states_));
      input_port_->FixValue(
          context_.get(),
          math::initializeAutoDiff(options_.u0
                                       ? options_.u0->value(time)
                                       : Eigen::VectorXd::Zero(num_inputs_)));
      const VectorX<AutoDiffXd> autodiff_xdot0 =
          system_->EvalTimeDerivatives(*context_).CopyToVector();
      const Eigen::MatrixXd B = math::autoDiffToGradientMatrix(autodiff_xdot0);
      RinvBt[i] = Rinv_ * B.transpose();
    }

    // Approximate inv(R)*B' with a first-order hold.
    // TODO(russt): Consider more accurate/refined interpolations here based
    // on the degree of the x0 and/or u0 trajectories, but remember that it
    // will also increase the degree of K(t).
    const PiecewisePolynomial<double> RinvBt_traj =
        PiecewisePolynomial<double>::FirstOrderHold(breaks, RinvBt);
    return std::make_unique<PiecewisePolynomial<double>>(RinvBt_traj *
                                                         S_trajectory);
  }

 private:
  const std::unique_ptr<const System<AutoDiffXd>> system_;
  const InputPort<AutoDiffXd>* input_port_{nullptr};

  // Note: Use a mutable context here to avoid reallocating on every call to
  // e.g. CalcTimeDerivatives. WARNING: this means that evaluation of the
  // RiccatiSystem is not thread safe, but since the implementation is internal
  // to this file, there are no entry points that would allow multi-threaded
  // execution with the same RiccatiSystem instance.
  const std::unique_ptr<Context<AutoDiffXd>> context_;

  const Eigen::Ref<const Eigen::MatrixXd>& Q_;
  const Eigen::MatrixXd Rinv_;
  const FiniteHorizonLinearQuadraticRegulatorOptions& options_;
  const int num_states_;
  const int num_inputs_;
};

}  // namespace internal

FiniteHorizonLinearQuadraticRegulatorResult
FiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options) {
  // Most argument consistency checks are handled by RiccatiSystem, but that
  // System doesn't need to understand the time range, so we perform (only)
  // those checks here.
  DRAKE_DEMAND(tf > t0);
  if (options.x0) {
    DRAKE_DEMAND(options.x0->start_time() <= t0);
    DRAKE_DEMAND(options.x0->end_time() >= tf);
  }
  if (options.u0) {
    DRAKE_DEMAND(options.u0->start_time() <= t0);
    DRAKE_DEMAND(options.u0->end_time() >= tf);
  }
  const int num_states = context.num_total_states();

  internal::RiccatiSystem riccati(system, context, Q, R, options);

  // Simulator doesn't support integrating backwards in time, so simulate the
  // time-reversed Riccati equation from -tf to -t0, and reverse it after the
  // fact.
  Simulator<double> simulator(riccati);

  // Set the initial conditions
  if (options.Qf) {
    const double kSymmetryTolerance = 1e-8;
    DRAKE_DEMAND(options.Qf->rows() == num_states &&
                 options.Qf->cols() == num_states);
    DRAKE_DEMAND(
        math::IsPositiveDefinite(*options.Qf, 0.0, kSymmetryTolerance));
    const Eigen::Map<const Eigen::VectorXd> Qf_vectorized(
        options.Qf->data(), num_states * num_states);
    simulator.get_mutable_context().SetContinuousState(Qf_vectorized);
  }

  simulator.get_mutable_context().SetTime(-tf);
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.StartDenseIntegration();

  simulator.AdvanceTo(-t0);

  FiniteHorizonLinearQuadraticRegulatorResult result;
  result.S = integrator.StopDenseIntegration();
  result.S->ReverseTime();
  result.S->Reshape(num_states, num_states);
  result.K = riccati.MakeKTrajectory(*result.S);

  return result;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
