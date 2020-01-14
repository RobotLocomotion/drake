#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <limits>
#include <memory>
#include <utility>
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
                const Trajectory<double>& x0,
                const Trajectory<double>& u0,
                const FiniteHorizonLinearQuadraticRegulatorOptions& options)
      : system_(System<double>::ToAutoDiffXd(system)),
        input_port_(
            system_->get_input_port_selection(options.input_port_index)),
        context_(system_->CreateDefaultContext()),
        Q_(Q),
        Rinv_(R.inverse()),
        x0_(x0),
        u0_(u0),
        num_states_(context_->num_total_states()),
        num_inputs_(input_port_->size()) {
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

    this->DeclareContinuousState(num_states_ * num_states_);

    // Initialize autodiff.
    context_->SetTimeStateAndParametersFrom(context);
    system_->FixInputPortsFrom(system, context, context_.get());
  }

  // Implement the (time-reversed) Riccati equation, as described in
  // http://underactuated.mit.edu/lqr.html#finite_horizon .
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    const Eigen::VectorXd S_vectorized =
        context.get_continuous_state_vector().CopyToVector();
    const Eigen::Map<const Eigen::MatrixXd> S(S_vectorized.data(), num_states_,
                                              num_states_);

    const double system_time = -context.get_time();
    context_->SetTime(system_time);

    // Get (time-varying) linearization of the plant.
    auto autodiff_args = math::initializeAutoDiffTuple(
        x0_.value(system_time), u0_.value(system_time));
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
  // Note that S_trajectory is assumed to have time span, [t0, tf], and size
  // num_states by num_states (e.g. it has already been flipped in time and
  // reshaped).
  PiecewisePolynomial<double> MakeKTrajectory(
      const PiecewisePolynomial<double>& S_trajectory) {
    // K = Rinv*B'*S.
    const std::vector<double>& breaks = S_trajectory.get_segment_times();

    std::vector<Eigen::MatrixXd> RinvBt(breaks.size());
    for (int i = 0; i < static_cast<int>(breaks.size()); i++) {
      const double time = breaks[i];
      context_->SetTime(time);
      context_->SetContinuousState(x0_.value(time).cast<AutoDiffXd>().eval());
      input_port_->FixValue(context_.get(),
                            math::initializeAutoDiff(u0_.value(time)));
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
    return PiecewisePolynomial<double>(RinvBt_traj * S_trajectory);
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

  const Eigen::Ref<const Eigen::MatrixXd>& Q_;
  const Eigen::MatrixXd Rinv_;
  const Trajectory<double>& x0_;
  const Trajectory<double>& u0_;
  const int num_states_;
  const int num_inputs_;
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
  DRAKE_DEMAND(system.num_input_ports() > 0);
  DRAKE_DEMAND(tf > t0);
  const int num_states = context.num_total_states();
  std::unique_ptr<Trajectory<double>> x0;
  if (options.x0) {
    DRAKE_DEMAND(options.x0->start_time() <= t0);
    DRAKE_DEMAND(options.x0->end_time() >= tf);
  } else {
    // Make a constant trajectory with the state from context.
    x0 = std::make_unique<PiecewisePolynomial<double>>(
        context.get_continuous_state_vector().CopyToVector());
  }

  std::unique_ptr<Trajectory<double>> u0;
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
  result.S = std::move(*(integrator.StopDenseIntegration()));
  result.S.ReverseTime();
  result.S.Reshape(num_states, num_states);
  result.K = riccati.MakeKTrajectory(result.S);

  return result;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
