#include "drake/systems/primitives/linear_system.h"

#include <string>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/event_collection.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::unique_ptr;

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              double time_period)
    : AffineSystem<T>(A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
                      Eigen::VectorXd::Zero(C.rows()), time_period) {}

template <typename T>
unique_ptr<LinearSystem<T>> LinearSystem<T>::MakeLinearSystem(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
    const double time_period) {
  // Need to extract, A, B, C, D such that,
  //
  //     dynamics = Ax + Bu
  //     output   = Cx + Du
  //
  // where x = state_vars and u = input_vars.
  const int num_states = state_vars.size();
  DRAKE_ASSERT(num_states == dynamics.size());
  const int num_inputs = input_vars.size();
  const int num_outputs = output.size();

  Eigen::MatrixXd AB(num_states, num_states + num_inputs);
  VectorX<symbolic::Variable> vars(num_states + num_inputs);
  vars << state_vars, input_vars;
  DecomposeLinearExpressions(dynamics, vars, &AB);
  const auto A = AB.leftCols(num_states);
  const auto B = AB.rightCols(num_inputs);

  Eigen::MatrixXd CD(num_outputs, num_states + num_inputs);
  DecomposeLinearExpressions(output, vars, &CD);
  const auto C = CD.leftCols(num_states);
  const auto D = CD.rightCols(num_inputs);

  return make_unique<LinearSystem<T>>(A, B, C, D, time_period);
}

template class LinearSystem<double>;
template class LinearSystem<AutoDiffXd>;

namespace {

// If the system has discrete states, checks that the sole registered event for
// the system is periodic, and returns the time_period. A time_period of zero is
// returned if the system has continuous states.
double GetTimePeriodIfDiscreteUpdatesArePeriodic(
    const System<double>& system, const Context<double>& context) {
  if (!context.has_only_discrete_state()) return 0.;

  std::unique_ptr<CompositeEventCollection<double>> event_info =
      system.AllocateCompositeEventCollection();
  const double time_period =
      system.CalcNextUpdateTime(context, event_info.get()) - context.get_time();

  // Verify that the system has only one discrete, periodic update event.
  // TODO(jadecastro) Upon resolution of #6878, clean up this implementation and
  // weed out all illegal systems having a combination of event handlers.
  DRAKE_THROW_UNLESS(event_info->HasDiscreteUpdateEvents());
  const auto leaf_info =
      dynamic_cast<const LeafCompositeEventCollection<double>*>(
          event_info.get());
  DRAKE_DEMAND(leaf_info != nullptr);
  auto discrete_events = leaf_info->get_discrete_update_events().get_events();
  DRAKE_THROW_UNLESS(discrete_events.size() == 1);
  DRAKE_THROW_UNLESS(discrete_events.front()->get_trigger_type() ==
                     Event<double>::TriggerType::kPeriodic);
  return time_period;
}

// Builds the A and B matrices of the system's discrete/continuous state
// equation.
Eigen::MatrixXd MakeStateAndInputMatrices(
    const VectorX<AutoDiffXd>& autodiff_x0_vec,
    double equilibrium_check_tolerance,
    const System<AutoDiffXd>& autodiff_system,
    Context<AutoDiffXd>* autodiff_context) {
  const std::string nonequilibrium_error_msg =
      "The nominal operating point (x0,u0) is not an equilibrium point of "
      "the system.  Without additional information, a time-invariant "
      "linearization of this system is not well defined.";
  if (autodiff_context->has_only_continuous_state()) {
    autodiff_context->get_mutable_continuous_state_vector()->SetFromVector(
        autodiff_x0_vec);
    std::unique_ptr<ContinuousState<AutoDiffXd>> autodiff_xdot =
        autodiff_system.AllocateTimeDerivatives();
    autodiff_system.CalcTimeDerivatives(*autodiff_context,
                                        autodiff_xdot.get());
    auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

    // Ensure that xdot0 = f(x0,u0) == 0.
    if (!math::autoDiffToValueMatrix(autodiff_xdot_vec)
        .isZero(equilibrium_check_tolerance)) {
      throw std::runtime_error(nonequilibrium_error_msg);
    }
    return math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  }
  auto autodiff_x0 =
      autodiff_context->get_mutable_discrete_state()->get_mutable_vector();
  autodiff_x0->SetFromVector(autodiff_x0_vec);
  std::unique_ptr<DiscreteValues<AutoDiffXd>> autodiff_x1 =
      autodiff_system.AllocateDiscreteVariables();
  autodiff_system.CalcDiscreteVariableUpdates(*autodiff_context,
                                              autodiff_x1.get());
  auto autodiff_x1_vec = autodiff_x1->get_vector()->CopyToVector();

  // Ensure that x1 = f(x0,u0) == x0.
  if (!(math::autoDiffToValueMatrix(autodiff_x1_vec) -
        math::autoDiffToValueMatrix(autodiff_x0_vec)).isZero(
            equilibrium_check_tolerance)) {
    throw std::runtime_error(nonequilibrium_error_msg);
  }
  return math::autoDiffToGradientMatrix(autodiff_x1_vec);
}

}  // namespace

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    double equilibrium_check_tolerance) {
  DRAKE_ASSERT_VOID(system.CheckValidContext(context));

  const bool has_only_discrete_states_contained_in_one_group =
      context.has_only_discrete_state() &&
      context.get_num_discrete_state_groups() == 1;
  DRAKE_DEMAND(context.is_stateless() || context.has_only_continuous_state() ||
               has_only_discrete_states_contained_in_one_group);

  const double time_period =
      GetTimePeriodIfDiscreteUpdatesArePeriodic(system, context);

  DRAKE_DEMAND(system.get_num_input_ports() <= 1);
  DRAKE_DEMAND(system.get_num_output_ports() <= 1);

  const int num_inputs = (system.get_num_input_ports() > 0)
                             ? system.get_input_port(0).size()
                             : 0,
            num_outputs = (system.get_num_output_ports() > 0)
                              ? system.get_output_port(0).size()
                              : 0;

  // Create an autodiff version of the system.
  std::unique_ptr<System<AutoDiffXd>> autodiff_system =
      drake::systems::System<double>::ToAutoDiffXd(system);

  // Initialize autodiff.
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  if (num_inputs > 0) {
    u0 = system.EvalEigenVectorInput(context, 0);
  }

  const Eigen::VectorXd x0 = (context.has_only_continuous_state())
      ? context.get_continuous_state_vector().CopyToVector()
      : context.get_discrete_state(0)->get_value();
  const int num_states = x0.size();

  auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);
  if (num_inputs > 0) {
    auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(num_inputs);
    input_vector->SetFromVector(std::get<1>(autodiff_args));
    autodiff_context->SetInputPortValue(
        0,
        std::make_unique<FreestandingInputPortValue>(std::move(input_vector)));
  }

  const Eigen::MatrixXd AB =
      MakeStateAndInputMatrices(std::get<0>(autodiff_args),
                                equilibrium_check_tolerance,
                                *autodiff_system, autodiff_context.get());
  const Eigen::MatrixXd A = AB.leftCols(num_states);
  const Eigen::MatrixXd B = AB.rightCols(num_inputs);

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);

  if (num_outputs > 0) {
    std::unique_ptr<SystemOutput<AutoDiffXd>> autodiff_y0 =
        autodiff_system->AllocateOutput(*autodiff_context);
    autodiff_system->CalcOutput(*autodiff_context, autodiff_y0.get());
    auto autodiff_y0_vec = autodiff_y0->get_vector_data(0)->CopyToVector();

    Eigen::MatrixXd CD = math::autoDiffToGradientMatrix(autodiff_y0_vec);
    C = CD.leftCols(num_states);
    D = CD.rightCols(num_inputs);
  }

  return std::make_unique<LinearSystem<double>>(A, B, C, D, time_period);
}

/// Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
Eigen::MatrixXd ControllabilityMatrix(const LinearSystem<double>& sys) {
  DRAKE_DEMAND(sys.time_period() == 0.0);
  // TODO(russt): handle the discrete time case

  const int num_states = sys.B().rows(), num_inputs = sys.B().cols();
  Eigen::MatrixXd R(num_states, num_states * num_inputs);
  R.leftCols(num_inputs) = sys.B();
  for (int i = 1; i < num_states; i++) {
    R.middleCols(num_inputs * i, num_inputs) =
        sys.A() * R.middleCols(num_inputs * (i - 1), num_inputs);
  }
  return R;
}

/// Returns true iff the controllability matrix is full row rank.
bool IsControllable(const LinearSystem<double>& sys, double threshold) {
  const auto R = ControllabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(R);
  lu_decomp.setThreshold(threshold);
  return lu_decomp.rank() == sys.A().rows();
}

/// Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
Eigen::MatrixXd ObservabilityMatrix(const LinearSystem<double>& sys) {
  DRAKE_DEMAND(sys.time_period() == 0.0);
  // TODO(russt): handle the discrete time case

  const int num_states = sys.C().cols(), num_outputs = sys.C().rows();
  Eigen::MatrixXd O(num_states * num_outputs, num_states);
  O.topRows(num_outputs) = sys.C();
  for (int i = 1; i < num_states; i++) {
    O.middleRows(num_outputs * i, num_outputs) =
        O.middleRows(num_outputs * (i - 1), num_outputs) * sys.A();
  }
  return O;
}

/// Returns true iff the observability matrix is full column rank.
bool IsObservable(const LinearSystem<double>& sys, double threshold) {
  const auto O = ObservabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(O);
  lu_decomp.setThreshold(threshold);
  return lu_decomp.rank() == sys.A().rows();
}

}  // namespace systems
}  // namespace drake
