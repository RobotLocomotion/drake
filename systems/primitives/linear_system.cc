#include "drake/systems/primitives/linear_system.h"

#include <string>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <fmt/format.h>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
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
    : LinearSystem<T>(SystemTypeTag<LinearSystem>{}, A, B, C, D,
                      time_period) {}

template <typename T>
template <typename U>
LinearSystem<T>::LinearSystem(const LinearSystem<U>& other)
    : LinearSystem<T>(other.A(), other.B(), other.C(), other.D(),
                      other.time_period()) {}

template <typename T>
LinearSystem<T>::LinearSystem(SystemScalarConverter converter,
                              const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              double time_period)
    : AffineSystem<T>(std::move(converter), A, B,
                      Eigen::VectorXd::Zero(A.rows()), C, D,
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
  DRAKE_DEMAND(num_states == dynamics.size());
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

namespace {

// Helper function allows reuse for both FirstOrderTaylorApproximation and
// Linearize.
std::unique_ptr<AffineSystem<double>> DoFirstOrderTaylorApproximation(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    std::variant<OutputPortSelection, OutputPortIndex> output_port_index,
    std::optional<double> equilibrium_check_tolerance = std::nullopt) {
  system.ValidateContext(context);

  double time_period = 0.0;
  const bool is_discrete_system =
      system.IsDifferenceEquationSystem(&time_period);
  DRAKE_THROW_UNLESS(context.is_stateless() ||
                     context.has_only_continuous_state() || is_discrete_system);

  // Create an autodiff version of the system.
  std::unique_ptr<System<AutoDiffXd>> autodiff_system =
      drake::systems::System<double>::ToAutoDiffXd(system);

  // Initialize autodiff.
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);
  autodiff_system->FixInputPortsFrom(system, context, autodiff_context.get());

  const InputPort<AutoDiffXd>* input_port =
      autodiff_system->get_input_port_selection(input_port_index);
  const OutputPort<AutoDiffXd>* output_port =
      autodiff_system->get_output_port_selection(output_port_index);

  // Verify that the input port is not abstract valued.
  if (input_port &&
      input_port->get_data_type() == PortDataType::kAbstractValued) {
    throw std::logic_error(
        "Port requested for differentiation is abstract, and differentiation "
        "of abstract ports is not supported.");
  }

  const int num_inputs = input_port ? input_port->size() : 0;
  const int num_outputs = output_port ? output_port->size() : 0;

  const Eigen::VectorXd x0 =
      context.is_stateless()
          ? Eigen::VectorXd::Zero(0)
          : ((context.has_only_continuous_state())
                 ? context.get_continuous_state_vector().CopyToVector()
                 : context.get_discrete_state(0).get_value());
  const int num_states = x0.size();

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  if (input_port) {
    u0 = system.get_input_port(input_port->get_index()).Eval(context);
  }

  auto autodiff_args = math::InitializeAutoDiffTuple(x0, u0);
  if (input_port) {
    VectorX<AutoDiffXd> input_vector = std::get<1>(autodiff_args);
    input_port->FixValue(autodiff_context.get(), input_vector);
  }

  Eigen::MatrixXd A(num_states, num_states), B(num_states, num_inputs);
  Eigen::VectorXd f0(num_states);
  if (num_states > 0) {
    if (autodiff_context->has_only_continuous_state()) {
      autodiff_context->get_mutable_continuous_state_vector().SetFromVector(
          std::get<0>(autodiff_args));
      std::unique_ptr<ContinuousState<AutoDiffXd>> autodiff_xdot =
          autodiff_system->AllocateTimeDerivatives();
      autodiff_system->CalcTimeDerivatives(*autodiff_context,
                                           autodiff_xdot.get());
      auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

      const Eigen::MatrixXd AB = math::ExtractGradient(autodiff_xdot_vec);
      A = AB.leftCols(num_states);
      B = AB.rightCols(num_inputs);

      const Eigen::VectorXd xdot0 = math::ExtractValue(autodiff_xdot_vec);

      if (equilibrium_check_tolerance &&
          !xdot0.isZero(*equilibrium_check_tolerance)) {
        throw std::runtime_error(
            "The nominal operating point (x0,u0) is not an equilibrium point "
            "of the system.  Without additional information, a time-invariant "
            "linearization of this system is not well defined.");
      }

      f0 = xdot0 - A * x0 - B * u0;
    } else {
      DRAKE_ASSERT(is_discrete_system);
      auto& autodiff_x0 =
          autodiff_context->get_mutable_discrete_state().get_mutable_vector();
      autodiff_x0.SetFromVector(std::get<0>(autodiff_args));
      std::unique_ptr<DiscreteValues<AutoDiffXd>> autodiff_x1 =
          autodiff_system->AllocateDiscreteVariables();
      autodiff_system->CalcDiscreteVariableUpdates(*autodiff_context,
                                                   autodiff_x1.get());
      auto autodiff_x1_vec = autodiff_x1->get_value();

      const Eigen::MatrixXd AB = math::ExtractGradient(autodiff_x1_vec);
      A = AB.leftCols(num_states);
      B = AB.rightCols(num_inputs);

      const Eigen::VectorXd x1 = math::ExtractValue(autodiff_x1_vec);

      if (equilibrium_check_tolerance &&
          !(x1 - x0).isZero(*equilibrium_check_tolerance)) {
        throw std::runtime_error(
            "The nominal operating point (x0,u0) is not an equilibrium point "
            "of the system.  Without additional information, a time-invariant "
            "linearization of this system is not well defined.");
      }

      f0 = x1 - A * x0 - B * u0;
    }
  } else {
    DRAKE_ASSERT(num_states == 0);
    A = Eigen::MatrixXd(0, 0);
    B = Eigen::MatrixXd(0, num_inputs);
    f0 = Eigen::VectorXd(0);
  }

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);
  Eigen::VectorXd y0 = Eigen::VectorXd::Zero(num_outputs);

  if (output_port) {
    const auto& autodiff_y0 = output_port->Eval(*autodiff_context);
    const Eigen::MatrixXd CD = math::ExtractGradient(autodiff_y0);
    C = CD.leftCols(num_states);
    D = CD.rightCols(num_inputs);

    const Eigen::VectorXd y = math::ExtractValue(autodiff_y0);

    // Note: No tolerance check needed here.  We have defined that the output
    // for the system produced by Linearize is in the coordinates (y-y0).

    y0 = y - C * x0 - D * u0;
  }

  return std::make_unique<AffineSystem<double>>(A, B, f0, C, D, y0,
                                                time_period);
}

}  // namespace

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    std::variant<OutputPortSelection, OutputPortIndex> output_port_index,
    double equilibrium_check_tolerance) {
  std::unique_ptr<AffineSystem<double>> affine =
      DoFirstOrderTaylorApproximation(
          system, context, std::move(input_port_index),
          std::move(output_port_index), equilibrium_check_tolerance);

  return std::make_unique<LinearSystem<double>>(affine->A(), affine->B(),
                                                affine->C(), affine->D(),
                                                affine->time_period());
}

std::unique_ptr<AffineSystem<double>> FirstOrderTaylorApproximation(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    std::variant<OutputPortSelection, OutputPortIndex> output_port_index) {
  return DoFirstOrderTaylorApproximation(system, context,
                                         std::move(input_port_index),
                                         std::move(output_port_index));
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
bool IsControllable(const LinearSystem<double>& sys,
                    std::optional<double> threshold) {
  const auto R = ControllabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(R);
  if (threshold) {
    lu_decomp.setThreshold(threshold.value());
  }
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
bool IsObservable(const LinearSystem<double>& sys,
                  std::optional<double> threshold) {
  const auto O = ObservabilityMatrix(sys);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> lu_decomp(O);
  if (threshold) {
    lu_decomp.setThreshold(threshold.value());
  }
  return lu_decomp.rank() == sys.A().rows();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LinearSystem)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TimeVaryingLinearSystem)
