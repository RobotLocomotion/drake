#include "drake/systems/primitives/linear_system.h"

#include <utility>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              double time_period)
    : AffineSystem<T>(A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
                      Eigen::VectorXd::Zero(C.rows()), time_period) {}
// TODO(naveenoid): Modify constructor to accommodate 0 dimension systems;
// i.e. in initializing xDot0 and y0 with a zero matrix.
template class LinearSystem<double>;
template class LinearSystem<AutoDiffXd>;

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    const double equilibrium_check_tolerance) {
  DRAKE_ASSERT_VOID(system.CheckValidContext(context));

  DRAKE_DEMAND(context.is_stateless() || context.has_only_continuous_state());
  // TODO(russt): handle the discrete time case

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

  const Eigen::VectorXd x0 =
      context.get_continuous_state_vector().CopyToVector();
  const int num_states = x0.size();

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  if (num_inputs > 0) {
    u0 = system.EvalEigenVectorInput(context, 0);
  }

  auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);
  autodiff_context->get_mutable_continuous_state_vector()->SetFromVector(
      std::get<0>(autodiff_args));

  if (num_inputs > 0) {
    auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(num_inputs);
    input_vector->SetFromVector(std::get<1>(autodiff_args));
    autodiff_context->SetInputPortValue(
        0,
        std::make_unique<FreestandingInputPortValue>(std::move(input_vector)));
  }

  std::unique_ptr<ContinuousState<AutoDiffXd>> autodiff_xdot =
      autodiff_system->AllocateTimeDerivatives();
  autodiff_system->CalcTimeDerivatives(*autodiff_context, autodiff_xdot.get());
  auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

  // Ensure that xdot0 = f(x0,u0) == 0.
  if (!math::autoDiffToValueMatrix(autodiff_xdot_vec)
           .isZero(equilibrium_check_tolerance)) {
    throw std::runtime_error(
        "The nominal operating point (x0,u0) is not an equilibrium point of "
        "the system.  Without additional information, a time-invariant "
        "linearization of this system is not well defined.");
  }

  Eigen::MatrixXd AB = math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  Eigen::MatrixXd A = AB.leftCols(num_states);
  Eigen::MatrixXd B = AB.rightCols(num_inputs);

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

  return std::make_unique<LinearSystem<double>>(A, B, C, D);
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
