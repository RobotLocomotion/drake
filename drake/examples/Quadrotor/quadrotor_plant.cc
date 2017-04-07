#include "drake/examples/Quadrotor/quadrotor_plant.h"

#include <memory>

#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace quadrotor {

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant() {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),
                                &QuadrotorPlant::CopyStateOut);
}

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(double m_arg, double L_arg,
                                  const Matrix3<T>& I_arg, double kF_arg,
                                  double kM_arg)
    : m_(m_arg), L_(L_arg), kF_(kF_arg), kM_(kM_arg), I_(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),
                                &QuadrotorPlant::CopyStateOut);
}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

template <typename T>
QuadrotorPlant<AutoDiffXd>* QuadrotorPlant<T>::DoToAutoDiffXd() const {
  return new QuadrotorPlant<AutoDiffXd>(m_, L_, I_, kF_, kM_);
}

template <typename T>
void QuadrotorPlant<T>::CopyStateOut(const systems::Context<T> &context,
                                     systems::BasicVector<T> *output) const {
  output->set_value(
      context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void QuadrotorPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  // Extract orientation and angular velocities.
  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);

  // Convert orientation to a rotation matrix.
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  // Compute the net input forces and moments.
  VectorX<T> uF = kF_ * u;
  VectorX<T> uM = kM_ * u;

  Vector3<T> Fg(0, 0, -m_ * g_);
  Vector3<T> F(0, 0, uF.sum());
  Vector3<T> M(L_ * (uF(1) - uF(3)), L_ * (uF(2) - uF(0)),
               uM(0) - uM(1) + uM(2) - uM(3));

  // Computing the resultant linear acceleration due to the forces.
  Vector3<T> xyz_ddot = (1.0 / m_) * (Fg + R * F);

  Vector3<T> pqr;
  rpydot2angularvel(rpy, rpy_dot, pqr);
  pqr = R.adjoint() * pqr;

  // Computing the resultant angular acceleration due to the moments.
  Vector3<T> pqr_dot = I_.ldlt().solve(M - pqr.cross(I_ * pqr));
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

  MatrixX<T> drpy2drotmat = drake::math::drpy2rotmat(rpy);
  VectorX<T> Rdot_vec(9);
  Rdot_vec = drpy2drotmat * rpy_dot;
  Matrix3<T> Rdot = Eigen::Map<Matrix3<T>>(Rdot_vec.data());
  VectorX<T> dPhi_x_rpydot_vec;
  dPhi_x_rpydot_vec = dPhi * rpy_dot;
  Matrix3<T> dPhi_x_rpydot = Eigen::Map<Matrix3<T>>(dPhi_x_rpydot_vec.data());
  Vector3<T> rpy_ddot =
      Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

  // Recomposing the derivatives vector.
  VectorX<T> xdot(12);
  xdot << state.tail(6), xyz_ddot, rpy_ddot;

  derivatives->SetFromVector(xdot);
}

template class QuadrotorPlant<double>;
template class QuadrotorPlant<AutoDiffXd>;

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadrotorPlant<double>* quadrotor_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_context_goal = quadrotor_plant->CreateDefaultContext();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = nominal_position;

  // Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
      4, quadrotor_plant->m() * quadrotor_plant->g() / 4);

  quad_context_goal->FixInputPort(0, u0);
  quadrotor_plant->set_state(quad_context_goal.get(), x0);

  // Setup LQR cost matrices (penalize position error 10x more than velocity
  // error).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();

  return systems::LinearQuadraticRegulator(*quadrotor_plant,
                                           *quad_context_goal, Q, R);
}

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
