#include "drake/examples/Quadrotor/quadrotor_plant.h"

#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace quadrotor {

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant() {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
}

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(double m_arg, double L_arg,
                                  const Matrix3<T>& I_arg, double kF_arg,
                                  double kM_arg)
    : m(m_arg), L(L_arg), kF(kF_arg), kM(kM_arg), I(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

template <typename T>
QuadrotorPlant<AutoDiffXd>* QuadrotorPlant<T>::DoToAutoDiffXd() const {
  return new QuadrotorPlant<AutoDiffXd>(m, L, I, kF, kM);
}

template <typename T>
void QuadrotorPlant<T>::DoCalcOutput(const systems::Context<T> &context,
                                     systems::SystemOutput<T> *output) const {
  output->GetMutableVectorData(0)->set_value(
      context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void QuadrotorPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  // Extract orientation and angular velocities.
  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);

  // Convert orientation to a rotation matrix
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  // Computing the net input forces and moments.
  VectorX<T> uF = kF * u;
  VectorX<T> uM = kM * u;

  Vector3<T> Fg(0, 0, -m * g);
  Vector3<T> F(0, 0, uF.sum());
  Vector3<T> M(L * (uF(1) - uF(3)), L * (uF(2) - uF(0)),
               uM(0) - uM(1) + uM(2) - uM(3));

  // Computing the resultant linear acceleration due to the forces.
  Vector3<T> xyz_ddot = (1.0 / m) * (Fg + R * F);

  Vector3<T> pqr;
  rpydot2angularvel(rpy, rpy_dot, pqr);
  pqr = R.adjoint() * pqr;

  // Computing the resultant angular acceleration due to the moments.
  Vector3<T> pqr_dot = I.ldlt().solve(M - pqr.cross(I * pqr));
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

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
