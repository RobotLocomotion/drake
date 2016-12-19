#include "drake/examples/Quadrotor/quadrotor_plant.h"

namespace drake {
namespace examples {
namespace quadrotor {

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant() {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
}

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(const double m_arg, const double L_arg,
                                  const Matrix3<T> I_arg, const double kF_arg,
                                  const double kM_arg)
    : m(m_arg), L(L_arg), kF(kF_arg), kM(kM_arg), I(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

template <typename T>
QuadrotorPlant<AutoDiffXd>* QuadrotorPlant<T>::DoToAutoDiffXd() const {
  return new QuadrotorPlant<AutoDiffXd>(m, L, I, kF, kM);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
QuadrotorPlant<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  DRAKE_THROW_UNLESS(descriptor.get_size() == kStateDimension);
  return std::make_unique<systems::BasicVector<T>>(kStateDimension);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
QuadrotorPlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<systems::BasicVector<T>>(12), 6 /* num_q */,
      6 /* num_v */, 0 /* num_z */);
}

template <typename T>
void QuadrotorPlant<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)->set_value(
      context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void QuadrotorPlant<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  VectorX<T> uF = kF * u;
  VectorX<T> uM = kM * u;

  Vector3<T> Fg(0, 0, -m * g);
  Vector3<T> F(0, 0, uF.sum());
  Vector3<T> M(L * (uF(1) - uF(3)), L * (uF(2) - uF(0)),
               uM(0) - uM(1) + uM(2) - uM(3));
  Vector3<T> xyz_ddot = (1.0 / m) * (Fg + R * F);

  Vector3<T> pqr;
  rpydot2angularvel(rpy, rpy_dot, pqr);
  pqr = R.adjoint() * pqr;

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

  VectorX<T> xdot(12);
  xdot << state.tail(6), xyz_ddot, rpy_ddot;
  derivatives->SetFromVector(xdot);
}

template class QuadrotorPlant<double>;
template class QuadrotorPlant<AutoDiffXd>;

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
