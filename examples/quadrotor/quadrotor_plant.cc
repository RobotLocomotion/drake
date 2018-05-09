#include "drake/examples/quadrotor/quadrotor_plant.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/math/gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Matrix3d;

namespace drake {
namespace examples {
namespace quadrotor {

namespace {
Matrix3d default_moment_of_inertia() {
  return (Eigen::Matrix3d() <<  // BR
          0.0023, 0, 0,  // BR
          0, 0.0023, 0,  // BR
          0, 0, 0.0040).finished();
}
}  // namespace

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant()
    : QuadrotorPlant(0.5,    // m (kg)
                     0.175,  // L (m)
                     default_moment_of_inertia(),
                     1.0,    // kF
                     0.0245  // kM
                     ) {}

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(double m_arg, double L_arg,
                                  const Matrix3d& I_arg, double kF_arg,
                                  double kM_arg)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<quadrotor::QuadrotorPlant>{}),
      g_{9.81}, m_(m_arg), L_(L_arg), kF_(kF_arg), kM_(kM_arg), I_(I_arg) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
  this->DeclareContinuousState(kStateDimension);
  this->DeclareVectorOutputPort(systems::BasicVector<T>(kStateDimension),
                                &QuadrotorPlant::CopyStateOut);
}

template <typename T>
template <typename U>
QuadrotorPlant<T>:: QuadrotorPlant(const QuadrotorPlant<U>& other)
    : QuadrotorPlant<T>(other.m_, other.L_, other.I_, other.kF_, other.kM_) {}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

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

  const VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  // Extract orientation and angular velocities.
  const drake::math::RollPitchYaw<T> rpy(state.segment(3, 3));
  const Vector3<T> rpyDt = state.segment(9, 3);

  // Convert orientation to a rotation matrix.
  const drake::math::RotationMatrix<T> R(rpy);

  // Compute the net input forces and moments.
  const VectorX<T> uF = kF_ * u;
  const VectorX<T> uM = kM_ * u;

  const Vector3<T> Fg(0, 0, -m_ * g_);
  const Vector3<T> F(0, 0, uF.sum());
  const Vector3<T> M(L_ * (uF(1) - uF(3)), L_ * (uF(2) - uF(0)),
                     uM(0) - uM(1) + uM(2) - uM(3));

  // Computing the resultant linear acceleration due to the forces.
  const Vector3<T> xyzDDt = (Fg + R * F) / m_;

  Vector3<T> pqr;
  rpydot2angularvel(rpy.vector(), rpyDt, pqr);
  pqr = R.matrix().adjoint() * pqr;

  // Computing the resultant angular acceleration due to the moments.
  const Vector3<T> pqrDt = I_.ldlt().solve(M - pqr.cross(I_ * pqr));
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy.vector(), Phi, &dPhi, ddPhi);

  const Matrix3<T> RDt =
      drake::math::RotationMatrix<T>::OrdinaryDerivativeRotationMatrix(rpy,
                                                                       rpyDt);
  VectorX<T> dPhi_x_rpyDt_vec = dPhi * rpyDt;
  const Matrix3<T> dPhi_x_rpyDt =
      Eigen::Map<Matrix3<T>>(dPhi_x_rpyDt_vec.data());
  const Vector3<T> rpyDDt = Phi * R.matrix() * pqrDt
                          + dPhi_x_rpyDt * R.matrix() * pqr + Phi * RDt * pqr;

  // Recomposing the derivatives vector.
  VectorX<T> xDt(12);
  xDt << state.tail(6), xyzDDt, rpyDDt;

  derivatives->SetFromVector(xDt);
}

// Declare storage for our constants.
template <typename T>
constexpr int QuadrotorPlant<T>::kStateDimension;
template <typename T>
constexpr int QuadrotorPlant<T>::kInputDimension;

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

  return systems::controllers::LinearQuadraticRegulator(
      *quadrotor_plant, *quad_context_goal, Q, R);
}

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::quadrotor::QuadrotorPlant)
