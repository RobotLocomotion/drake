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
  // Get the input value characterizing each of the 4 rotor's aerodynamics.
  const Vector4<T> u = this->EvalVectorInput(context, 0)->get_value();

  // For each rotor, calculate the Bz measure of its aerodynamic force on B.
  // Note: B is the quadrotor body and Bz is parallel to each rotor's spin axis.
  const Vector4<T> uF_Bz = kF_ * u;

  // Compute the net aerodynamic force on B (from the 4 rotors), expressed in B.
  const Vector3<T> Faero_B(0, 0, uF_Bz.sum());

  // Compute the Bx and By measures of the moment on B about Bcm (B's center of
  // mass) from the 4 rotor forces.  These moments arise from the cross product
  // of a position vector with an aerodynamic force at the center of each rotor.
  // For example, the moment of the aerodynamic forces on rotor 0 about Bcm
  // results from Cross( L_* Bx, uF_Bz(0) * Bz ) = -L_ * uF_Bz(0) * By.
  const T Mx = L_ * (uF_Bz(1) - uF_Bz(3));
  const T My = L_ * (uF_Bz(2) - uF_Bz(0));

  // For rotors 0 and 2, get the Bz measure of its aerodynamic torque on B.
  // For rotors 1 and 3, get the -Bz measure of its aerodynamic torque on B.
  // Sum the net Bz measure of the aerodynamic torque on B.
  // Note: Rotors 0 and 2 rotate one way and rotors 1 and 3 rotate the other.
  const Vector4<T> uM_Bz = kM_ * u;
  const T Mz = uM_Bz(0) - uM_Bz(1) + uM_Bz(2) - uM_Bz(3);

  // Form the net moment on B about Bcm, expressed in B. The net moment accounts
  // for all contact and distance forces (aerodynamic and gravity forces) on B.
  // Note: Since the net moment on B is about Bcm, gravity does not contribute.
  const Vector3<T> M(Mx, My, Mz);

  // Calculate local celestial body's (Earth's) gravity force on B, expressed in
  // the Newtonian frame N (a.k.a the inertial or World frame).
  const Vector3<T> Fgravity_N(0, 0, -m_ * g_);

  // Extract roll-pitch-yaw angles (rpy) and their time-derivatives (rpyDt).
  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();
  const drake::math::RollPitchYaw<T> rpy(state.template segment<3>(3));
  const Vector3<T> rpyDt = state.template segment<3>(9);

  // Convert roll-pitch-yaw (rpy) orientation to the R_NB rotation matrix.
  const drake::math::RotationMatrix<T> R_NB(rpy);

  // Calculate the net force on B, expressed in N.
  // Calculate B's center of mass acceleration, expressed in N.
  const Vector3<T> Fnet_N = Fgravity_N + R_NB * Faero_B;
  const Vector3<T> xyzDDt = Fnet_N / m_;

  // Use rpy and rpyDt to calculate B's angular velocity in N, expressed in N.
  // Then calculate B's angular velocity in N, expressed in B.
  // TODO(mitiguy) replace rpydot2angularvel with new RollPitchYaw method.
  Vector3<T> w_BN_N;
  rpydot2angularvel(rpy.vector(), rpyDt, w_BN_N);
  const Vector3<T> w_BN_B = R_NB.inverse() * w_BN_N;

  // To compute B's angular acceleration in N, expressed in B, due to the net
  // moment on B, rearrange Euler rigid body equation to solve for alpha_BN_B.
  // Euler's equation: M = I_.Dot(alpha_BN_B) + w.Cross(I_.Dot(w)).
  // Next, calculate B's angular acceleration in N, expressed in N.
  const Vector3<T> wIw = w_BN_B.cross(I_ * w_BN_B);      // Expressed in B.
  const Vector3<T> alf_BN_B = I_.ldlt().solve(M - wIw);  // Expressed in B.
  const Vector3<T> alf_BN_N = R_NB * alf_BN_B;           // Expressed in N.

  // TODO(mitiguy) replace angularvel2rpydotMatrix with new RollPitchYaw method.
  // TODO(mitiguy) continue fixing documentation for this example (here to end).
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy.vector(), Phi, &dPhi, ddPhi);

  const Eigen::Matrix<T, 9, 1> dPhi_x_rpyDt_vec = dPhi * rpyDt;
  const Eigen::Map<const Matrix3<T>> dPhi_x_rpyDt(dPhi_x_rpyDt_vec.data());
  const Matrix3<T> R_NB_Dt = rpy.OrdinaryDerivativeRotationMatrix(rpyDt);
  const Vector3<T> rpyDDt = Phi * alf_BN_N
                          + dPhi_x_rpyDt * w_BN_N
                          + Phi * R_NB_Dt * w_BN_B;

  // Recomposing the derivatives vector.
  VectorX<T> xDt(12);
  xDt << state.template tail<6>(), xyzDDt, rpyDDt;
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
