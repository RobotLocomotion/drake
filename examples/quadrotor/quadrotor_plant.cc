#include "drake/examples/quadrotor/quadrotor_plant.h"

#include "drake/common/default_scalars.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

using Eigen::Matrix3d;

namespace drake {
namespace examples {
namespace quadrotor {

namespace {
Matrix3d default_moment_of_inertia() {
  return (Eigen::Matrix3d() <<  // BR
          0.0015, 0, 0,  // BR
          0, 0.0025, 0,  // BR
          0, 0, 0.0035).finished();
}
}  // namespace

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant()
    : QuadrotorPlant(0.775,  // m (kg)
                     0.15,  // L (m)
                     default_moment_of_inertia(),
                     1.0,    // kF
                     0.0245  // kM
                     ) {}

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(double m_arg, double L_arg,
                                  const Matrix3d& I_arg, double kF_arg,
                                  double kM_arg)
    : systems::LeafSystem<T>(systems::SystemTypeTag<QuadrotorPlant>{}),
      g_{9.81}, m_(m_arg), L_(L_arg), kF_(kF_arg), kM_(kM_arg), I_(I_arg) {
  // Four inputs -- one for each propellor.
  this->DeclareInputPort("propellor_force", systems::kVectorValued, 4);
  // State is x ,y , z, roll, pitch, yaw + velocities.
  auto state_index = this->DeclareContinuousState(12);
  this->DeclareStateOutputPort("state", state_index);
  // TODO(russt): Declare input limits.  R2 has @ 2:1 thrust to weight ratio.
}

template <typename T>
template <typename U>
QuadrotorPlant<T>:: QuadrotorPlant(const QuadrotorPlant<U>& other)
    : QuadrotorPlant<T>(other.m_, other.L_, other.I_, other.kF_, other.kM_) {}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

// TODO(russt): Generalize this to support the rotor locations on the Skydio R2.
template <typename T>
void QuadrotorPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  // Get the input value characterizing each of the 4 rotor's aerodynamics.
  const Vector4<T> u = this->EvalVectorInput(context, 0)->value();

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
  const Vector4<T> uTau_Bz = kM_ * u;
  const T Mz = uTau_Bz(0) - uTau_Bz(1) + uTau_Bz(2) - uTau_Bz(3);

  // Form the net moment on B about Bcm, expressed in B. The net moment accounts
  // for all contact and distance forces (aerodynamic and gravity forces) on B.
  // Note: Since the net moment on B is about Bcm, gravity does not contribute.
  const Vector3<T> Tau_B(Mx, My, Mz);

  // Calculate local celestial body's (Earth's) gravity force on B, expressed in
  // the Newtonian frame N (a.k.a the inertial or World frame).
  const Vector3<T> Fgravity_N(0, 0, -m_ * g_);

  // Extract roll-pitch-yaw angles (rpy) and their time-derivatives (rpyDt).
  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();
  const drake::math::RollPitchYaw<T> rpy(state.template segment<3>(3));
  const Vector3<T> rpyDt = state.template segment<3>(9);

  // Convert roll-pitch-yaw (rpy) orientation to the R_NB rotation matrix.
  const drake::math::RotationMatrix<T> R_NB(rpy);

  // Calculate the net force on B, expressed in N.  Use Newton's law to
  // calculate a_NBcm_N (acceleration of B's center of mass, expressed in N).
  const Vector3<T> Fnet_N = Fgravity_N + R_NB * Faero_B;
  const Vector3<T> xyzDDt = Fnet_N / m_;  // Equal to a_NBcm_N.

  // Use rpy and rpyDt to calculate B's angular velocity in N, expressed in B.
  const Vector3<T> w_BN_B = rpy.CalcAngularVelocityInChildFromRpyDt(rpyDt);

  // To compute α (B's angular acceleration in N) due to the net moment 𝛕 on B,
  // rearrange Euler rigid body equation  𝛕 = I α + ω × (I ω)  and solve for α.
  const Vector3<T> wIw = w_BN_B.cross(I_ * w_BN_B);            // Expressed in B
  const Vector3<T> alpha_NB_B = I_.ldlt().solve(Tau_B - wIw);  // Expressed in B
  const Vector3<T> alpha_NB_N = R_NB * alpha_NB_B;             // Expressed in N

  // Calculate the 2nd time-derivative of rpy.
  const Vector3<T> rpyDDt =
      rpy.CalcRpyDDtFromRpyDtAndAngularAccelInParent(rpyDt, alpha_NB_N);

  // Recomposing the derivatives vector.
  VectorX<T> xDt(12);
  xDt << state.template tail<6>(), xyzDDt, rpyDDt;
  derivatives->SetFromVector(xDt);
}

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadrotorPlant<double>* quadrotor_plant,
    Eigen::Vector3d nominal_position) {
  auto quad_context_goal = quadrotor_plant->CreateDefaultContext();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = nominal_position;

  // Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
      4, quadrotor_plant->m() * quadrotor_plant->g() / 4);

  quadrotor_plant->get_input_port(0).FixValue(quad_context_goal.get(), u0);
  quad_context_goal->SetContinuousState(x0);

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
