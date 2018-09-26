#pragma once

#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

// -----------------------------------------------------------------------------
namespace drake {
namespace multibody {
namespace benchmarks {
namespace bowling_ball {

/// This class provides an analytical solution for the motion of a spherically-
/// symmetric ball B that is in contact with a flat horizontal plane N.
/// The solution accurately predicts transition from sliding to rolling.
/// The horizontal plane N passes through a point No (the origin of N) and is
/// perpendicular to a vertically-upward unit vector Ny.  Two relevant points of
/// ball B are point Bcm (B's center of mass, located at B's geometric center)
/// and point BN (the point of B in contact with N -- the lowest point of B).
/// The gravitational forces on B are equivalent to `-m*g*Ny` at Bcm.
/// The contact force on B at point BN consists of an upward normal force
/// `F_normal*Ny` and a horizontal friction force `Fx*Nx + Fy*Nz` (where Nx and
/// Nz are horizontal unit vectors fixed in N).  At any time t, Bcm's position
/// from No is `x*Nx + r*Ny + z*Nz`, where x(t) and z(t) are time-dependent
/// variables and r is the ball's radius (constant).  Other constants are
/// m (B's mass), g (Earth's gravitational acceleration), I (B's moment of
/// inertia about any line passing through Bcm), and muk (coefficient of kinetic
/// friction between B and N).  Ball B has 5 degrees-of-freedom when it slides
/// on N and 3 degrees-of-freedom when it rolls on N.
///
/// The solution at time t depends on the initial (t = 0) values of:
/// ----------|----------------------------------------------------------
/// w_NB_N    | B's angular velocity in N, expressed in terms of Nx, Ny, Nz.
///           | `w_NB_N = wx*Nx + wy*Ny + wz*Nz`
/// v_NBcm_N  | Point Bcm's velocity in N, expressed in terms of Nx, Ny, Nz.
///           | `v_NBcm_N = ẋ*Nx + ż*Nz`
/// r_NoBcm_N | Position from point No to Bcm, expressed in terms of Nx, Ny, Nz.
///           | `r_NoBcm_N = x*Nx + r*Ny + z*Nz`
///
/// @note All units must be self-consistent (e.g., standard SI with MKS units).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
template<typename T>
class BowlingBallAnalyticalSolution {
 public:
  /// Construct a spherically-symmetric ball with default initialization.
  BowlingBallAnalyticalSolution() {}

  /// @param[in] r Ball B's radius (default is 0.10795 meters).
  void SetRadius(const T r) { r_ = r; }

  /// @param[in] m Ball B's mass (default is 6.35 kg).
  void SetMass(const T m) { m_ = m; }

  /// @param[in] I Bs moment of inertia about Bcm (default is 0.0296 kg*m^2).
  void SetMomentOfInertiaAboutBcm(const T I) { I_ = I; }

  /// @param[in] g Earth's gravitational acceleration (default is 9.8 m/s^2).
  void SetEarthGravityAcceleration(const T g) { g_ = g; }

  /// @param[in] muk Coefficient of kinetic friction (default is 0.12).
  void SetCoefficientOfKineticFriction(const T muk) { muk_ = muk; }

  /// Sets ball B's initial position, velocity, and angular velocity in N.
  /// @param[in] x0  Nx measure of Bcm's position from No at t = 0.
  /// @param[in] z0  Nz measure of Bcm's position from No at t = 0.
  /// @param[in] xDt0 Nx measure of Bcm's velocity in N at t = 0, i.e., ẋ(t=0).
  /// @param[in] zDt0 Nz measure of Bcm's velocity in N at t = 0, i.e., ż̇(t=0).
  /// @param[in] wx0  Nx measure of B's angular velocity in N at t = 0.
  /// @param[in] wy0  Ny measure of B's angular velocity in N at t = 0.
  /// @param[in] wz0  Nz measure of B's angular velocity in N at t = 0.
  void SetInitialValues(const T x0, const T z0, const T xDt0, const T zDt0,
                        const T wx0, const T wy0, const T wz0) {
    x0_ = x0;
    z0_ = z0;
    xDt0_ = xDt0;
    zDt0_ = zDt0;
    wx0_ = wx0;
    wy0_ = wy0;
    wz0_ = wz0;
  }

  /// @returns radius of ball B.
  T radius() { return r_; }

  /// Calculate the maximum difference between the analytical closed-form
  /// solution calculated by this class and the arguments to this method.
  /// @param[in] t The time at which the analytical solution is to be evaluated.
  /// @param[in] p_NoBcm_N Bcm's position from No,expressed in N.
  /// @param[in] v_NBcm_N, Bcm's velocity in N, expressed in N.
  /// @param[in] w_NB_N, B's angular velocity in N, expressed in N.
  /// @returns maximum absolute difference between the analytical closed-form
  /// solution calculated by this class and the arguments to this method
  /// (the arguments may be estimated or expected quantities).
  T CalcMaxDifferenceWithAnalyticalSolution(const T t,
                                            const Vector3<T>& p_NoBcm_N,
                                            const Vector3<T>& v_NBcm_N,
                                            const Vector3<T>& w_NB_N) {
    Vector3<T> p, v, a, w, alpha;
    CalcPositionAndMotion(t, &p, &v, &a, &w, &alpha);
    const Vector3<T> p_diff = p - p_NoBcm_N;
    const Vector3<T> v_diff = v - v_NBcm_N;
    const Vector3<T> w_diff = w - w_NB_N;
    const T p_max_difference = p_diff.template lpNorm<Eigen::Infinity>();
    const T v_max_difference = v_diff.template lpNorm<Eigen::Infinity>();
    const T w_max_difference = w_diff.template lpNorm<Eigen::Infinity>();
    T max_diff = p_max_difference;
    if (v_max_difference > max_diff) max_diff = v_max_difference;
    if (w_max_difference > max_diff) max_diff = w_max_difference;
    return max_diff;
  }

  /// @returns time t >= 0 at which rolling starts.
  T CalcTimeAtWhichRollingStarts() {
    const Vector3<T> v0_BN_N = CalcInitialContactPointVelocity();
    const T v0_BN_N_magnitude = v0_BN_N.norm();

    // If at time t = 0, |v_BN_N| <= v_epsilon, B is initially rolling on N.
    const double v_epsilon = CalcEpsilonContactSpeed();
    if (v0_BN_N_magnitude <= v_epsilon) return 0.0;

    // If muk is 0 or negative (or g is 0 or negative), assume B never rolls.
    // Rather than returning +infinite, just return largest double number.
    const T muk_g =   muk_ * g_;
    if (muk_g <= 0.0) return std::numeric_limits<double>::max();

    // Calculate time t at which B transitions from sliding to rolling.
    const Vector3<T> v0_NBcm_N = InitialVelocityBcm();
    const Vector3<T> vRoll = CalcVelocityBcmIfRolling();
    const Vector3<T> vDiff = v0_NBcm_N - vRoll;
    const Vector3<T> u = CalcUnitVectorInSlidingDirection();
    return 1 / muk_g * vDiff.dot(u);
  }

  /// Calculate B's position, velocity, and angular velocity at time t.
  /// @param[in] t The time at which the solution is to be evaluated.
  /// @param[out] p_NoBcm_N Bcm's position from No,expressed in N (at time t).
  /// @param[out] v_NBcm_N, Bcm's velocity in N, expressed in N (at time t).
  /// @param[out] a_NBcm_N, Bcm's acceleration in N, expressed in N (at time t).
  /// @param[out] w_NB_N, B's angular velocity in N, expressed in N (at time t).
  /// @param[out] alpha_NB_N, B's angular acceleration in N, expressed in N.
  void CalcPositionAndMotion(const T t,
       Vector3<T>* p_NoBcm_N, Vector3<T>* v_NBcm_N, Vector3<T>* a_NBcm_N,
                              Vector3<T>* w_NB_N,  Vector3<T>* alpha_NB_N);

 private:
  // Get vertically-upward unit vector, expressed in N (i.e., Ny).
  Vector3<T> VerticallyUpwardUnitVector() const {
    return Vector3<T>(0, 1, 0);
  }

  // Get position vector from No to Bcm (B's center of mass) at time t = 0.
  Vector3<T> InitialPositionVectorNoToBcm() const {
    return Vector3<T>(x0_, r_, z0_);
  }

  // Get position vector from Bcm to BN, expressed in N.
  Vector3<T> PositionVectorBcmToBN() const {
    return Vector3<T>(0, -r_, 0);
  }

  // Get velocity of Bcm (B's center of mass) in N at time t = 0.
  Vector3<T> InitialVelocityBcm() const {
    return Vector3<T>(xDt0_, 0, zDt0_);
  }

  // Get B's angular velocity in N, expressed in N, at time t = 0.
  Vector3<T> InitialAngularVelocity() const {
    return Vector3<T>(wx0_, wy0_, wz0_);
  }

  // Calculate velocity of BN (B's contact point with N).
  // @param[in] v An array with elements ẋ, ż, wx, wy, wz.
  // @returns v_NBN_N, BN's velocity in N, expressed in N.
  Eigen::Vector3d CalcContactPointVelocity(const Vector5<T>& v) const {
    const Eigen::Vector3d v_Bcm_N = SelectVelocityBcm(v);
    const Eigen::Vector3d w_BN_N = SelectAngularVelocity(v);
    const Eigen::Vector3d p_BcmBN_N = PositionVectorBcmToBN();
    return v_Bcm_N + w_BN_N.cross(p_BcmBN_N);
  }

  // Calculate velocity of BN (B's contact point with N) at time t = 0.
  Vector3<T> CalcInitialContactPointVelocity() const {
    Vector5<T> v;
    v << xDt0_, zDt0_, wx0_, wy0_, wz0_;
    return CalcContactPointVelocity(v);
  }

  // Calculate velocity of Bcm (B's center of mass) when B rolls on N.
  Vector3<T> CalcVelocityBcmIfRolling() const {
    const Vector3<T> w0 = InitialAngularVelocity();
    const Vector3<T> v0 = InitialVelocityBcm();
    const Vector3<T> n = VerticallyUpwardUnitVector();
    const Vector3<T> angular_momentum_constant = I_*w0  + m_*r_*n.cross(v0);
    const T s = r_ / (I_ + m_*r_*r_);
    return s * angular_momentum_constant.cross(n);
  }

  // Calculate unit vector in the direction that BN initially slides on N.
  Vector3<T> CalcUnitVectorInSlidingDirection() const {
    const Vector3<T> v0_BN_N = CalcInitialContactPointVelocity();
    const T v0_BN_N_magnitude = v0_BN_N.norm();
    DRAKE_ASSERT(v0_BN_N_magnitude > 0);
    return v0_BN_N / v0_BN_N_magnitude;
  }

  // Selects proper elements of `v` to create B's angular velocity in N.
  // @param[in] v Array with elements ẋ, ż, wx, wy, wz.
  // @returns w_NB_N, B's angular velocity in N, expressed in N.
  Vector3<T> SelectAngularVelocity(const Vector5<T>& v) const {
    return Vector3<T>(v[2], v[3], v[4]);
  }

  // Selects proper elements of `v` to create Bcm's velocity in N.
  // @param[in] v Array with elements ẋ, ż, wx, wy, wz.
  // @returns w_NB_N, B's angular velocity in N, expressed in N.
  Vector3<T> SelectVelocityBcm(const Vector5<T>& v) const {
    return Vector3<T>(v[0], 0, v[1]);
  }

  // @returns epsilon_v, a very small contact speed for bowling.
  // Assumes there may be as many of 6 of the 53 mantissa bits to be inaccurate
  // and bowling speeds of up to 20 mph = 8.94 m/s.
  static double CalcEpsilonContactSpeed() {
    constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
    constexpr double kFast_bowl = 8.94;  // Fast bowling is 20 mph = 8.94 m/s.
    return 64 * kEpsilon * kFast_bowl;   // 2^6 * machine_epsilon * high_speed.
  }

  // Class data with default bowling ball physical parameters (constants).
  T r_{ 0.10795 };      // Ball's radius.
  T m_{ 6.35 };         // Ball's mass.
  T I_{ 0.0296 };       // Ball's moment of inertia about Bcm.
  T g_{ 9.8 };          // Earth's local gravitational acceleration.
  T muk_{ 0.12 };       // Ball-to-plane coefficient of kinetic friction.

  // Class data with default initial values (at time t = 0).
  T x0_{ 0 };           // Initial value of x.
  T z0_{ 0 };           // Initial value of z.
  T xDt0_{ 0 };         // Initial value of ẋ.
  T zDt0_{ 8 };         // Initial value of ż.
  T wx0_{ 0 };          // Initial value of wx.
  T wy0_{ 0 };          // Initial value of wy.
  T wz0_{ -8 * M_PI };  // Initial value of wz (-4 revolutions/second).
};

}  // namespace bowling_ball
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
