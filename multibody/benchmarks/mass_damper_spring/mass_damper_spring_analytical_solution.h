#pragma once

#include <cmath>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {

/// This class provides an analytical solution to a mass-damper-spring system.
/// The system consists of a particle Q of mass m that can only move left/right
/// on flat Earth (frame N).  Particle Q is connected by an ideal translational
/// spring/damper. The other end of the spring/damper is welded to point No
/// (the origin of frame N).  Q's position from No is x*Nx where x(t) is a time-
/// dependent variable (to-be-calculated) and Nx is a horizontal unit vector
/// fixed in Earth (N).  The spring force on Q is -k*x*Nx, where k is a spring
/// constant.  The damper force on Q is -b*ẋ*Nx where b is a damper constant
/// and ẋ is the time-derivative of x.
///
/// @note All units must be self-consistent (e.g., standard SI with MKS units).
///       The solution provided herein is also applicable to a rotating system,
///       e.g., having rigid-body inertia, rotational damper, rotational spring.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class MassDamperSpringAnalyticalSolution {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MassDamperSpringAnalyticalSolution);

  /// This constructs the aforementioned mass-damper-spring system.
  ///
  /// @param[in] mass Mass of system (particle Q).
  /// @param[in] b Linear damping constant.
  /// @param[in] k Linear spring constant.
  MassDamperSpringAnalyticalSolution(const T& mass, const T& b, const T& k) :
                                     m_(mass), b_(b), k_(k) {}

  /// Sets the initial values of x and ẋ for `this` system.
  ///
  /// @param[in] x0 Initial value of x (value of x at time t = 0).
  /// @param[in] xDt0 Initial value of ẋ (value of ẋ at time t = 0).
  void SetInitialValue(const T& x0, const T& xDt0)  { x0_ = x0;  xDt0_ = xDt0; }

  /// For `this` mass-damper-spring system, and with the given initial
  /// values, this method calculates the values of x, ẋ, ẍ at time t.
  ///
  /// @param[in] t The value of time at which output is requested.
  ///
  /// @returns Three-element matrix consisting of x, ẋ, ẍ, respectively.
  Vector3<T> CalculateOutput(const T& t) const;

  /// Returns x (Nx measure of Q's position from No) at time t.
  T get_x(const T& t) const  { return CalculateOutput(t)(0); }

  /// Returns ẋ (Nx measure of Q's velocity in N) at time t.
  T get_xDt(const T& t) const  { return CalculateOutput(t)(1); }

  /// Returns ẍ (Nx measure of Q's acceleration in N) at time t.
  T get_xDtDt(const T& t) const  { return CalculateOutput(t)(2); }

 private:
  // Class data.
  // m_     |  Mass of system.
  // b_     |  Linear damping constant.
  // k_     |  Linear spring constant.
  // x0_    |  Initial value of x (at time t = 0).
  // xDt0_  |  Initial value of ẋ (at time t = 0).
  T m_, b_, k_, x0_, xDt0_;

  // Calculate `this` mass-damper-spring system's natural frequency (wn).
  T CalculateNaturalFrequency() const  {
    using std::sqrt;
    return sqrt(k_ / m_);
  }

  // Calculate `this` mass-damper-spring system's damping ratio (zeta).
  T CalculateDampingRatio() const  {
    using std::sqrt;
    return b_ / (2 * sqrt(m_ * k_));
  }

  // Calculates the values of x, ẋ, ẍ at time t associated with the ODE
  // ẍ  +  2 ζ ωₙ ẋ  +  ωₙ²  =  0  and the given initial values.
  //
  // @param[in] zeta Damping ratio (ζ) associated with this 2nd-order ODE.
  //                 There are no units for zeta (ζ) (dimensionless quantity).
  // @param[in] wn  Natural frequency (ωₙ) associated with this 2nd-order ODE.
  //                The units of wn are typically in rad/sec.
  // @param[in] x0 Initial value of x (value of x at time t = 0).
  //               For translation, the units of x0 are typically in meters.
  // @param[in] xDt0 Initial value of ẋ (value of ẋ at time t = 0).
  //                 For translation, the units of xDt0 are typically in m/s.
  // @param[in] t The value of time at which output is requested (usually in s)
  //              The units of t are typically in seconds.
  //
  // @returns Three-element matrix consisting of x, ẋ, ẍ, respectively.
  static Vector3<T> CalculateOutputImpl(const T& zeta, const T& wn,
                                        const T& x0, const T& xDt0, const T& t);
};


}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

