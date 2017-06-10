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
/// @note All units must be self-consistent (e.g., standard SI units).
class MassDamperSpringAnalyticalSolution {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MassDamperSpringAnalyticalSolution);

  /// This constructs the aforementioned mass-damper-spring system.
  ///
  /// @param[in] mass Mass of system (particle Q).
  /// @param[in] b Linear damping constant.
  /// @param[in] k Linear spring constant.
  MassDamperSpringAnalyticalSolution(const double mass,
                                     const double b,
                                     const double k) :
                                     m_(mass), b_(b), k_(k) {}

  /// Sets the initial values of x and ẋ for `this` system.
  ///
  /// @param[in] x0 Initial value of x (value of x at time t = 0).
  /// @param[in] xDt0 Initial value of ẋ (value of ẋ at time t = 0).
  void SetInitialValue(const double x0, const double xDt0) {
    x0_ = x0;  xDt0_ = xDt0;
  }

  /// For `this` mass-damper-spring system, and with the given initial
  /// values, this method calculates the values of x, ẋ, ẍ at time t.
  ///
  /// @param[in] t The value of time at which output is requested.
  ///
  /// @returns Three-element matrix consisting of x, ẋ, ẍ, respectively.
  Eigen::Vector3d CalculateOutput(const double t) const;

  /// Returns x (Nx measure of Q's position from No) at time t.
  double get_x (const double t)  { return CalculateOutput(t)(0); }

  /// Returns ẋ (Nx measure of Q's velocity in N) at time t.
  double get_xDt(const double t)  { return CalculateOutput(t)(1); }

  /// Returns ẍ (Nx measure of Q's acceleration in N) at time t.
  double get_xDtDt(const double t)  { return CalculateOutput(t)(2); }

 private:
  // Class data.
  // m_     |  Mass of system.
  // b_     |  Linear damping constant.
  // k_     |  Linear spring constant.
  // x0_    |  Initial value of x (at time t = 0).
  // xDt0_  |  Initial value of ẋ (at time t = 0).
  double m_, b_, k_, x0_, xDt0_;
};


}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

