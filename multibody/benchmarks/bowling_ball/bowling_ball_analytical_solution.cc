// -----------------------------------------------------------------------------
#include "drake/multibody/benchmarks/bowling_ball/bowling_ball_analytical_solution.h"

// -----------------------------------------------------------------------------
namespace drake {
namespace multibody {
namespace benchmarks {
namespace bowling_ball {

// -----------------------------------------------------------------------------
template<typename T>
void BowlingBallAnalyticalSolution<T>::CalcPositionAndMotion(const T t,
     Vector3<T>* p_NoBcm_N, Vector3<T>* v_NBcm_N, Vector3<T>* a_NBcm_N,
                            Vector3<T>* w_NB_N,  Vector3<T>* alpha_NB_N) {
  // Position, velocity, acceleration to be calculated and returned.
  Vector3<T> p, v, a;

  // Initial position/velocity are needed -- regardless if B rolls or slides.
  const Vector3<T> p0 = InitialPositionVectorNoToBcm();
  const Vector3<T> v0 = InitialVelocityBcm();

  // Solution depends on whether B slides or rolls.
  const T t_roll_start = CalcTimeAtWhichRollingStarts();
  if (t < t_roll_start) {
    // Sliding: Calculate Bcm's velocity and acceleration in N at time t.
    const Vector3<T> u = CalcUnitVectorInSlidingDirection();
    a = -muk_ * g_ * u;                 // Bcm's acceleration in N is constant.
    v = v0 + a * t;                     // Bcm's velocity in N at time t.
    p = p0 + v0 * t + 0.5 * a * t * t;  // Bcm's position from No at t.
  } else {
    // Once rolling, Bcm's velocity in N is constant (constant-speed rolling).
    a = Vector3<T>::Zero();           // Bcm's acceleration in N is zero.
    v = CalcVelocityBcmIfRolling();   // Bcm's velocity in N is constant.
    p = p0 + v * (t - t_roll_start);  // Bcm's position from No (straight line).

    // If t_roll_start != 0, add amount slid from t = 0 to t = t_roll_start.
    if (t_roll_start != 0) {
      const Vector3<T> u = CalcUnitVectorInSlidingDirection();
      const Vector3<T> a_slide = -muk_ * g_ * u;
      const Vector3<T> p_slide = v0 * t_roll_start
                               + 0.5 * a_slide * t_roll_start * t_roll_start;
      p += p_slide;
    }
  }

  // Calculate B's angular velocity and angular acceleration in N at time t.
  const Vector3<T> w0 = InitialAngularVelocity();
  const Vector3<T> n = VerticallyUpwardUnitVector();
  const T s = m_ * r_ / I_;
  const Vector3<T> w = w0 + s * n.cross(v0 - v);
  const Vector3<T> alpha = -s * n.cross(a);

  // Return results through method arguments.
  if (p_NoBcm_N != nullptr) *p_NoBcm_N = p;
  if (v_NBcm_N != nullptr) *v_NBcm_N = v;
  if (a_NBcm_N != nullptr) *a_NBcm_N = a;
  if (w_NB_N != nullptr) *w_NB_N = w;
  if (alpha_NB_N != nullptr) *alpha_NB_N = alpha;
}

// -----------------------------------------------------------------------------
// Explicitly instantiate on the most common scalar type.
template
class BowlingBallAnalyticalSolution<double>;

}  // namespace bowling_ball
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
