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
                                                          Vector3<T>* p_NoBcm_N,
                                                          Vector3<T>* v_NBcm_N,
                                                          Vector3<T> *w_NB_N) {
  Vector3<T> p, v, w;
  const T t_roll_start = CalcTimeAtWhichRollingStarts();
  if (t < t_roll_start) {
    // Calculate Bcm's velocity in N, expressed in N at time t.
    const Vector3<T> v0_NBcm_N  = InitialVelocityBcm();
    const Vector3<T> u = CalcUnitVectorInSlidingDirection();
    v = v0_NBcm_N - muk_ * g_ * t * u;

    // Calculate Bcm's position from No, expressed in N at time t.
    const Vector3<T> p0 = InitialPositionBcm();
    p = p0 + v0_NBcm_N * t - 0.5 * muk_ * g_ * t * t * u;

  } else {
    // Once rolling, Bcm's velocity in N is constant (stays rolling).
    v = CalcVelocityBcmIfRolling();

    // Calculate Bcm's position from No, expressed in N at time t.
    const Vector3<T> p0 = InitialPositionBcm();
    p = p0 + v * t;
  }

  // Return results through method arguments.
  if (p_NoBcm_N != nullptr) *p_NoBcm_N = p;
  if (v_NBcm_N != nullptr) *v_NBcm_N = v;
  if (w_NB_N != nullptr) *w_NB_N = w;
}

// -----------------------------------------------------------------------------
// Explicitly instantiate on the most common scalar type.
template
class BowlingBallAnalyticalSolution<double>;

}  // namespace bowling_ball
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
