#ifndef DRAKE_SIMULATION_H
#define DRAKE_SIMULATION_H

#include <thread>
#include <chrono>
#include <stdexcept>

#include <iostream>

namespace Drake {

/** @defgroup simulation Simulation
*@{
*@brief Algorithms for simulating dynamical systems
*@}
*/

// simulation options
struct SimulationOptions {
  double realtime_factor;  // 1 means try to run at realtime speed, 0 is run as
                           // fast as possible, < 0 means use default
  double initial_step_size;
  double timeout_seconds;

  SimulationOptions()
      : realtime_factor(-1.0), initial_step_size(0.01), timeout_seconds(1.0){};
};
const static SimulationOptions default_simulation_options;

typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock,
                                              // but it seems to not compile on
                                              // all platforms (e.g. MSVC 2013
                                              // Win64)
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock, TimeDuration> TimePoint;

inline void handle_realtime_factor(const TimePoint& wall_clock_start_time,
                                   double sim_time, double realtime_factor,
                                   double timeout_seconds) {
  if (realtime_factor > 0.0) {
    TimePoint wall_time = TimeClock::now();
    TimePoint desired_time =
        wall_clock_start_time + TimeDuration(sim_time / realtime_factor);
    if (desired_time >
        wall_time) {  // could probably just call sleep_until, but just in case
      std::this_thread::sleep_until(desired_time);
    } else if (wall_time >
               desired_time + TimeDuration(timeout_seconds / realtime_factor)) {
      throw std::runtime_error(
          "Simulation is not keeping up with desired real-time factor -- "
          "behind by more than 1 (scaled) second at simulation time " +
          std::to_string(sim_time));
    }
  }
}

/** simulate
 * @brief Runs a simulation given a model, it's initial conditions, and a number
 *of simulation parameters
 * @ingroup simulation
 *
 * Currently runs with a fixed step integrator using the initial step size in 
 * \p options and stepping from initial time \p ti to final time \p tf.
 * There is no error control; if you have accuracy or stability problems try
 * a smaller step size.
 */
template <typename System>
void simulate(const System& sys, double ti, double tf,
              const typename System::template StateVector<double>& xi,
              SimulationOptions& options) {
  if (options.realtime_factor < 0.0) options.realtime_factor = 0.0;

  TimePoint start = TimeClock::now();
  typename System::template StateVector<double> x(xi), x1est, xdot0, xdot1;
  typename System::template InputVector<double> u(
      Eigen::VectorXd::Zero(getNumInputs(sys)));
  typename System::template OutputVector<double> y;

  // size_t simCycleCnt = 0;

  // Take steps from ti to tf.
  double t = ti;
  while (t < tf) {
    handle_realtime_factor(start, t, options.realtime_factor,
                           options.timeout_seconds);
    const double dt = (std::min)(options.initial_step_size, tf - t);

    // std::cout << "Drake::simulate: [" << simCycleCnt++ << "] sim time " << t <<  std::endl;

    // Output is at t0, x0, u0.

    y = sys.output(t, x, u);

    // This is an RK2 integrator (explicit trapezoid rule).
    // First stage: xd0 = dynamics(t0,x0,u0).
    xdot0 = sys.dynamics(t, x, u);
    x1est = toEigen(x) + dt * toEigen(xdot0); // explicit Euler step
    t += dt;

    // Second stage: xd1 = dynamics(t1,x1est,u0).
    xdot1 = sys.dynamics(t, x1est, u);

    // 2nd order result: x = x0 + dt (xd0+xd1)/2.
    x = toEigen(x) + (dt/2) * (toEigen(xdot0) + toEigen(xdot1));

    // std::cout << "Drake::simulate: Done with one simulation cycle. Press any key to continue." << std::endl;
    // getchar();
  }
}

/** simulate
 * @brief Runs a simulation using the default simulation options
 * @ingroup simulation
 *
 */
template <typename System>
void simulate(const System& sys, double t0, double tf,
              const typename System::template StateVectorType<double>& x0) {
  simulate(sys, t0, tf, x0, default_simulation_options);
}

/** simulate
 * @brief Runs a simulation using the default simulation options
 * @ingroup simulation
 *
 */
template <typename System>
void simulate(const System& sys, double t0, double tf) {
  auto x0 = getInitialState(sys);
  simulate(sys, t0, tf, x0);
}

}  // end namespace Drake

#endif  // DRAKE_SIMULATION_H
