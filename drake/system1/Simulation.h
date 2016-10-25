#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "drake/system1/simulation_options.h"
#include "drake/system1/vector.h"

namespace drake {

/** @defgroup simulation Simulation
*@{
*@brief Algorithms for simulating dynamical systems
*@}
*/

typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock,
                                              // but it seems to not compile on
                                              // all platforms (e.g. MSVC 2013
                                              // Win64)
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock, TimeDuration> TimePoint;

/*!
 * Determines whether the simulation time has lagged behind the real time beyond
 * the specified <pre>timeout_seconds</pre>, after accounting for the real-time
 * factor. If it has, throw a <pre>std::runtime_error</pre> exception.
 *
 * @param wall_clock_start_time The the simulation's start time.
 * @param sim_time The current simulation time.
 * @param realtime_factor The simulation's desired real-time factor. This
 * is the speed at which the simulation should run relative to real-time.
 * For example, 0 means the simulation should run as fast as possible,
 * 1.0 means the simulation should run at real-time, and 2.0 means the
 * simulation should run at 2X real-time speed.
 * @param timeout_seconds The maximum difference between the current time and
 * the desired time (as determined based on the current simulation time
 * and real-time factor) before which an exception is thrown.
 * return True if the realtime factor was successfully handled. False otherwise.
 */
inline bool handle_realtime_factor(const TimePoint& wall_clock_start_time,
                                   double sim_time, double realtime_factor,
                                   double timeout_seconds) {
  bool result = true;
  if (realtime_factor > 0.0) {
    TimePoint wall_time = TimeClock::now();
    TimePoint desired_time =
        wall_clock_start_time + TimeDuration(sim_time / realtime_factor);
    if (desired_time > wall_time) {
      // could probably just call sleep_until, but just in case
      std::this_thread::sleep_until(desired_time);
    } else if (wall_time >
               desired_time + TimeDuration(timeout_seconds / realtime_factor)) {
      result = false;
    }
  }

  return result;
}

/** simulate
 * @brief Runs a simulation given a model, its initial conditions, and a number
 *of simulation parameters
 * @ingroup simulation
 *
 * Currently runs with a fixed step integrator using the initial step size in
 * \p options and stepping from initial time \p ti to final time \p tf.
 * There is no error control; if you have accuracy or stability problems try
 * a smaller step size.
 *
 * @param sys The system being simulated.
 * @param ti The initial time of the simulation.
 * @param tf The final time of the simulation.
 * @param xi The state vector of the system being simulated.
 * @param options The simulation options.
 * @return The final simulation time.
 */
template <typename System>
double simulate(const System& sys, double ti, double tf,
              const typename System::template StateVector<double>& xi,
              const SimulationOptions& options) {
  TimePoint start = TimeClock::now();
  typename System::template StateVector<double> x(xi), x1est, xdot0, xdot1;
  typename System::template InputVector<double> u(
      Eigen::VectorXd::Zero(getNumInputs(sys)));
  typename System::template OutputVector<double> y;

  bool rt_warning_printed = false;

  // Take steps from ti to tf.
  double t = ti;
  while (t < tf && !options.should_stop(t)) {
    double realtime_factor = options.realtime_factor;
    if (realtime_factor < 0.0) {
      realtime_factor = 0.0;
    }
    if (!handle_realtime_factor(start, t, realtime_factor,
                                options.timeout_seconds)) {
      std::stringstream error_msg;
      error_msg
          << "The simulation is not keeping up with desired real-time factor. "
          << "It is behind by more than " << options.timeout_seconds
          << " (scaled) second at simulation time " << t;

      if (options.warn_real_time_violation) {
        if (!rt_warning_printed) {
          std::cerr << "WARNING: " << error_msg.str() << std::endl;
          rt_warning_printed = true;  // Suppress future warnings.
        }
      } else {
        throw std::runtime_error(error_msg.str());
      }
    }
    const double dt = (std::min)(options.initial_step_size, tf - t);

    // Output is at t0, x0, u0.
    y = sys.output(t, x, u);

    // This is an RK2 integrator (explicit trapezoid rule).
    // First stage: xd0 = dynamics(t0,x0,u0).
    xdot0 = sys.dynamics(t, x, u);
    x1est = toEigen(x) + dt * toEigen(xdot0);  // explicit Euler step
    t += dt;

    // Second stage: xd1 = dynamics(t1,x1est,u0).
    xdot1 = sys.dynamics(t, x1est, u);

    // 2nd order result: x = x0 + dt (xd0+xd1)/2.
    x = toEigen(x) + (dt / 2) * (toEigen(xdot0) + toEigen(xdot1));
  }

  return t;
}

/** simulate
 * @brief Runs a simulation using the default simulation options
 * @ingroup simulation
 *
 */
template <typename System>
void simulate(const System& sys, double t0, double tf,
              const typename System::template StateVector<double>& x0) {
  simulate(sys, t0, tf, x0, SimulationOptions());
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

}  // end namespace drake
