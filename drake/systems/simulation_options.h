#pragma once

namespace drake {

/**
 * Defines the user-tunable simulation options.
 */
struct SimulationOptions {
  /**
   * The desired simulation real-time factor. The real-time factor is the
   * simulation time divided by the real-world time. A value of 1 means the
   * simulation should run in real-time. A value greater than one means the
   * simulation should run faster than real-time. A value less than one means
   * the simulation should run slower than real-time.
   *
   * A value of 0 means the simulation should run as fast as possible.
   *
   * Negative values indicate the default real-time factor should be used, which
   * is to run as fast as possible.
   *
   * Its default value is -1.
   */
  double realtime_factor;

  /**
   * The initial time increment the simulator should take. It's default value
   * is 0.01.
   */
  double initial_step_size;

  /**
   * The maximum difference between the current time and the desired time (as
   * determined based on the current simulation time and real-time factor)
   * before which an exception is thrown. Its default value is 1.0.
   */
  double timeout_seconds;

  /**
   * Dermines what happens if the simulation's timing is delayed by more than
   * `timeout_seconds`. A value of `true` results in a warning being printed.
   * A value of `false` results in an exception being raised. Its default value
   * is false.
   */
  bool warn_real_time_violation;

  /**
   * Enables a custom simulation termination condition. This function is called
   * each cycle of the simulation loop. The input parameter of type double is
   * the current simulation time. If the function returns true, the simulation
   * is terminated. The default for this is a function that always returns
   * false.
   */
  std::function<bool(double)> should_stop;

  SimulationOptions()
      : realtime_factor(-1.0),
        initial_step_size(0.01),
        timeout_seconds(1.0),
        warn_real_time_violation(false),
        should_stop([](double sim_time) { return false; }) {}
};

}  // end namespace drake
