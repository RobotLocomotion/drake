#ifndef DRAKE_SIMULATION_H
#define DRAKE_SIMULATION_H

#include <thread>
#include <chrono>

namespace Drake {

  // simulation options
  struct SimulationOptions {
    double realtime_factor;  // 1 means try to run at realtime speed, < 0 is run as fast as possible
    double initial_step_size;

    SimulationOptions() :
            realtime_factor(-1.0),
            initial_step_size(0.01)
    {};
  };
  const static SimulationOptions default_simulation_options;

  typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock, but it seems to not compile on all platforms (e.g. MSVC 2013 Win64)
  typedef std::chrono::duration<double> TimeDuration;
  typedef std::chrono::time_point<TimeClock,TimeDuration> TimePoint;

  inline void handle_realtime_factor(const TimePoint& wall_clock_start_time, double sim_time, double realtime_factor)
  {
    if (realtime_factor>0.0) {
      TimePoint wall_time = TimeClock::now();
      TimePoint desired_time = wall_clock_start_time + TimeDuration(sim_time/realtime_factor);
      if (desired_time>wall_time) { // could probably just call sleep_until, but just in case
        std::this_thread::sleep_until(desired_time);
      } else if (wall_time>desired_time+TimeDuration(1.0/realtime_factor)) {
        throw std::runtime_error("Simulation is not keeping up with desired real-time factor -- behind by more than 1 (scaled) second at simulation time " + std::to_string(sim_time));
      }
    }
  }

  template <typename System>
  void simulate(const System& sys, double t0, double tf, const typename System::template StateVector<double>& x0, const SimulationOptions& options) {
    const static int num_states = SystemTraits<System>::num_states;
    const static int num_inputs = SystemTraits<System>::num_inputs;
    const static int num_outputs = SystemTraits<System>::num_outputs;


    double t = t0, dt;
//    std::cout << "x0 = " << x0.transpose() << std::endl;
    TimePoint start = TimeClock::now();
    Eigen::Matrix<double,num_states,1> x = toEigen(x0);
    Eigen::Matrix<double,num_states,1> xdot;
    Eigen::Matrix<double,num_inputs,1> u; u.setConstant(0);
    Eigen::Matrix<double,num_outputs,1> y;
    while (t<tf) {
      handle_realtime_factor(start, t, options.realtime_factor);

//      std::cout << "t=" << t << ", x = " << x.transpose() << std::endl;
      dt = (std::min)(options.initial_step_size,tf-t);
      y = toEigen(output<System,double>(sys,t,x,u));   // todo: can I get rid of the explicit template parameter specifications?
      xdot = toEigen(dynamics<System,double>(sys,t,x,u));
      x += dt * xdot;
      t += dt;
    }
  }

  template <typename System>
  void simulate(const System& sys, double t0, double tf, const typename System::template StateVectorType<double>& x0)  {
    simulate(sys,t0,tf,x0,default_simulation_options);
  }

}  // end namespace Drake

#endif //DRAKE_SIMULATION_H
