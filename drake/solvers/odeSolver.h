#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <chrono>
#include "DrakeSystem.h"

typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock, but it seems to not compile on all platforms (e.g. MSVC 2013 Win64)
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock,TimeDuration> TimePoint;

void handle_realtime_factor(const TimePoint& wall_clock_start_time, double sim_time, double realtime_factor)
{
  if (realtime_factor > 0.0) {
    TimePoint wall_time = TimeClock::now();
    TimePoint desired_time = wall_clock_start_time + TimeDuration(sim_time/realtime_factor);
    if (desired_time>wall_time) { // could probably just call sleep_until, but just in case
      std::this_thread::sleep_until(desired_time);
    } else if (wall_time>desired_time+TimeDuration(1.0/realtime_factor)) {
      throw std::runtime_error("Simulation is not keeping up with desired real-time factor -- behind by more than 1 (scaled) second at simulation time " + std::to_string(sim_time));
    }
  }
}

namespace Drake {

template <typename state_type>
class ODESolver {
	
	const ConstDrakeSystemPtr& sys;
	DrakeSystem::SimulationOptions sim_options;

	void dynamics(state_type const & x, state_type &xdot, double t) {
		Eigen::VectorXd u = Eigen::VectorXd::Zero(sys->input_frame->getDim());
		xdot = sys->dynamics(t, x, u);
	}
	
	void write_dynamics(state_type const& x, const double t) {
		std::cout << t << std::endl;
	}

public:
	ODESolver(ConstDrakeSystemPtr const& system, DrakeSystem::SimulationOptions const& options) 
		: sys(system), sim_options(options) {};

	void solve(double t0, double tf, state_type const& x0) {
		double t = t0, dt;
		TimePoint start = TimeClock::now();
		Eigen::VectorXd x = x0;
		Eigen::VectorXd u = Eigen::VectorXd::Zero(sys->input_frame->getDim());
		Eigen::VectorXd y(sys->output_frame->getDim());
		while (t<tf) {
			handle_realtime_factor(start, t, sim_options.realtime_factor);
			dt = (std::min)(sim_options.initial_step_size,tf-t);
			y = sys->output(t,x,u);
			x += dt * sys->dynamics(t, x, u);
			t += dt;
		}	
	}

	void integrate(double t0, double tf, state_type &x0) {
		boost::numeric::odeint::integrate(dynamics, x0, t0, tf, 0.01, write_dynamics);
	}
};

}