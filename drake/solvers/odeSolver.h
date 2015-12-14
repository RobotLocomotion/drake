#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <chrono>
#include "DrakeSystem.h"

namespace Drake {

typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock, but it seems to not compile on all platforms (e.g. MSVC 2013 Win64)
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock,TimeDuration> TimePoint;

template<typename state_type> using ode45 = boost::numeric::odeint::runge_kutta4<state_type>;
template<typename state_type> using ode1 = boost::numeric::odeint::euler<state_type>;
template<typename state_type> using odeRKD = boost::numeric::odeint::runge_kutta_dopri5<state_type>;

template <typename state_type, typename input_type, typename output_type, typename stepper_type>
class ODESolver {	
	const ConstDrakeSystemPtr& sys;
	DrakeSystem::SimulationOptions sim_options;
	stepper_type stepper;

	void system_function(state_type const& x, state_type &xdot, double t) const {
		xdot = sys->dynamics(t, x, input_type());
	}

	inline void handle_realtime_factor(const TimePoint& wall_clock_start_time, double sim_time, double realtime_factor) const
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

public:
	ODESolver(ConstDrakeSystemPtr const& system, DrakeSystem::SimulationOptions const& options) 
		: sys(system), sim_options(options) {
		};

	void solve(double t0, double tf, state_type const& x0) {
		using namespace std::placeholders;
		double t = t0, dt;
		TimePoint start = TimeClock::now();
		state_type x = x0;
		input_type u;
		output_type y;
		auto ode_system = std::bind(&ODESolver::system_function, this, _1,  _2, _3);

		while (t < tf) {
			handle_realtime_factor(start, t, sim_options.realtime_factor);
			dt = (std::min)(sim_options.initial_step_size,tf-t);
			y = sys->output(t,x,u);
			stepper.do_step(ode_system, x, t, dt);
			t += dt;
		}	
	}
};

}