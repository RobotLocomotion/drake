
#include <algorithm>
#include <thread>
#include <chrono>
#include "DrakeSystem.h"

using namespace std;
using namespace Eigen;

DrakeSystem::DrakeSystem(const std::string &_name,
                         const CoordinateFramePtr& _continuous_state_frame,
                         const CoordinateFramePtr& _discrete_state_frame,
                         const CoordinateFramePtr& _input_frame,
                         const CoordinateFramePtr& _output_frame)
  : name(_name),
    continuous_state_frame(_continuous_state_frame),
    discrete_state_frame(_discrete_state_frame),
    input_frame(_input_frame),
    output_frame(_output_frame) {
  if (!continuous_state_frame) continuous_state_frame = make_shared<CoordinateFrame>(name+"ContinuousState");
  if (!discrete_state_frame) discrete_state_frame = make_shared<CoordinateFrame>(name+"DiscreteState");
  if (!input_frame) input_frame = make_shared<CoordinateFrame>(name+"Input");
  if (!output_frame) output_frame = make_shared<CoordinateFrame>(name+"Output");
  state_frame = make_shared<MultiCoordinateFrame>(name+"State",initializer_list<CoordinateFramePtr>({continuous_state_frame,discrete_state_frame}));
}

DrakeSystem::DrakeSystem(const std::string &_name, unsigned int num_continuous_states, unsigned int num_discrete_states,
                         unsigned int num_inputs, unsigned int num_outputs)
        : name(_name),
          input_frame(make_shared<CoordinateFrame>(name+"Input",num_inputs,"u")),
          continuous_state_frame(make_shared<CoordinateFrame>(name+"ContinuousState",num_continuous_states,"xc")),
          discrete_state_frame(make_shared<CoordinateFrame>(name+"DiscreteState",num_discrete_states,"xd")),
          state_frame(make_shared<MultiCoordinateFrame>(name+"State",initializer_list<CoordinateFramePtr>({continuous_state_frame,discrete_state_frame}))),
          output_frame(make_shared<CoordinateFrame>(name+"Output",num_outputs,"y")) { }

DrakeSystem::VectorXs DrakeSystem::getRandomState() {
  return DrakeSystem::VectorXs::Random(state_frame->getDim());
}

DrakeSystem::VectorXs DrakeSystem::getInitialState() {
  return getRandomState();
}

void DrakeSystem::simulate(double t0, double tf, const DrakeSystem::VectorXs &x0, const SimulationOptions& options) const {
  ode1(t0,tf,x0,options);
}

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
      throw runtime_error("Simulation is not keeping up with desired real-time factor -- behind by more than 1 (scaled) second at simulation time " + to_string(sim_time));
    }
  }
}


void DrakeSystem::ode1(double t0, double tf, const DrakeSystem::VectorXs& x0, const SimulationOptions& options) const {
  double t = t0, dt;
  TimePoint start = TimeClock::now();
  DrakeSystem::VectorXs x = x0;
  DrakeSystem::VectorXs u = DrakeSystem::VectorXs::Zero(input_frame->getDim());
  DrakeSystem::VectorXs y(output_frame->getDim());
  while (t<tf) {
    handle_realtime_factor(start, t, options.realtime_factor);

    dt = (std::min)(options.initial_step_size,tf-t);
    y = output(t,x,u);
    x += dt * dynamics(t, x, u);
    t += dt;
  }
}



CascadeSystem::CascadeSystem(const DrakeSystemPtr& _sys1, const DrakeSystemPtr& _sys2)
  : DrakeSystem("CascadeSystem"), sys1(_sys1), sys2(_sys2) {
  if (sys1->output_frame != sys2->input_frame)
    throw runtime_error("Cascade combination failed: output frame of "+sys1->name+" must match the input frame of "+sys2->name);
  input_frame = sys1->input_frame;
  output_frame = sys2->output_frame;
  continuous_state_frame = make_shared<MultiCoordinateFrame>("CascadeSystemContState",initializer_list<CoordinateFramePtr>({sys1->continuous_state_frame, sys2->continuous_state_frame}));
  discrete_state_frame = make_shared<MultiCoordinateFrame>("CascadeSystemDiscState",initializer_list<CoordinateFramePtr>({sys1->discrete_state_frame, sys2->discrete_state_frame}));
  state_frame = make_shared<MultiCoordinateFrame>("CascadeSystemState",initializer_list<CoordinateFramePtr>({continuous_state_frame,discrete_state_frame}));
}

DrakeSystem::VectorXs CascadeSystem::getX1(const VectorXs &x) const {
  DrakeSystem::VectorXs x1(sys1->state_frame->getDim());
  x1 << x.head(sys1->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim(),sys1->discrete_state_frame->getDim());
  return x1;
}

DrakeSystem::VectorXs CascadeSystem::getX2(const VectorXs &x) const {
  DrakeSystem::VectorXs x2(sys2->state_frame->getDim());
  x2 << x.segment(sys1->continuous_state_frame->getDim(),sys2->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim()+sys2->continuous_state_frame->getDim(),sys2->discrete_state_frame->getDim());
  return x2;
}

DrakeSystem::VectorXs CascadeSystem::dynamics(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) const {
  unsigned int num_xc1 = sys1->continuous_state_frame->getDim(),
               num_xc2 = sys2->continuous_state_frame->getDim();

  DrakeSystem::VectorXs x1 = getX1(x);
  DrakeSystem::VectorXs y1 = sys1->output(t,x1,u);
  DrakeSystem::VectorXs xdot(continuous_state_frame->getDim());
  if (num_xc1>0) xdot.head(num_xc1) = sys1->dynamics(t,x1,u);
  if (num_xc2>0) xdot.tail(num_xc2) = sys2->dynamics(t,getX2(x),y1);
  return xdot;
}

DrakeSystem::VectorXs CascadeSystem::update(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) const  {
  unsigned int num_xd1 = sys1->discrete_state_frame->getDim(),
          num_xd2 = sys2->discrete_state_frame->getDim();

  DrakeSystem::VectorXs x1 = getX1(x);
  DrakeSystem::VectorXs y1 = sys1->output(t,x1,u);
  DrakeSystem::VectorXs xn(discrete_state_frame->getDim());
  if (num_xd1>0) xn.head(num_xd1) = sys1->update(t,x1,u);
  if (num_xd2>0) xn.tail(num_xd2) = sys2->update(t,getX2(x),y1);
  return xn;
}

DrakeSystem::VectorXs CascadeSystem::output(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) const {
  DrakeSystem::VectorXs y1 = sys1->output(t,getX1(x),u);
  return sys2->output(t,getX2(x),y1);
}
