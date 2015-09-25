
#include <algorithm>
#include <thread>
#include <chrono>
#include "DrakeSystem.h"
#include "drakeGradientUtil.h"
#include "drakeUtil.h"
#include "LinearSystem.h"

using namespace std;
using namespace Eigen;

DrakeSystem::DrakeSystem(const string &_name,
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

DrakeSystem::DrakeSystem(const string &_name, unsigned int num_continuous_states, unsigned int num_discrete_states,
                         unsigned int num_inputs, unsigned int num_outputs)
        : name(_name),
          input_frame(make_shared<CoordinateFrame>(name+"Input",num_inputs,"u")),
          continuous_state_frame(make_shared<CoordinateFrame>(name+"ContinuousState",num_continuous_states,"xc")),
          discrete_state_frame(make_shared<CoordinateFrame>(name+"DiscreteState",num_discrete_states,"xd")),
          state_frame(make_shared<MultiCoordinateFrame>(name+"State",initializer_list<CoordinateFramePtr>({continuous_state_frame,discrete_state_frame}))),
          output_frame(make_shared<CoordinateFrame>(name+"Output",num_outputs,"y")) { }

VectorXd DrakeSystem::getRandomState() {
  return VectorXd::Random(state_frame->getDim());
}

VectorXd DrakeSystem::getInitialState() {
  return getRandomState();
}

DrakeSystemPtr DrakeSystem::timeInvariantLQR(const Eigen::VectorXd& x0, const Eigen::VectorXd& u0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  if (!isTimeInvariant())
  throw runtime_error("DrakeSystem::timeInvariantLQR.  Time-invariant LQR only makes sense for time invariant systems.");

  if (discrete_state_frame->getDim()>0)
  throw runtime_error("DrakeSystem::timeInvariantLQR.  Discrete-time LQR is not implemented yet (but would be easy).");
  if (continuous_state_frame->getDim()<1)
  throw runtime_error("DrakeSystem::timeInvariantLQR.  This system has no continuous states.");

  int num_x = continuous_state_frame->getDim(),
      num_u = input_frame->getDim();

  Eigen::VectorXd xu(num_x+num_u);
  xu << x0, u0;
  Drake::TaylorVecX xu_taylor = Drake::initTaylorVecX(xu);

  auto xdot = autoDiffToGradientMatrix(dynamics(0*xu_taylor(1),xu_taylor.head(num_x),xu_taylor.tail(num_u)));
  auto A = xdot.leftCols(continuous_state_frame->getDim());
  auto B = xdot.rightCols(input_frame->getDim());

  Eigen::MatrixXd K(num_u,num_x), S(num_x,num_x);
  lqr(A, B, Q, R, K, S);

  // todo: return the linear system with the affine transform.  But for now, just give the affine controller:
  // u = u0 - K(x-x0)
  MatrixXd nullmat(0, 0);
  VectorXd nullvec((Eigen::DenseIndex)0);

  DrakeSystemPtr controller =  make_shared<AffineSystem>(name+"LQR",
                                   nullmat,MatrixXd::Zero(0,num_x),nullvec,
                                   nullmat,MatrixXd::Zero(0,num_x),nullvec,
                                   MatrixXd::Zero(num_u,0),-K,u0+K*x0);
  controller->input_frame = output_frame;
  controller->output_frame = input_frame;
  return controller;
}


void DrakeSystem::simulate(double t0, double tf, const VectorXd &x0, const SimulationOptions& options) const {
  ode1(t0,tf,x0,options);
}

typedef chrono::system_clock TimeClock;  // would love to use steady_clock, but it seems to not compile on all platforms (e.g. MSVC 2013 Win64)
typedef chrono::duration<double> TimeDuration;
typedef chrono::time_point<TimeClock,TimeDuration> TimePoint;

inline void handle_realtime_factor(const TimePoint& wall_clock_start_time, double sim_time, double realtime_factor)
{
  if (realtime_factor>0.0) {
    TimePoint wall_time = TimeClock::now();
    TimePoint desired_time = wall_clock_start_time + TimeDuration(sim_time/realtime_factor);
    if (desired_time>wall_time) { // could probably just call sleep_until, but just in case
      this_thread::sleep_until(desired_time);
    } else if (wall_time>desired_time+TimeDuration(1.0/realtime_factor)) {
      throw runtime_error("Simulation is not keeping up with desired real-time factor -- behind by more than 1 (scaled) second at simulation time " + to_string(sim_time));
    }
  }
}


void DrakeSystem::ode1(double t0, double tf, const VectorXd& x0, const SimulationOptions& options) const {
  double t = t0, dt;
  TimePoint start = TimeClock::now();
  VectorXd x = x0;
  VectorXd u = VectorXd::Zero(input_frame->getDim());
  VectorXd y(output_frame->getDim());
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

VectorXd CascadeSystem::getX1(const VectorXd &x) const {
  VectorXd x1(sys1->state_frame->getDim());
  x1 << x.head(sys1->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim(),sys1->discrete_state_frame->getDim());
  return x1;
}

VectorXd CascadeSystem::getX2(const VectorXd &x) const {
  VectorXd x2(sys2->state_frame->getDim());
  x2 << x.segment(sys1->continuous_state_frame->getDim(),sys2->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim()+sys2->continuous_state_frame->getDim(),sys2->discrete_state_frame->getDim());
  return x2;
}

VectorXd CascadeSystem::dynamics(double t, const VectorXd& x, const VectorXd& u) const {
  unsigned int num_xc1 = sys1->continuous_state_frame->getDim(),
               num_xc2 = sys2->continuous_state_frame->getDim();

  const VectorXd& x1 = getX1(x);
  const VectorXd& y1 = sys1->output(t,x1,u);

  VectorXd xdot(continuous_state_frame->getDim());
  if (num_xc1>0) xdot.head(num_xc1) = sys1->dynamics(t,x1,u);
  if (num_xc2>0) xdot.tail(num_xc2) = sys2->dynamics(t,getX2(x),y1);
  return xdot;
}

VectorXd CascadeSystem::update(double t, const VectorXd& x, const VectorXd& u) const  {
  unsigned int num_xd1 = sys1->discrete_state_frame->getDim(),
          num_xd2 = sys2->discrete_state_frame->getDim();

  const VectorXd& x1 = getX1(x);
  const VectorXd& y1 = sys1->output(t,x1,u);

  VectorXd xn(discrete_state_frame->getDim());
  if (num_xd1>0) xn.head(num_xd1) = sys1->update(t,x1,u);
  if (num_xd2>0) xn.tail(num_xd2) = sys2->update(t,getX2(x),y1);
  return xn;
}

VectorXd CascadeSystem::output(double t, const VectorXd& x, const VectorXd& u) const {
  const VectorXd& y1 = sys1->output(t,getX1(x),u);
  return sys2->output(t,getX2(x),y1);
}



FeedbackSystem::FeedbackSystem(const DrakeSystemPtr& _sys1, const DrakeSystemPtr& _sys2)
        : DrakeSystem("FeedbackSystem"), sys1(_sys1), sys2(_sys2) {
  if (sys1->output_frame != sys2->input_frame)
    throw runtime_error("Feedback combination failed: output frame of "+sys1->name+" must match the input frame of "+sys2->name);
  if (sys2->output_frame != sys1->input_frame)
    throw runtime_error("Feedback combination failed: output frame of "+sys1->name+" must match the input frame of "+sys2->name);
  if (sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough())
    throw runtime_error("Drake:FeedbackSystem:AlgebraicLoop.  algebraic loop");

  input_frame = sys1->input_frame;
  output_frame = sys1->output_frame;
  continuous_state_frame = make_shared<MultiCoordinateFrame>("FeedbackSystemContState",initializer_list<CoordinateFramePtr>({sys1->continuous_state_frame, sys2->continuous_state_frame}));
  discrete_state_frame = make_shared<MultiCoordinateFrame>("FeedbackSystemDiscState",initializer_list<CoordinateFramePtr>({sys1->discrete_state_frame, sys2->discrete_state_frame}));
  state_frame = make_shared<MultiCoordinateFrame>("FeedbackSystemState",initializer_list<CoordinateFramePtr>({continuous_state_frame,discrete_state_frame}));
}

VectorXd FeedbackSystem::getX1(const VectorXd &x) const {
  VectorXd x1(sys1->state_frame->getDim());
  x1 << x.head(sys1->continuous_state_frame->getDim()),
          x.segment(sys1->state_frame->getDim(),sys1->discrete_state_frame->getDim());
  return x1;
}

VectorXd FeedbackSystem::getX2(const VectorXd &x) const {
  VectorXd x2(sys2->state_frame->getDim());
  x2 << x.segment(sys1->continuous_state_frame->getDim(),sys2->continuous_state_frame->getDim()),
          x.segment(sys1->state_frame->getDim()+sys2->continuous_state_frame->getDim(),sys2->discrete_state_frame->getDim());
  return x2;
}

void FeedbackSystem::subsystemOutputs(double t, const Eigen::VectorXd &x1, const Eigen::VectorXd& x2, const Eigen::VectorXd &u,
                                      Eigen::VectorXd &y1, Eigen::VectorXd &y2) const {
  if (!sys1->isDirectFeedthrough()) {
    y1 = sys1->output(t,x1,u);  // output does not depend on u (so it's ok that we're not passing u+y2)
    y2 = sys2->output(t,x2,y1);
  } else {
    y2 = sys1->output(t,x2,y1); // y1 might be uninitialized junk, but has to be ok
    y1 = sys2->output(t,x1,y2+u);
  }
}

VectorXd FeedbackSystem::dynamics(double t, const VectorXd& x, const VectorXd& u) const {
  unsigned int num_xc1 = sys1->continuous_state_frame->getDim(),
          num_xc2 = sys2->continuous_state_frame->getDim();

  const VectorXd& x1 = getX1(x), x2 = getX2(x);
  VectorXd y1(sys1->output_frame->getDim()), y2(sys2->output_frame->getDim());
  subsystemOutputs(t,x1,x2,u,y1,y2);

  VectorXd xdot(continuous_state_frame->getDim());
  if (num_xc1>0) xdot.head(num_xc1) = sys1->dynamics(t,x1,y2+u);
  if (num_xc2>0) xdot.tail(num_xc2) = sys2->dynamics(t,x2,y1);
  return xdot;
}

VectorXd FeedbackSystem::update(double t, const VectorXd& x, const VectorXd& u) const  {
  unsigned int num_xd1 = sys1->discrete_state_frame->getDim(),
          num_xd2 = sys2->discrete_state_frame->getDim();

  const VectorXd& x1 = getX1(x), x2 = getX2(x);
  VectorXd y1(sys1->output_frame->getDim()), y2(sys2->output_frame->getDim());
  subsystemOutputs(t,x1,x2,u,y1,y2);

  VectorXd xn(discrete_state_frame->getDim());
  if (num_xd1>0) xn.head(num_xd1) = sys1->update(t,x1,y2+u);
  if (num_xd2>0) xn.tail(num_xd2) = sys2->update(t,x2,y1);
  return xn;
}

VectorXd FeedbackSystem::output(double t, const VectorXd& x, const VectorXd& u) const {
  VectorXd y1(sys1->output_frame->getDim());
  if (!sys1->isDirectFeedthrough()) {
    y1 = sys1->output(t,getX1(x),u);  // output does not depend on u (so it's ok that we're not passing u+y2)
  } else {
    const VectorXd& y2 = sys2->output(t,getX2(x),y1); // y1 might be uninitialized junk, but has to be ok
    y1 = sys1->output(t,getX2(x),y2+u);
  }
  return y1;
}