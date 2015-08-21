
#include <algorithm>
#include "DrakeSystem.h"

using namespace std;

DrakeSystem::DrakeSystem(const std::string &name, std::shared_ptr<CoordinateFrame> _continuous_state_frame,
                         std::shared_ptr<CoordinateFrame> _discrete_state_frame,
                         std::shared_ptr<CoordinateFrame> _input_frame,
                         std::shared_ptr<CoordinateFrame> _output_frame)
  : continuous_state_frame(_continuous_state_frame), discrete_state_frame(_discrete_state_frame),
    input_frame(_input_frame), output_frame(_output_frame) {
  state_frame = MultiCoordinateFrame::constructFrame(name+"State",{continuous_state_frame,discrete_state_frame});
}

DrakeSystem::DrakeSystem(const std::string &name, unsigned int num_continuous_states, unsigned int num_discrete_states,
                         unsigned int num_inputs, unsigned int num_outputs)
        : input_frame(nullptr), continuous_state_frame(nullptr), discrete_state_frame(nullptr), output_frame(nullptr) {
  if (num_inputs>0) input_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"Input",num_inputs,"u"));
  if (num_continuous_states>0) continuous_state_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"ContinuousState",num_continuous_states,"xc"));
  if (num_discrete_states>0) discrete_state_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"DiscreteState",num_discrete_states,"xd"));
  if (num_outputs>0) output_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"Output",num_outputs,"y"));
  state_frame = shared_ptr<CoordinateFrame>(new MultiCoordinateFrame(name+"state",{continuous_state_frame,discrete_state_frame}));
}

DrakeSystem::VectorXs DrakeSystem::getRandomState(void) {
  return DrakeSystem::VectorXs::Random(state_frame->getDim());
}

DrakeSystem::VectorXs DrakeSystem::getInitialState(void) {
  return getRandomState();
}

void DrakeSystem::simulate(double t0, double tf, const DrakeSystem::VectorXs &x0) {
  ode1(t0,tf,x0,.1);
}

void DrakeSystem::runLCM(double t0, double tf, const VectorXs &x0) {
//  input_frame->setupLCMInputs(); // not implemented yet
//  output_frame->setupLCMOutputs();
  simulate(t0,tf,x0);
}

void DrakeSystem::ode1(double t0, double tf, const DrakeSystem::VectorXs& x0, double step_size) {
  double t = t0, dt;
  DrakeSystem::VectorXs x = x0;
  DrakeSystem::VectorXs u = DrakeSystem::VectorXs::Zero(input_frame->getDim());
  DrakeSystem::VectorXs y(output_frame->getDim());
  while (t<tf) {
    dt = std::min(step_size,tf-t);
    y = output(t,x,u);
    x += dt * dynamics(t, x, u);
    t += dt;
  }
}



CascadeSystem::CascadeSystem(std::shared_ptr<DrakeSystem> _sys1, std::shared_ptr<DrakeSystem> _sys2)
  : DrakeSystem("CascadeSystem"), sys1(_sys1), sys2(_sys2) {
  if (sys1->output_frame != sys2->input_frame)
    throw runtime_error("Cascade combination failed: output frame of "+sys1->name+" must match the input frame of "+sys2->name);
  input_frame = sys1->input_frame;
  output_frame = sys2->output_frame;
  continuous_state_frame = MultiCoordinateFrame::constructFrame("CascadeSystemContState",{sys1->continuous_state_frame, sys2->continuous_state_frame});
  discrete_state_frame = MultiCoordinateFrame::constructFrame("CascadeSystemDiscState",{sys1->discrete_state_frame, sys2->discrete_state_frame});
  state_frame = MultiCoordinateFrame::constructFrame("CascadeSystemState",{continuous_state_frame,discrete_state_frame});
}

DrakeSystem::VectorXs CascadeSystem::getX1(const VectorXs &x) {
  DrakeSystem::VectorXs x1(sys1->state_frame->getDim());
  x1 << x.head(sys1->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim(),sys1->discrete_state_frame->getDim());
  return x1;
}

DrakeSystem::VectorXs CascadeSystem::getX2(const VectorXs &x) {
  DrakeSystem::VectorXs x2(sys2->state_frame->getDim());
  x2 << x.segment(sys1->continuous_state_frame->getDim(),sys2->continuous_state_frame->getDim()),
        x.segment(sys1->state_frame->getDim()+sys2->continuous_state_frame->getDim(),sys2->discrete_state_frame->getDim());
  return x2;
}

DrakeSystem::VectorXs CascadeSystem::dynamics(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) {
  unsigned int num_xc1 = sys1->continuous_state_frame->getDim(),
               num_xc2 = sys2->continuous_state_frame->getDim();

  DrakeSystem::VectorXs x1 = getX1(x);
  DrakeSystem::VectorXs y1 = sys1->output(t,x1,u);
  DrakeSystem::VectorXs xdot(continuous_state_frame->getDim());
  if (num_xc1>0) xdot.head(num_xc1) = sys1->dynamics(t,x1,u);
  if (num_xc2>0) xdot.tail(num_xc2) = sys2->dynamics(t,getX2(x),y1);
  return xdot;
}

DrakeSystem::VectorXs CascadeSystem::update(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) {
  unsigned int num_xd1 = sys1->discrete_state_frame->getDim(),
          num_xd2 = sys2->discrete_state_frame->getDim();

  DrakeSystem::VectorXs x1 = getX1(x);
  DrakeSystem::VectorXs y1 = sys1->output(t,x1,u);
  DrakeSystem::VectorXs xn(discrete_state_frame->getDim());
  if (num_xd1>0) xn.head(num_xd1) = sys1->update(t,x1,u);
  if (num_xd2>0) xn.tail(num_xd2) = sys2->update(t,getX2(x),y1);
  return xn;
}

DrakeSystem::VectorXs CascadeSystem::output(double t, const DrakeSystem::VectorXs& x, const DrakeSystem::VectorXs& u) {
  DrakeSystem::VectorXs y1 = sys1->output(t,getX1(x),u);
  return sys2->output(t,getX2(x),y1);
}
