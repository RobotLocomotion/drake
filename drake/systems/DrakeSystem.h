#ifndef __DrakeSystem_H__
#define __DrakeSystem_H__

#include <string>
#include <memory>
#include <stdexcept>
#include <Eigen/Dense>
#include "CoordinateFrame.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakeSystem_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif


/// A dynamical system authored in Drake registers it's dynamics as well as information
/// about it's coordinate frames.

class DLLEXPORT DrakeSystem : public std::enable_shared_from_this<DrakeSystem> {
public:

  typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorXs;

  DrakeSystem(const std::string& name,
              const std::shared_ptr<CoordinateFrame>& continuous_state_frame,  // note: the frames are not const
              const std::shared_ptr<CoordinateFrame>& discrete_state_frame,
              const std::shared_ptr<CoordinateFrame>& input_frame,
              const std::shared_ptr<CoordinateFrame>& output_frame);

  DrakeSystem(const std::string& name,
              unsigned int num_continuous_states = 0,
              unsigned int num_discrete_states = 0,
              unsigned int num_inputs = 0,
              unsigned int num_outputs = 0);

  virtual ~DrakeSystem() {};

  const CoordinateFrame& getInputFrame() { return *input_frame.get(); }
  const CoordinateFrame& getStateFrame() { return *state_frame.get(); }
  const CoordinateFrame& getOutputFrame() { return *output_frame.get(); }

  // todo: templates for these

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u) {
    throw std::runtime_error("(Drake:DrakeSystem:dynamics) systems with continuous dynamics must overload the dynamics method");
  }

  virtual VectorXs update(double t, const VectorXs& x, const VectorXs& u) {
    throw std::runtime_error("(Drake:DrakeSystem:update) systems with discrete dynamics must overload the update method");
  }

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) {
    if (output_frame->getDim()>0)
      throw std::runtime_error("(Drake:DrakeSystem:dynamics) systems with outputs must overload the output method");
    else
      return VectorXs::Zero(0);
  }

  virtual VectorXs getRandomState();
  virtual VectorXs getInitialState();

  // simulation options
  typedef struct _SimulationOptions {
    double realtime_factor;  // 1 means try to run at realtime speed, < 0 is run as fast as possible
    double initial_step_size;

    _SimulationOptions() :
            realtime_factor(-1.0),
            initial_step_size(0.01)
    {};
  } SimulationOptions;
  SimulationOptions default_simulation_options;

  virtual void simulate(double t0, double tf, const VectorXs& x0, const SimulationOptions& options);
  virtual void simulate(double t0, double tf, const VectorXs& x0) {
    simulate(t0,tf,x0,default_simulation_options);
  }

  std::string name;

  std::shared_ptr<CoordinateFrame> input_frame;
  std::shared_ptr<CoordinateFrame> output_frame;
  std::shared_ptr<CoordinateFrame> continuous_state_frame, discrete_state_frame, state_frame; // should either protect these or avoid storing them all

  virtual bool isTimeInvariant() { return false; }    // are the dynamics,update, and output methods independent of t?  set to true if possible!
  virtual bool isDirectFeedthrough() { return true; } // does the output method depend (directly) on the input u?  set to false if possible!

protected:

  virtual void ode1(double t0, double tf, const VectorXs& x0, const SimulationOptions& options);

//  virtual void ode45(double t0, double tf, const VectorXs& x0, double initial_step_size, double relative_error_tolerance, double absolute_error_tolerance);
// c.f. https://www.google.com/search?q=Runge-Kutta-Fehlberg and edit ode45.m in matlab.
};

typedef std::shared_ptr<DrakeSystem> DrakeSystemPtr;

class CascadeSystem : public DrakeSystem {
public:
  CascadeSystem(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2);
  virtual ~CascadeSystem(void) {};

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u);
  virtual VectorXs update(double t, const VectorXs& x, const VectorXs& u);
  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u);

  virtual bool isTimeInvariant() { return sys1->isTimeInvariant() && sys2->isTimeInvariant(); }
  virtual bool isDirectFeedthrough() { return sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough(); }

private:
  VectorXs getX1(const VectorXs& x);
  VectorXs getX2(const VectorXs& x);

  DrakeSystemPtr sys1, sys2;
};

static DrakeSystemPtr cascade(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2) {
  return std::make_shared<CascadeSystem>(sys1,sys2);
}

#endif // #define __DrakeSystem_H_