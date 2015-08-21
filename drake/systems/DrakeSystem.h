#ifndef __DrakeSystem_H__
#define __DrakeSystem_H__

#include <string>
#include <memory>
#include <exception>
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

class DLLEXPORT DrakeSystem {
public:

  typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorXs;

  DrakeSystem(const std::string& name,
              std::shared_ptr<CoordinateFrame> continuous_state_frame,  // should these be const? (right now, I think that we should be allowed to modify them)
              std::shared_ptr<CoordinateFrame> discrete_state_frame,
              std::shared_ptr<CoordinateFrame> input_frame,
              std::shared_ptr<CoordinateFrame> output_frame);

  DrakeSystem(const std::string& name,
              unsigned int num_continuous_states,
              unsigned int num_discrete_states,
              unsigned int num_inputs,
              unsigned int num_outputs);

  DrakeSystem(const std::string& name) : DrakeSystem(name,0,0,0,0) {}; // build the empty system

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

  virtual VectorXs getRandomState(void);
  virtual VectorXs getInitialState(void);
  virtual void simulate(double t0, double tf, const VectorXs& x0);
  virtual void runLCM(double t0, double tf, const VectorXs& x0);

  std::string name;

  std::shared_ptr<CoordinateFrame> input_frame;
  std::shared_ptr<CoordinateFrame> output_frame;
  std::shared_ptr<CoordinateFrame> continuous_state_frame, discrete_state_frame, state_frame; // should either protect these or avoid storing them all

protected:

  virtual void ode1(double t0, double tf, const VectorXs& x0, double step_size);

//  virtual void ode45(double t0, double tf, const VectorXs& x0, double initial_step_size, double relative_error_tolerance, double absolute_error_tolerance);
// c.f. https://www.google.com/search?q=Runge-Kutta-Fehlberg and edit ode45.m in matlab.

  /*
  bool is_direct_feedthrough;  // does the output method depend on the input u?  set false if you can!
  bool is_time_invariant; // are all of the dynamics and output methods independent of time? set to true if you can!
   */
};

class CascadeSystem : public DrakeSystem {
public:
  CascadeSystem(std::shared_ptr<DrakeSystem> sys1, std::shared_ptr<DrakeSystem> sys2);

  virtual VectorXs dynamics(double t, const VectorXs& x, const VectorXs& u);
  virtual VectorXs update(double t, const VectorXs& x, const VectorXs& u);
  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u);

private:
  VectorXs getX1(const VectorXs& x);
  VectorXs getX2(const VectorXs& x);

  std::shared_ptr<DrakeSystem> sys1, sys2;
};


#endif // #define __DrakeSystem_H_