#ifndef __DrakeSystem_H__
#define __DrakeSystem_H__

#include <string>
#include <memory>
#include <stdexcept>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include "CoordinateFrame.h"


/// A DrakeSystem is a dynamical system that is compatible with most of our tools for design and analysis.
/// It must have:
///   - a real-vector-valued input, state, and output
///   - deterministic dynamics and outputs given the input and state
///   - no more than one discrete time-step (in addition to continuous dynamics)
///  The input, state, and output coordinate systems are all described by CoordinateSystem objects
///  In addition, it MAY have
///   - time-varying dynamics and outputs
///   - input limits (c++ support coming soon)
///   - algebraic constraints (c++ support coming soon)
///   - zero-crossings (c++ support coming soon) to inform the tools of discontinuities in the dynamics

namespace Drake {
  // todo: move this to a util directory?

  // note: tried using template default values (e.g. Eigen::Dynamic), but they didn't seem to work on my mac clang
  template <int order> using TaylorVar = Eigen::AutoDiffScalar< Eigen::Matrix<double,order,1> >;
  template <int order, int cols> using TaylorVec = Eigen::Matrix< TaylorVar<order>, cols, 1>;
  template <int order, int rows, int cols> using TaylorMat = Eigen::Matrix< TaylorVar<order>, rows, cols>;

  typedef TaylorVar<Eigen::Dynamic> TaylorVarX;
  typedef TaylorVec<Eigen::Dynamic,Eigen::Dynamic> TaylorVecX;
  typedef TaylorMat<Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic> TaylorMatX;
}

class DLLEXPORT DrakeSystem : public std::enable_shared_from_this<DrakeSystem> {
public:

  DrakeSystem(const std::string& name,
              const CoordinateFramePtr& continuous_state_frame,
              const CoordinateFramePtr& discrete_state_frame,
              const CoordinateFramePtr& input_frame,
              const CoordinateFramePtr& output_frame);

  DrakeSystem(const std::string& name,
              unsigned int num_continuous_states = 0,
              unsigned int num_discrete_states = 0,
              unsigned int num_inputs = 0,
              unsigned int num_outputs = 0);

  virtual ~DrakeSystem() {};

  const CoordinateFrame& getInputFrame() { return *input_frame.get(); }
  const CoordinateFrame& getStateFrame() { return *state_frame.get(); }
  const CoordinateFrame& getOutputFrame() { return *output_frame.get(); }

#define DRAKESYSTEM_DYNAMICS_METHOD(ScalarType,VectorType) \
  virtual VectorType dynamics(ScalarType t, const VectorType& x, const VectorType& u) const { \
    throw std::runtime_error("Drake:DrakeSystem:dynamics: your system needs to overload the dynamics method with ScalarType t and VectorType x and u"); \
  }
// end of #define

  DRAKESYSTEM_DYNAMICS_METHOD(double, Eigen::VectorXd)
  DRAKESYSTEM_DYNAMICS_METHOD(Drake::TaylorVarX, Drake::TaylorVecX)
#undef DRAKESYTEM_DYNAMICS_METHOD


  virtual Eigen::VectorXd update(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
    throw std::runtime_error("(Drake:DrakeSystem:update) systems with discrete dynamics must overload the update method");
  }

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
    if (output_frame->getDim()>0)
      throw std::runtime_error("(Drake:DrakeSystem:dynamics) systems with outputs must overload the output method");
    else
      return Eigen::VectorXd::Zero(0);
  }

  virtual Eigen::VectorXd getRandomState();
  virtual Eigen::VectorXd getInitialState();

  // simulation options
  struct SimulationOptions {
    double realtime_factor;  // 1 means try to run at realtime speed, < 0 is run as fast as possible
    double initial_step_size;

    SimulationOptions() :
            realtime_factor(-1.0),
            initial_step_size(0.01)
    {};
  };
  SimulationOptions default_simulation_options;

  virtual void simulate(double t0, double tf, const Eigen::VectorXd& x0, const SimulationOptions& options) const;
  virtual void simulate(double t0, double tf, const Eigen::VectorXd& x0) const {
    simulate(t0,tf,x0,default_simulation_options);
  }

  std::string name;

  std::shared_ptr<const CoordinateFrame> input_frame;
  std::shared_ptr<const CoordinateFrame> output_frame;
  std::shared_ptr<const CoordinateFrame> continuous_state_frame, discrete_state_frame, state_frame; // should either protect these or avoid storing them all

  virtual bool isTimeInvariant() const { return false; }    // are the dynamics,update, and output methods independent of t?  set to true if possible!
  virtual bool isDirectFeedthrough() const { return true; } // does the output method depend (directly) on the input u?  set to false if possible!

protected:

  virtual void ode1(double t0, double tf, const Eigen::VectorXd& x0, const SimulationOptions& options) const;

//  virtual void ode45(double t0, double tf, const VectorXs& x0, double initial_step_size, double relative_error_tolerance, double absolute_error_tolerance);
// c.f. https://www.google.com/search?q=Runge-Kutta-Fehlberg and edit ode45.m in matlab.
};

class DLLEXPORT CascadeSystem : public DrakeSystem {
public:
  CascadeSystem(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2);
  virtual ~CascadeSystem(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
  virtual Eigen::VectorXd update(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;

  virtual bool isTimeInvariant() const override { return sys1->isTimeInvariant() && sys2->isTimeInvariant(); }
  virtual bool isDirectFeedthrough() const override { return sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough(); }

private:
  Eigen::VectorXd getX1(const Eigen::VectorXd& x) const;
  Eigen::VectorXd getX2(const Eigen::VectorXd& x) const;

  DrakeSystemPtr sys1, sys2;
};

static DrakeSystemPtr cascade(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2) {
  return std::make_shared<CascadeSystem>(sys1,sys2);
}

#endif // #define __DrakeSystem_H_
