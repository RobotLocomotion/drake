#ifndef __DrakeSystem_H__
#define __DrakeSystem_H__

#include <string>
#include <memory>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "DrakeCore.h"
#include "DrakeSystemSpecializations.h"
#include "CoordinateFrame.h"
#include "drakeGradientUtil.h"


/// A Drake::System is a dynamical system that is compatible with most of our tools for design and analysis.
/// It must have:
///   - a real-vector-valued input, state, and output
///   - deterministic dynamics and outputs given the input and state
///   - continuous dynamics [ coming soon: no more than one discrete time-step (in addition to continuous dynamics) ]
///  The input, state, and output coordinate systems are all described by CoordinateSystem objects
///  In addition, it MAY have
///   - time-varying dynamics and outputs
///   - input limits (c++ support coming soon)
///   - algebraic constraints (c++ support coming soon)
///   - zero-crossings (c++ support coming soon) to inform the tools of discontinuities in the dynamics


namespace Drake {

  template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool _isTimeVarying = true, bool _isDirectFeedthrough = true >
  class System {
  public:
    typedef Derived Type;
    template <typename ScalarType> using StateVectorType = StateVector<ScalarType>;
    template <typename ScalarType> using InputVectorType = InputVector<ScalarType>;
    template <typename ScalarType> using OutputVectorType = OutputVector<ScalarType>;
    const static bool isTimeVarying = _isTimeVarying;
    const static bool isDirectFeedthrough = _isDirectFeedthrough;
    const static unsigned int num_states = VectorTraits<StateVector<double> >::RowsAtCompileTime;
    const static unsigned int num_inputs = VectorTraits<InputVector<double> >::RowsAtCompileTime;
    const static unsigned int num_outputs = VectorTraits<OutputVector<double> >::RowsAtCompileTime;

    /// dynamics
    /// @param t time in seconds
    /// @param x state vector
    /// @param u input vector
    ///
    /// derived classes with non-empty state vectors (RowsAtCompileTime!=0) must implement
    ///   template <typename ScalarType>
    ///   StateVector<ScalarType> dynamicsImplementation(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const;
    /// but where the t argument must be left out if the system is time-invariant
    ///           the u argument must be left out if the system has no inputs (RowsAtCompileTime==0)

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return DynamicsDispatch< isTimeVarying, (num_states != 0), (num_inputs != 0), ScalarType, Derived, StateVector, InputVector>::eval(static_cast<const Derived*>(this),t,x,u);
    };
/*
 * // todo: allow people to access the dynamics with minimal calls, too
    StateVector<double> dynamics(const StateVector<double>& x, const InputVector<double>& u) const {
      static_assert(!isTimeVarying,"You must set the isTimeVarying template parameter to false to use this method");
      return static_cast<const Derived*>(this)->dynamicsImplementation(x,u);
    };
*/
    // todo: add update method (and in general support for DT or mixed CT/DT systems)

    /// output
    /// @param t time in seconds
    /// @param x state vector
    /// @param u input vector
    ///
    /// derived classes with non-empty output vectors (RowsAtCompileTime!=0) must implement
    ///   template <typename ScalarType>
    ///   OutputVector<ScalarType> outputImplementation(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const;
    /// but where the t argument must be left out if the system is time-invariant
    ///           the x argument must be left out if the system has zero state (RowsAtCompileTime==0)
    ///           the u argument must be left out if the system has no inputs or if the system is labeled as not direct feedthrough

//  todo: implemented the templated forms here. (need to help compiler figure out the instantiation for casting to Eigen::Matrix<double>)
    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return OutputDispatch< (num_outputs!=0), isTimeVarying, (num_states != 0), ((num_inputs != 0) && isDirectFeedthrough), ScalarType, Derived, StateVector, InputVector, OutputVector>::eval(static_cast<const Derived*>(this),t,x,u);
    };
/*
    OutputVector<double> output(const StateVector<double>& x, const InputVector<double>& u) const {
      static_assert(!isTimeVarying,"You must set the isTimeVarying template parameter to false to use this method");
      return static_cast<const Derived*>(this)->outputImplementation(x,u);
    };
    OutputVector<double> output(const double& t, const StateVector<double>& x) const {
      static_assert(!isDirectFeedthrough,"You must set the isDirectFeedthrough template parameter to false to use this method");
      return static_cast<const Derived*>(this)->outputImplementation(t,x);
    };
    OutputVector<double> output(const StateVector<double>& x) const {
      static_assert(!isTimeVarying,"You must set the isTimeVarying template parameter to false to use this method");
      static_assert(!isDirectFeedthrough,"You must set the isDirectFeedthrough template parameter to false to use this method");
      return static_cast<const Derived*>(this)->outputImplementation(x);
    };
*/

    // todo: add sparsity information about dynamics, update, and output methods

  };

// This is the class design that I want.  But it seems that recursive templates in the inheritance line don't work (more discussion in the git history)
//
//  template <class System1, class System2>
//  class FeedbackSystem : public System<FeedbackSystem<System1,System2>,
//          CombinedVectorBuilder<System1::StateVectorType , System2::StateVectorType>::VecType, System1::InputVectorType, System1::OutputVectorType, System1::isTimeVarying||System2::isTimeVarying, System1::isDirectFeedthrough> {
//
// So I'm getting around it with this helper macro (which makes the call outside of the template class, so it's not recursive)
#define FeedbackSystemType(System1,System2) Drake::FeedbackSystem<System1,System2,System1::StateVectorType,System2::StateVectorType,Drake::CombinedVectorBuilder<System1::StateVectorType,System2::StateVectorType>::VecType,System1::InputVectorType,System1::OutputVectorType,System1::isTimeVarying||System2::isTimeVarying,System1::isDirectFeedthrough>

  template <typename System1, typename System2,
          template <typename> class StateVector1, template <typename> class StateVector2, template<typename> class StateVector,
          template<typename> class InputVector, template<typename> class OutputVector,
          bool isTimeVarying, bool isDirectFeedthrough>
  class FeedbackSystem : public System<FeedbackSystem<System1,System2,StateVector1,StateVector2,StateVector,InputVector,OutputVector,isTimeVarying,isDirectFeedthrough> ,
          StateVector, InputVector, OutputVector, isTimeVarying, isDirectFeedthrough> {
  public:
    template <typename ScalarType> using EigenInput = Eigen::Matrix<ScalarType,System1::num_inputs,1>;
    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;

    static_assert(!System1::isDirectFeedthrough || !System2::isDirectFeedthrough,"Algebraic Loop: cannot feedback combine two systems that are both direct feedthrough");
//    static_assert(System1::OutputVectorType == System2::InputVectorType,"System 2 input vector must match System 1 output vector");
//    static_assert(System2::OutputVectorType == System1::InputVectorType,"System 1 input vector must match System 2 output vector");
    // todo: assert that StateVector is indeed a CombinedVector<...,StateVector1,StateVector2>

    FeedbackSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) {
    };

    template <typename ScalarType>
    StateVector<ScalarType> dynamicsImplementation(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      OutputVector<ScalarType> y1;
      InputVector<ScalarType> y2;
      subsystemOutputs(t,x.first(),x.second(),u,y1,y2);

      StateVector<ScalarType> xdot(sys1->dynamics(t,x.first(),static_cast<InputVector<ScalarType> >( static_cast<EigenInput<ScalarType> >(y2)+static_cast<EigenInput<ScalarType> >(u))),
                                   sys2->dynamics(t,x.second(),y1));
      return xdot;
    }
    template <typename ScalarType>
    StateVector<ScalarType> dynamicsImplementation(const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      ScalarType t;
      return dynamicsImplementation(t,x,u);
    }

    template <typename ScalarType>
    OutputVector<ScalarType> outputImplementation(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      OutputVector<ScalarType> y1;
      if (!System1::isDirectFeedthrough) {
        y1 = sys1->output(t,x.first(),u);   // then don't need u+y2 here, u will be ignored
      } else {
        InputVector<ScalarType> y2;
        y2 = sys2->output(t,x.second(),y1); // y1 might be uninitialized junk, but has to be ok
        y1 = sys1->output(t,x.first(),static_cast<InputVector<ScalarType> >( static_cast<EigenInput<ScalarType> >(y2)+static_cast<EigenInput<ScalarType> >(u)));
      }
      return y1;
    }
    template <typename ScalarType>
    OutputVector<ScalarType> outputImplementation(const StateVector<ScalarType>& x) const {
      ScalarType t;
      InputVector<ScalarType> u;
      return outputImplementation(t,x,u);
    }

    template <typename ScalarType>
    void subsystemOutputs(const ScalarType& t, const StateVector1<ScalarType>& x1, const StateVector2<ScalarType>& x2, const InputVector<ScalarType> &u,
                                          OutputVector<ScalarType> &y1, InputVector<ScalarType> &y2) const {
      if (!System1::isDirectFeedthrough) {
        y1 = sys1->output(t,x1,u);  // output does not depend on u (so it's ok that we're not passing u+y2)
        y2 = sys2->output(t,x2,y1);
      } else {
        y2 = sys2->output(t,x2,y1); // y1 might be uninitialized junk, but has to be ok
        y1 = sys1->output(t,x1,static_cast<InputVector<ScalarType> >( static_cast<EigenInput<ScalarType> >(y2)+static_cast<EigenInput<ScalarType> >(u)));
      }
    }

    System1Ptr sys1;
    System2Ptr sys2;
  };


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

  template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool isTimeVarying = true, bool isDirectFeedthrough = true >
  void simulate(const System<Derived,StateVector,InputVector,OutputVector,isTimeVarying,isDirectFeedthrough>& sys, double t0, double tf, const Eigen::VectorXd& x0, const SimulationOptions& options) {
    double t = t0, dt;
    std::cout << "x0 = " << x0.transpose() << std::endl;
    Eigen::Matrix<double,Drake::VectorTraits<StateVector<double> >::RowsAtCompileTime,1> x = x0;
    Eigen::Matrix<double,Drake::VectorTraits<StateVector<double> >::RowsAtCompileTime,1> xdot;
    Eigen::Matrix<double,Drake::VectorTraits<InputVector<double> >::RowsAtCompileTime,1> u(InputVector<double>::size()); u.setConstant(0);
    Eigen::Matrix<double,Drake::VectorTraits<OutputVector<double> >::RowsAtCompileTime,1> y;
    while (t<tf) {
      std::cout << "t=" << t << ", x = " << x.transpose() << std::endl;
      dt = (std::min)(options.initial_step_size,tf-t);
      y = sys.template output<double>(t,x,u);
      xdot = sys.template dynamics<double>(t,x,u);
      x += dt * xdot;
      t += dt;
    }
  }

  template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool isTimeVarying = true, bool isDirectFeedthrough = true >
  void simulate(const System<Derived,StateVector,InputVector,OutputVector,isTimeVarying,isDirectFeedthrough>& sys, double t0, double tf, const Eigen::VectorXd& x0)  {
    simulate(sys,t0,tf,x0,default_simulation_options);
  }



} // end namespace Drake



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

#define DRAKESYSTEM_DYNAMICS_METHOD(VectorType) \
  virtual VectorType dynamics(VectorType::Scalar t, const VectorType& x, const VectorType& u) const { \
    throw std::runtime_error("Drake:DrakeSystem:dynamics: your system needs to overload the dynamics method with ScalarType t and VectorType x and u"); \
  }
// end of #define

  DRAKESYSTEM_DYNAMICS_METHOD(Eigen::VectorXd)
  DRAKESYSTEM_DYNAMICS_METHOD(Drake::TaylorVecX)
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

  DrakeSystemPtr timeInvariantLQR(const Eigen::VectorXd& x0, const Eigen::VectorXd& u0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

  std::string name;

  std::shared_ptr<const CoordinateFrame> continuous_state_frame, discrete_state_frame, state_frame; // should either protect these or avoid storing them all
  std::shared_ptr<const CoordinateFrame> input_frame;
  std::shared_ptr<const CoordinateFrame> output_frame;

  virtual bool isTimeInvariant() const { return false; }    // are the dynamics,update, and output methods independent of t?  set to true if possible!
  virtual bool isDirectFeedthrough() const { return true; } // does the output method depend (directly) on the input u?  set to false if possible!

private:

  void ode1(double t0, double tf, const Eigen::VectorXd& x0, const SimulationOptions& options) const;
//  void ode45(double t0, double tf, const VectorXs& x0, double initial_step_size, double relative_error_tolerance, double absolute_error_tolerance);
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


class DLLEXPORT FeedbackSystem : public DrakeSystem {
public:
  FeedbackSystem(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2);
  virtual ~FeedbackSystem(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
  virtual Eigen::VectorXd update(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;

  virtual bool isTimeInvariant() const override { return sys1->isTimeInvariant() && sys2->isTimeInvariant(); }
  virtual bool isDirectFeedthrough() const override { return sys1->isDirectFeedthrough(); } // && sys2->isDirectFeedthrough(); }

private:
  Eigen::VectorXd getX1(const Eigen::VectorXd& x) const;
  Eigen::VectorXd getX2(const Eigen::VectorXd& x) const;
  void subsystemOutputs(double t, const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, const Eigen::VectorXd& u, Eigen::VectorXd& y1, Eigen::VectorXd& y2) const;

  DrakeSystemPtr sys1, sys2;
};

static DrakeSystemPtr feedback(const DrakeSystemPtr& sys1, const DrakeSystemPtr& sys2) {
  return std::make_shared<FeedbackSystem>(sys1,sys2);
}

#endif // #define __DrakeSystem_H_
