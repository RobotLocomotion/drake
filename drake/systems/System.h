#ifndef DRAKE_SYSTEM_H
#define DRAKE_SYSTEM_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "Core.h"
#include "SystemSpecializations.h"


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
    const static int num_states = StateVector<double>::RowsAtCompileTime;
    const static int num_inputs = InputVector<double>::RowsAtCompileTime;
    const static int num_outputs = OutputVector<double>::RowsAtCompileTime;

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
   // todo: allow people to access the dynamics with minimal calls, too
    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      static_assert(!isTimeVarying,"You must set the isTimeVarying template parameter to false to use this method");
      ScalarType t;
      return dynamics(t,x,u);
    };

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
    /// Note that (by convention) the output method is called even if there is no output for the system. (e.g. for Visualizers)

//  todo: implemented the templated forms here. (need to help compiler figure out the instantiation for casting to Eigen::Matrix<double>)
    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return OutputDispatch< isTimeVarying, (num_states != 0), ((num_inputs != 0) && isDirectFeedthrough), ScalarType, Derived, StateVector, InputVector, OutputVector>::eval(static_cast<const Derived*>(this),t,x,u);
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

    template <typename ScalarType>
    StateVector<ScalarType> getRandomState() {
      // todo: handle the dynamic size case
      StateVector<ScalarType> x(Eigen::Matrix<ScalarType,num_states,1>::Random());
      return x;
    }
  };

  template <class System1, class System2>
  class FeedbackSystem : public System<FeedbackSystem<System1,System2>,
          CombinedVectorBuilder<System1::template StateVectorType , System2::template StateVectorType>::template VecType,
          System1::template InputVectorType,
          System1::template OutputVectorType,
          System1::isTimeVarying||System2::isTimeVarying,
          System1::isDirectFeedthrough> {
  public:
    template <typename ScalarType> using StateVector = CombinedVector<ScalarType, System1::template StateVectorType , System2::template StateVectorType>;
    template <typename ScalarType> using StateVector1 = typename System1::template StateVectorType<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVectorType<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVectorType<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System1::template OutputVectorType<ScalarType>;

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

  private:
    System1Ptr sys1;
    System2Ptr sys2;
  };

  template <typename System1, typename System2>
  std::shared_ptr<FeedbackSystem<System1,System2>> feedback(const std::shared_ptr<System1>& sys1, const std::shared_ptr<System2>& sys2)
  {
    return std::make_shared<FeedbackSystem<System1,System2> >(sys1,sys2);
  };


#define CascadeSystemType(System1,System2) Drake::CascadeSystem<System1,System2,System1::StateVectorType,System2::StateVectorType,,>

  template <class System1, class System2>
  class CascadeSystem : public System<CascadeSystem<System1,System2>,
          CombinedVectorBuilder<System1::template StateVectorType , System2::template StateVectorType>::template VecType,
          System1::template InputVectorType,System2::template OutputVectorType,
          System1::isTimeVarying||System2::isTimeVarying,
          System1::isDirectFeedthrough&&System2::isDirectFeedthrough> {
  public:
    template <typename ScalarType> using StateVector = CombinedVector<ScalarType, System1::template StateVectorType , System2::template StateVectorType>;
    template <typename ScalarType> using StateVector1 = typename System1::template StateVectorType<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVectorType<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVectorType<ScalarType>;
    template <typename ScalarType> using System1OutputVector = typename System1::template OutputVectorType<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System2::template OutputVectorType<ScalarType>;
    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;

//    static_assert(std::is_same<System1::OutputVectorType<double>,System2::InputVectorType<double>>,"System 2 input vector must match System 1 output vector");
    // todo: assert that StateVector is indeed a CombinedVector<...,StateVector1,StateVector2>

    CascadeSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) {
    };

    template <typename ScalarType>
    StateVector<ScalarType> dynamicsImplementation(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
//      System1::OutputVectorType<ScalarType> y1;  // don't understand why this doesn't work (then could get rid of System1OutputVector)
      System1OutputVector<ScalarType> y1 = sys1->output(t,x.first(),u);
      StateVector<ScalarType> xdot(sys1->dynamics(t,x.first(),u),
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
      System1OutputVector<ScalarType> y1 = sys1->output(t,x.first(),u);
      OutputVector<ScalarType> y2 = sys2->output(t,x.second(),y1);
      return y2;
    }
    template <typename ScalarType>
    OutputVector<ScalarType> outputImplementation(const ScalarType& t, const StateVector<ScalarType>& x) const {
      InputVector<ScalarType> u;
      return outputImplementation(t,x,u);
    }
    template <typename ScalarType>
    OutputVector<ScalarType> outputImplementation(const StateVector<ScalarType>& x) const {
      ScalarType t;
      InputVector<ScalarType> u;
      return outputImplementation(t,x,u);
    }

  private:
    System1Ptr sys1;
    System2Ptr sys2;
  };

  template <typename System1, typename System2>
  std::shared_ptr<CascadeSystem<System1,System2>> cascade(const std::shared_ptr<System1>& sys1, const std::shared_ptr<System2>& sys2)
  {
    return std::make_shared<CascadeSystem<System1,System2> >(sys1,sys2);
  };



} // end namespace Drake


#endif //DRAKE_SYSTEM_H
