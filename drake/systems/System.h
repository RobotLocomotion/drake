#ifndef DRAKE_SYSTEM_H
#define DRAKE_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Core.h"
#include "SystemSpecializations.h"

/** Modeling API:
 *
 *  System concept
 *  feedback
 *  cascade
 *
 */



/** A Drake::System is a dynamical system that is compatible with most of our tools for design and analysis.
 * It must have:
 *   - a real-vector-valued input, state, and output
 *   - deterministic dynamics and outputs given the input and state
 *   - continuous dynamics [ coming soon: no more than one discrete time-step (in addition to continuous dynamics) ]
 *  The input, state, and output coordinate systems are all described by CoordinateSystem objects
 *  In addition, it MAY have
 *   - time-varying dynamics and outputs
 *   - input limits (c++ support coming soon)
 *   - algebraic constraints (c++ support coming soon)
 *  - zero-crossings (c++ support coming soon) to inform the tools of discontinuities in the dynamics
 */

namespace Drake {

  // todo: document description of the System concept

  template <typename System>
  struct SystemSizeTraits {
    const static int num_states = System::template StateVector<double>::RowsAtCompileTime;
    const static int num_inputs = System::template InputVector<double>::RowsAtCompileTime;
    const static int num_outputs = System::template OutputVector<double>::RowsAtCompileTime;
    static_assert(num_states >= 0, "still need to handle the variable-size case");
    static_assert(num_inputs >= 0, "still need to handle the variable-size case");
    static_assert(num_outputs >= 0, "still need to handle the variable-size case");
  };

  template <typename System>
  struct SystemStructureTraits {
    const static bool isTimeVarying = SystemDynamicsMethodTraits<System>::hasTimeArgument || SystemOutputMethodTraits<System>::hasTimeArgument;
    const static bool isDirectFeedthrough = SystemOutputMethodTraits<System>::hasInputArgument;
    static_assert(SystemOutputMethodTraits<System>::hasOutputMethod,"Could not find the output method for your system.  Also, make sure it's mark const");
  };

  // provide consistent access methods for dynamics and output

  template <typename System, typename ScalarType>
  typename System::template StateVector<ScalarType> dynamics(const System& sys, const ScalarType& t, const typename System::template StateVector<ScalarType>& x, const typename System::template InputVector<ScalarType>& u) {
    return DynamicsDispatch<System, ScalarType, SystemStructureTraits<System>::isTimeVarying, (SystemSizeTraits<System>::num_states != 0), (SystemSizeTraits<System>::num_inputs != 0)>::eval(sys,t,x,u);
  }

  template <typename System, typename ScalarType>
  typename System::template StateVector<ScalarType> dynamics(const System& sys, const typename System::template StateVector<ScalarType>& x, const typename System::template InputVector<ScalarType>& u) {
    static_assert(!SystemStructureTraits<System>::isTimeVarying,"Called dynamics(sys,x,u) for a time-varying system");
    ScalarType t;
    return dynamics(sys,t,x,u);
  }

  template <typename System, typename ScalarType>
  typename System::template OutputVector<ScalarType> output(const System& sys, const ScalarType& t, const typename System::template StateVector<ScalarType>& x, const typename System::template InputVector<ScalarType>& u) {
    return OutputDispatch<System, ScalarType, SystemStructureTraits<System>::isTimeVarying, (SystemSizeTraits<System>::num_states != 0), ((SystemSizeTraits<System>::num_inputs != 0) && SystemStructureTraits<System>::isDirectFeedthrough)>::eval(sys, t, x, u);
  }
  template <typename System, typename ScalarType>
  typename System::template OutputVector<ScalarType> output(const System& sys, const ScalarType& t, const typename System::template StateVector<ScalarType>& x) {
    static_assert(!SystemStructureTraits<System>::isDirectFeedthrough,"Called output(sys,t,x) for a direct feedthrough system");
    typename System::template InputVector<ScalarType> u;
    return OutputDispatch<System, ScalarType, SystemStructureTraits<System>::isTimeVarying, (SystemSizeTraits<System>::num_states != 0), ((SystemSizeTraits<System>::num_inputs != 0) && SystemStructureTraits<System>::isDirectFeedthrough)>::eval(sys, t, x, u);
  }


  template <class System1, class System2>
  class FeedbackSystem {
  public:
    template <typename ScalarType> using StateVector = typename CombinedVectorBuilder<System1::template StateVector , System2::template StateVector>::template VecType<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVector<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System1::template OutputVector<ScalarType>;
    const static int num_inputs = SystemSizeTraits<System1>::num_inputs;

    template <typename ScalarType> using StateVector1 = typename System1::template StateVector<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVector<ScalarType>;
    template <typename ScalarType> using EigenInput = Eigen::Matrix<ScalarType,num_inputs,1>;
    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;

    static_assert(!SystemStructureTraits<System1>::isDirectFeedthrough || !SystemStructureTraits<System2>::isDirectFeedthrough,"Algebraic Loop: cannot feedback combine two systems that are both direct feedthrough");
//    static_assert(System1::OutputVectorType == System2::InputVectorType,"System 2 input vector must match System 1 output vector");
//    static_assert(System2::OutputVectorType == System1::InputVectorType,"System 1 input vector must match System 2 output vector");
    // todo: assert that StateVector is indeed a CombinedVector<...,StateVector1,StateVector2>

    FeedbackSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) { };

// todo: implement only the correct dynamics signature based on the system traits of System1 and System2

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      OutputVector<ScalarType> y1;
      InputVector<ScalarType> y2;
      subsystemOutputs(t,x.first(),x.second(),u,y1,y2);

      StateVector<ScalarType> xdot(sys1->dynamics(t,x.first(),static_cast<InputVector<ScalarType> >( static_cast<EigenInput<ScalarType> >(y2)+static_cast<EigenInput<ScalarType> >(u))),
                                   sys2->dynamics(t,x.second(),y1));
      return xdot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
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


  private:
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

  template <typename System1, typename System2>
  std::shared_ptr<FeedbackSystem<System1,System2>> feedback(const std::shared_ptr<System1>& sys1, const std::shared_ptr<System2>& sys2)
  {
    return std::make_shared<FeedbackSystem<System1,System2> >(sys1,sys2);
  };

  template <typename System1, typename System2>
  struct SystemStructureTraits<FeedbackSystem<System1,System2>> {
    const static bool isTimeVarying = SystemStructureTraits<System1>::isTimeVarying || SystemStructureTraits<System2>::isTimeVarying;
    const static bool isDirectFeedthrough = SystemStructureTraits<System1>::isDirectFeedthrough;
  };


  template <class System1, class System2>
  class CascadeSystem {
  public:
    template <typename ScalarType> using StateVector = typename CombinedVectorBuilder<System1::template StateVector , System2::template StateVector>::template VecType<ScalarType>;
    template <typename ScalarType> using StateVector1 = typename System1::template StateVector<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVector<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVector<ScalarType>;
    template <typename ScalarType> using System1OutputVector = typename System1::template OutputVector<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System2::template OutputVector<ScalarType>;
    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;

//    static_assert(std::is_same<System1::OutputVectorType<double>,System2::InputVectorType<double>>,"System 2 input vector must match System 1 output vector");
    // todo: assert that StateVector is indeed a CombinedVector<...,StateVector1,StateVector2>

    CascadeSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) {
    };

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
//      System1::OutputVectorType<ScalarType> y1;  // don't understand why this doesn't work (then could get rid of System1OutputVector)
      System1OutputVector<ScalarType> y1 = output(*sys1,t,x.first(),u);
      StateVector<ScalarType> xdot(dynamics(*sys1,t,x.first(),u),
                                   dynamics(*sys2,t,x.second(),y1));
      return xdot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      System1OutputVector<ScalarType> y1 = output(*sys1,t,x.first(),u);
      OutputVector<ScalarType> y2 = output(*sys2,t,x.second(),y1);
      return y2;
    }
    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x) const {
      if (SystemStructureTraits<System2>::isDirectFeedthrough) {
        static_assert(!SystemStructureTraits<System1>::isDirectFeedthrough,"if I get here, then system 1 had better not be direct feedthrough");
        return output<System2,ScalarType>(*sys2, t, x.second(), output<System1,ScalarType>(*sys1, t, x.first()));
      } else {
        return output<System2,ScalarType>(*sys2, t, x.second());
      }
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

  template <typename System1, typename System2>
  struct SystemStructureTraits<CascadeSystem<System1,System2>> {
    const static bool isTimeVarying = SystemStructureTraits<System1>::isTimeVarying || SystemStructureTraits<System2>::isTimeVarying;
    const static bool isDirectFeedthrough = SystemStructureTraits<System1>::isDirectFeedthrough && SystemStructureTraits<System2>::isDirectFeedthrough;
  };


} // end namespace Drake


#endif //DRAKE_SYSTEM_H
