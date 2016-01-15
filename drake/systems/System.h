#ifndef DRAKE_SYSTEM_H
#define DRAKE_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "drake/core/Core.h"
#include "drake/systems/SystemSpecializations.h"


namespace Drake {

/** @defgroup modeling Modeling Dynamical Systems
 * @{
 * @brief Algorithms for combining sub-systems into a (potentially complex) system
 * @}
 */


/**
 * @defgroup system_concept System Concept
 * @ingroup concepts
 * @ingroup modeling
 * @{
 * @brief Describes a dynamical system that is compatible with most of our tools for design and analysis
 *
 * @nbsp
 *
 * | Every model of this concept must implement |  |
 * ---------------------|------------------------------------------------------------|
 * | X::StateVector     | type for the internal state of the system, which models the Vector<ScalarType> concept |
 * | X::InputVector     | type for the input to the system, which models the Vector<ScalarType> concept |
 * | X::OutputVector    | type for the output from the system, which models the Vector<ScalarType> concept |
 * | template <ScalarType> StateVector<ScalarType> X::dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) | @f$ \dot{x} = \text{dynamics}(t,x,u) @f$ |
 * | template <ScalarType> OutputVector<ScalarType> X::output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) | @f$ y = \text{output}(t,x,u) @f$  |
 * | bool isTimeVarying()  | should return false if output() and dynamics() methods do not depend on time.  @default true |
 * | bool isDirectFeedthrough() | should return false if output() does not depend directly on the input u.  @default true |
 * | size_t getNumStates() | only required if the state vector is dynamically-sized |
 * | size_t getNumInputs() | only required if the input vector is dynamically-sized |
 * | size_t getNumOutputs() | only required if the output vector is dynamically-sized |
 *
 * (always try to label your methods with const if possible)
 *
 * @nbsp
 *
 * @par Coming soon.  Support for:
 *   - deterministic discrete update
 *   - input limits
 *   - state constraints
 *   - zero-crossings (to inform the tools of discontinuities in the dynamics)
 * @}
 */

  namespace internal {
    template<typename System, bool Enable = false>
    struct NumStatesDispatch {
      static std::size_t eval(const System &sys) { return System::template StateVector<double>::RowsAtCompileTime; }
    };

    template<typename System>
    struct NumStatesDispatch<System,true > {
      static std::size_t eval(const System &sys) { return sys.getNumStates(); }
    };
  }
  /** getNumStates()
   * @brief Retrieve the size of the state vector
   * @concept{system_concept}
   *
   * @retval RowsAtCompileTime or the result of getNumStates() for dynamically sized vectors
   */
  template <typename System> std::size_t getNumStates(const System& sys) { return internal::NumStatesDispatch<System,System::template StateVector<double>::RowsAtCompileTime==-1>::eval(sys); }

  namespace internal {
    template<typename System, bool Enable = false>
    struct NumInputsDispatch {
      static std::size_t eval(const System &sys) { return System::template InputVector<double>::RowsAtCompileTime; }
    };

    template<typename System>
    struct NumInputsDispatch<System,true > {
      static std::size_t eval(const System &sys) { return sys.getNumInputs(); }
    };
  }
  /** getNumInputs()
   * @brief Retrieve the size of the input vector
   * @concept{system_concept}
   *
   * @retval RowsAtCompileTime or the result of getNumInputs() for dynamically sized vectors
   */
  template <typename System> std::size_t getNumInputs(const System& sys) { return internal::NumInputsDispatch<System,System::template InputVector<double>::RowsAtCompileTime==-1>::eval(sys); }

  namespace internal {
    template<typename System, bool Enable = false>
    struct NumOutputsDispatch {
      static std::size_t eval(const System &sys) { return System::template OutputVector<double>::RowsAtCompileTime; }
    };

    template<typename System>
    struct NumOutputsDispatch<System,true > {
      static std::size_t eval(const System &sys) { return sys.getNumOutputs(); }
    };
  }
  /** getNumOutputs()
   * @brief Retrieve the size of the output vector
   * @concept{system_concept}
   *
   * @retval RowsAtCompileTime or the result of getNumOutputs() for dynamically sized vectors
   */
  template <typename System> std::size_t getNumOutputs(const System& sys) { return internal::NumOutputsDispatch<System,System::template OutputVector<double>::RowsAtCompileTime==-1>::eval(sys); }

  namespace internal {
    template <typename System, typename Enable = void>
    struct RandomVectorDispatch {
      static typename System::template StateVector<double> getRandomState(const System& sys) { return getRandomVector<System::template StateVector>(); }
    };
    template <typename System>
    struct RandomVectorDispatch<System, typename std::enable_if<System::template StateVector<double>::RowsAtCompileTime==Eigen::Dynamic>::type> {
      static typename System::template StateVector<double> getRandomState(const System& sys) { return System::template StateVector<double>(Eigen::VectorXd::Random(getNumStates(sys))); }
    };
  }
  /** getInitialState()
   * @brief Returns a random feasible initial condition
    */
  template <typename System> typename System::template StateVector<double> getInitialState(const System& sys) { return internal::RandomVectorDispatch<System>::getRandomState(sys); };

  /** @brief Create a new, uninitialized state vector for the system.
   * @concept{system_concept}
   * @return a new, uninitialized state vector for the system.
   */
  template <typename Scalar, typename System>
  typename System::template StateVector<Scalar> createStateVector(const System &sys);

  namespace internal {
    template <typename System, typename Scalar, class Enable = void>
    struct CreateStateVectorDispatch {
      static typename System::template StateVector<Scalar> eval(const System& sys) {
        return typename System::template StateVector<Scalar>();
      }
    };

    // case: Eigen vector
    template <typename System, typename Scalar>
    struct CreateStateVectorDispatch<System, Scalar, typename std::enable_if<is_eigen_vector<typename System::template StateVector<Scalar>>::value>::type >{
      static typename System::template StateVector<Scalar> eval(const System& sys) {
        return typename System::template StateVector<Scalar>(Drake::getNumStates(sys));
      }
    };

    // case: Combined vector
    template<typename System, typename Scalar>
    struct CreateStateVectorDispatch<System, Scalar, typename std::enable_if< is_combined_vector<typename System::template StateVector<Scalar>>::value>::type > {
      static typename System::template StateVector<Scalar> eval(const System& sys) {
        auto x1 = createStateVector<Scalar>(*sys.getSys1());
        auto x2 = createStateVector<Scalar>(*sys.getSys2());
        return typename System::template StateVector<Scalar>(x1, x2);
      }
    };
  }

  template <typename Scalar, typename System>
  typename System::template StateVector<Scalar> createStateVector(const System& sys) {
    return internal::CreateStateVectorDispatch<System, Scalar>::eval(sys);
  };

/** FeedbackSystem<System1,System2>
 * @brief Builds a new system from the feedback connection of two simpler systems
 * @concept{system_concept}
 *
 * ![Feedback combination of two systems](http://underactuated.csail.mit.edu/figures/feedback_system.svg)
 *
 */

  template <class System1, class System2>
  class FeedbackSystem {
  public:
    template <typename ScalarType> using StateVector1 = typename System1::template StateVector<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVector<ScalarType>;
    template <typename ScalarType> using StateVector = typename CombinedVectorUtil<System1::template StateVector, System2::template StateVector>::template type<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVector<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System1::template OutputVector<ScalarType>;
    typedef CombinedVectorUtil<StateVector1,StateVector2> util;

    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;

    static_assert(std::is_same<typename System1::template OutputVector<double>,typename System2::template InputVector<double>>::value,"System 2 input vector must match System 1 output vector");
    static_assert(std::is_same<typename System2::template OutputVector<double>,typename System1::template InputVector<double>>::value,"System 1 input vector must match System 2 output vector");

    FeedbackSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) { };

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      OutputVector<ScalarType> y1;
      InputVector<ScalarType> y2;
      auto x1 = util::first(x);
      auto x2 = util::second(x);
      subsystemOutputs(t,x1,x2,u,y1,y2);

      StateVector<ScalarType> xdot = util::combine(sys1->dynamics(t,x1,static_cast<InputVector<ScalarType> >(toEigen(y2)+toEigen(u))),
                                   sys2->dynamics(t,x2,y1));
      return xdot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      OutputVector<ScalarType> y1;
      auto x1 = util::first(x);
      if (!sys1->isDirectFeedthrough()) {
        y1 = sys1->output(t,x1,u);   // then don't need u+y2 here, u will be ignored
      } else {
        InputVector<ScalarType> y2;
        auto x2 = util::second(x);
        y2 = sys2->output(t,x2,y1); // y1 might be uninitialized junk, but has to be ok
        y1 = sys1->output(t,x1,static_cast<InputVector<ScalarType> >(toEigen(y2)+toEigen(u)));
      }
      return y1;
    }

    bool isTimeVarying() const { return sys1->isTimeVarying() || sys2->isTimeVarying(); }
    bool isDirectFeedthrough() const { return sys1->isDirectFeedthrough(); }
    size_t getNumStates() const {return Drake::getNumStates(*sys1) + Drake::getNumStates(*sys2); };
    size_t getNumInputs() const {return Drake::getNumInputs(*sys1); };
    size_t getNumOutputs() const {return Drake::getNumOutputs(*sys1); };


    const System1Ptr& getSys1() const {
      return sys1;
    }

    const System2Ptr& getSys2() const {
      return sys2;
    }

    friend StateVector<double> getInitialState(const FeedbackSystem<System1,System2>& sys) {
      return util::combine( getInitialState(*(sys.sys1)), getInitialState(*(sys.sys2)));
    }

  private:
    template <typename ScalarType>
    void subsystemOutputs(const ScalarType& t, const StateVector1<ScalarType>& x1, const StateVector2<ScalarType>& x2, const InputVector<ScalarType> &u,
                          OutputVector<ScalarType> &y1, InputVector<ScalarType> &y2) const {
      if (!sys1->isDirectFeedthrough()) {
        y1 = sys1->output(t,x1,u);  // output does not depend on u (so it's ok that we're not passing u+y2)
        y2 = sys2->output(t,x2,y1);
      } else {
        y2 = sys2->output(t,x2,y1); // y1 might be uninitialized junk, but has to be ok
        y1 = sys1->output(t,x1,static_cast<InputVector<ScalarType> >( toEigen(y2)+toEigen(u)));
      }
    }

    System1Ptr sys1;
    System2Ptr sys2;
  };

  /** feedback(sys1, sys2)
   * @brief Convenience method to create a feedback combination of two systems
   * @ingroup modeling
   */
  template <typename System1, typename System2>
  std::shared_ptr<FeedbackSystem<System1,System2>> feedback(const std::shared_ptr<System1>& sys1, const std::shared_ptr<System2>& sys2)
  {
    return std::make_shared<FeedbackSystem<System1,System2> >(sys1,sys2);
  };

/** CascadeSystem<System1,System2>
 * @brief Builds a new system from the cascade connection of two simpler systems
 * @concept{system_concept}
 *
 * ![Cascade combination of two systems](http://underactuated.csail.mit.edu/figures/cascade_system.svg)
 *
 */
  template <class System1, class System2>
  class CascadeSystem {
  public:
    template <typename ScalarType> using StateVector = typename CombinedVectorUtil<System1::template StateVector, System2::template StateVector>::template type<ScalarType>;
    template <typename ScalarType> using StateVector1 = typename System1::template StateVector<ScalarType>;
    template <typename ScalarType> using StateVector2 = typename System2::template StateVector<ScalarType>;
    template <typename ScalarType> using InputVector = typename System1::template InputVector<ScalarType>;
    template <typename ScalarType> using System1OutputVector = typename System1::template OutputVector<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System2::template OutputVector<ScalarType>;
    typedef std::shared_ptr<System1> System1Ptr;
    typedef std::shared_ptr<System2> System2Ptr;
    typedef CombinedVectorUtil<StateVector1,StateVector2> util;

    static_assert(std::is_same<typename System1::template OutputVector<double>,typename System2::template InputVector<double>>::value,"System 2 input vector must match System 1 output vector");

    CascadeSystem(const System1Ptr& _sys1, const System2Ptr& _sys2) : sys1(_sys1),sys2(_sys2) { };

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      auto x1 = util::first(x);
      auto x2 = util::second(x);
      System1OutputVector<ScalarType> y1 = sys1->output(t,x1,u);
      StateVector<ScalarType> xdot = util::combine(sys1->dynamics(t,x1,u),
                                                   sys2->dynamics(t,x2,y1));
      return xdot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      auto x1 = util::first(x);
      auto x2 = util::second(x);
      System1OutputVector<ScalarType> y1 = sys1->output(t,x1,u);
      OutputVector<ScalarType> y2 = sys2->output(t,x2,y1);
      return y2;
    }

    bool isTimeVarying() const { return sys1->isTimeVarying() || sys2->isTimeVarying(); }
    bool isDirectFeedthrough() const { return sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough(); }
    size_t getNumStates() const {return Drake::getNumStates(*sys1) + Drake::getNumStates(*sys2); };
    size_t getNumInputs() const {return Drake::getNumInputs(*sys1); };
    size_t getNumOutputs() const {return Drake::getNumOutputs(*sys2); };

  public:
    const System1Ptr& getSys1() const {
      return sys1;
    }

    const System2Ptr& getSys2() const {
      return sys2;
    }

    friend StateVector<double> getInitialState(const CascadeSystem<System1,System2>& sys) {
      return util::combine( getInitialState(*(sys.sys1)), getInitialState(*(sys.sys2)));
    }

  private:
    System1Ptr sys1;
    System2Ptr sys2;
  };

  /** cascade(sys1, sys2)
   * @brief Convenience method to create a cascade combination of two systems
   * @ingroup modeling
   */
  template <typename System1, typename System2>
  std::shared_ptr<CascadeSystem<System1,System2>> cascade(const std::shared_ptr<System1>& sys1, const std::shared_ptr<System2>& sys2)
  {
    return std::make_shared<CascadeSystem<System1,System2> >(sys1,sys2);
  };


/** PDControlSystem<System>
 * @brief Wraps an existing system with a PD controller (the new system represents the closed-loop controller + system)
 * @concept{system_concept}
 *
 *   x_d --->[ Kp, Kd ]-->(+)----->[ sys ]----------> yout
 *                     | -                 |
 *                     -------[ Kp, Kd ]<----
 *
 */
  template <class System>
  class PDControlSystem {
  public:
    template <typename ScalarType> using StateVector = typename System::template StateVector<ScalarType>;
    template <typename ScalarType> using InputVector = typename System::template StateVector<ScalarType>;
    template <typename ScalarType> using OutputVector = typename System::template OutputVector<ScalarType>;
    typedef std::shared_ptr<System> SystemPtr;

    template <typename DerivedA, typename DerivedB>
    PDControlSystem(const SystemPtr& sys, const Eigen::MatrixBase<DerivedA>& Kp, const Eigen::MatrixBase<DerivedB>& Kd) :
            sys(sys),Kp(Kp),Kd(Kd)
    {
      assert(Drake::getNumInputs(*sys) == Kp.rows() && "Kp must have the same number of rows as the system has inputs");
      assert(Kp.rows()==Kd.rows() && "Kd must have the same number of rows as Kp");
      assert(Drake::getNumStates(*sys) == Kp.cols()+Kd.cols() && "Kp and Kd must match the number of states");
    };

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      typename System::template InputVector<ScalarType> system_u = Kp*(toEigen(u).head(Kp.cols()) - toEigen(x).head(Kp.cols())) + Kd*(toEigen(u).tail(Kd.cols()) - toEigen(x).tail(Kd.cols()));
      return sys->dynamics(t,x,system_u);
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      typename System::template InputVector<ScalarType> system_u = Kp*(toEigen(u).head(Kp.cols()) - toEigen(x).head(Kp.cols())) + Kd*(toEigen(u).tail(Kd.cols()) - toEigen(x).tail(Kd.cols()));
      return sys->output(t,x,system_u);
    }

    bool isTimeVarying() const { return sys->isTimeVarying(); }
    bool isDirectFeedthrough() const { return sys->isDirectFeedthrough(); }
    size_t getNumStates() const {return Drake::getNumStates(*sys); };
    size_t getNumInputs() const {return Drake::getNumStates(*sys); };
    size_t getNumOutputs() const {return Drake::getNumOutputs(*sys); };

  public:
    const SystemPtr& getSys() const {
      return sys;
    }
    friend StateVector<double> getInitialState(const PDControlSystem<System>& sys) {
      return getInitialState(*sys.sys);
    }

  private:
    SystemPtr sys;
    Eigen::MatrixXd Kp, Kd;
  };

} // end namespace Drake


#endif //DRAKE_SYSTEM_H
