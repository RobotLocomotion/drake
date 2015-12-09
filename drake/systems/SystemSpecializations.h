#ifndef DRAKE_SYSTEMSPECIALIZATIONS_H
#define DRAKE_SYSTEMSPECIALIZATIONS_H

namespace Drake {

  // Use class template specialization trick to enable the method signature of the System::dynamicsImplementation to change based on the template parameters
  // Unfortunately, I have to do something like this so that the compiler only looks for the one method signature (a switch statement in the dynamics
  // method could call only the right method, but all of them would have to exist at compile time)
  template<bool hasTime, bool hasDynamics, bool hasInput, typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector>
  struct DynamicsDispatch {
    static StateVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                        const InputVector<ScalarType> &u) {
      return sys->dynamicsImplementation(t, x, u);
    }
  };

  template<typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector>
  struct DynamicsDispatch<false, true, true, ScalarType, Derived, StateVector, InputVector> { // specialization time-invariant dynamics w/ input
    static StateVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                        const InputVector<ScalarType> &u) {
      return sys->dynamicsImplementation(x, u);
    }
  };

  template<typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector>
  struct DynamicsDispatch<false, true, false, ScalarType, Derived, StateVector, InputVector> { // specialization time-invariant dynamics w/ input
    static StateVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                        const InputVector<ScalarType> &u) {
      return sys->dynamicsImplementation(x);
    }
  };

  template<bool hasTime, bool hasInput, typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector>
  struct DynamicsDispatch<hasTime, false, hasInput, ScalarType, Derived, StateVector, InputVector> { // specialization for no dynamics.  should probably never get called, but needs to compile
    static StateVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                        const InputVector<ScalarType> &u) {
      StateVector<ScalarType> empty;
      return empty;
    }
  };

  template<bool hasOutput, bool hasTime, bool hasDynamics, bool hasInput, typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector,
          template<typename> class OutputVector>
  struct OutputDispatch {
    static OutputVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                         const InputVector<ScalarType> &u) {
      return sys->outputImplementation(t, x, u);
    }
  };

#define OutputDispatchSpecialization(output, time, state, input, func_call) \
  template <typename ScalarType, typename Derived, template <typename> class StateVector, template <typename> class InputVector, template <typename> class OutputVector> \
  struct OutputDispatch<output,time,state,input,ScalarType,Derived,StateVector,InputVector,OutputVector>   { \
    static OutputVector<ScalarType> eval(const Derived* sys, const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) \
    { \
      return func_call; \
    } \
  };

  OutputDispatchSpecialization(true, true, true, false, sys->outputImplementation(t,x))
  OutputDispatchSpecialization(true, true, false, true, sys->outputImplementation(t,u))
  OutputDispatchSpecialization(true, true, false, false, sys->outputImplementation(t))
  OutputDispatchSpecialization(true, false, true, true, sys->outputImplementation(x,u))
  OutputDispatchSpecialization(true, false, true, false, sys->outputImplementation(x))
  OutputDispatchSpecialization(true, false, false, true, sys->outputImplementation(u))

  template<bool hasTime, bool hasDynamics, bool hasInput, typename ScalarType, typename Derived, template<typename> class StateVector,
          template<typename> class InputVector,
          template<typename> class OutputVector>
  struct OutputDispatch<false, hasTime, hasDynamics, hasInput, ScalarType, Derived, StateVector, InputVector, OutputVector> {
    static OutputVector<ScalarType> eval(const Derived *sys, const ScalarType &t, const StateVector<ScalarType> &x,
                                         const InputVector<ScalarType> &u) {
      OutputVector<ScalarType> empty;
      return empty;
    }
  };

};

#endif //DRAKE_SYSTEMSPECIALIZATIONS_H
