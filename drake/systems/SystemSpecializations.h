#ifndef DRAKE_SYSTEMSPECIALIZATIONS_H
#define DRAKE_SYSTEMSPECIALIZATIONS_H

#include <type_traits>

namespace Drake {

  // Use class template specialization trick to enable the method signature of the System::dynamicsImplementation to change based on the template parameters
  // Unfortunately, I have to do something like this so that the compiler only looks for the one method signature (a switch statement in the dynamics
  // method could call only the right method, but all of them would have to exist at compile time)
  template<typename System, typename ScalarType, bool hasTime, bool hasDynamics, bool hasInput>
  struct DynamicsDispatch {
    static typename System::template StateVector<ScalarType> eval(const System &sys, const ScalarType &t, const typename System::template StateVector<ScalarType> &x,
                                        const typename System::template InputVector<ScalarType> &u) {
      return sys.dynamics(t, x, u);
    }
  };

  template<typename System, typename ScalarType>
  struct DynamicsDispatch<System,ScalarType,false, true, true> { // specialization time-invariant dynamics w/ input
    static typename System::template StateVector<ScalarType> eval(const System &sys, const ScalarType &t, const typename System::template StateVector<ScalarType> &x,
                                                                  const typename System::template InputVector<ScalarType> &u) {
      return sys.dynamics(x, u);
    }
  };

  template<typename System, typename ScalarType>
  struct DynamicsDispatch<System,ScalarType,false, true, false> { // specialization time-invariant dynamics w/ input
    static typename System::template StateVector<ScalarType> eval(const System &sys, const ScalarType &t, const typename System::template StateVector<ScalarType> &x,
                                                                  const typename System::template InputVector<ScalarType> &u) {
      return sys.dynamics(x);
    }
  };

  template<typename System, typename ScalarType, bool hasTime, bool hasInput>
  struct DynamicsDispatch<System,ScalarType,hasTime, false, hasInput> { // specialization for no dynamics.  should probably never get called, but needs to compile
    static typename System::template StateVector<ScalarType> eval(const System &sys, const ScalarType &t, const typename System::template StateVector<ScalarType> &x,
                                                                  const typename System::template InputVector<ScalarType> &u) {
      typename System::template StateVector<ScalarType> empty;
      return empty;
    }
  };

  template <typename System, typename Enable = void>
  struct SystemDynamicsMethodTraits {
    static const bool hasDynamicsMethod = false;
    static const bool hasTimeArgument = false;
    static const bool hasStateArgument = false;
    static const bool hasInputArgument = false;
  };

#define DrakeDynamicsMethodTraitsSpecialization(time, state, input, ...) \
  template <typename System> \
  struct SystemDynamicsMethodTraits<System, typename std::enable_if<std::is_same<decltype(&System::template dynamics<double>), typename System::template StateVector<double> (System::*)( __VA_ARGS__ )>::value >::type> { \
    static const bool hasDynamicsMethod = true; \
    static const bool hasTimeArgument = time; \
    static const bool hasStateArgument = state; \
    static const bool hasInputArgument = input; \
  }; \
  template <typename System> \
  struct SystemDynamicsMethodTraits<System, typename std::enable_if<std::is_same<decltype(&System::dynamics), typename System::template StateVector<double> (System::*)( __VA_ARGS__ )>::value >::type> { \
      static const bool hasDynamicsMethod = true; \
      static const bool hasTimeArgument = time; \
      static const bool hasStateArgument = state; \
      static const bool hasInputArgument = input; \
  };

  DrakeDynamicsMethodTraitsSpecialization(true,true,true,const double&, const typename System::template StateVector<double>&, const typename System::template InputVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(true,true,false,const double&, const typename System::template StateVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(true,false,true,const double&, const typename System::template InputVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(true,false,false,const double&)
  DrakeDynamicsMethodTraitsSpecialization(false,true,true,const typename System::template StateVector<double>&, const typename System::template InputVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(false,true,false,const typename System::template StateVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(false,false,true,const typename System::template InputVector<double>&)
  DrakeDynamicsMethodTraitsSpecialization(false,false,false)

#undef DrakeOutputMethodTraitsSpecialization


  template<typename System, typename ScalarType, bool hasTime, bool hasDynamics, bool hasInput>
  struct OutputDispatch {
    static typename System::template OutputVector<ScalarType> eval(const System& sys, const ScalarType &t, const typename System::template StateVector<ScalarType> &x,
                                         const typename System::template InputVector<ScalarType> &u) {
      return sys.output(t, x, u);
    }
  };

#define DrakeOutputDispatchSpecialization(time, state, input, func_call) \
  template <typename System, typename ScalarType> \
  struct OutputDispatch<System,ScalarType,time,state,input>   { \
    static typename System::template OutputVector<ScalarType> eval(const System& sys, const ScalarType& t, const typename System::template StateVector<ScalarType>& x, const typename System::template InputVector<ScalarType>& u) \
    { \
      return func_call; \
    } \
  };

  DrakeOutputDispatchSpecialization(true, true, false, sys.output(t,x))
  DrakeOutputDispatchSpecialization(true, false, true, sys.output(t,u))
  DrakeOutputDispatchSpecialization(true, false, false, sys.output(t))
  DrakeOutputDispatchSpecialization(false, true, true, sys.output(x,u))
  DrakeOutputDispatchSpecialization(false, true, false, sys.output(x))
  DrakeOutputDispatchSpecialization(false, false, true, sys.output(u))
  DrakeOutputDispatchSpecialization(false, false, false, sys.output())

#undef DrakeOutputDispatchSpecialization


  template <typename System, typename Enable = void>
  struct SystemOutputMethodTraits {
    static const bool hasOutputMethod = false;
    static const bool hasTimeArgument = false;
    static const bool hasStateArgument = false;
    static const bool hasInputArgument = false;
  };

#define DrakeOutputMethodTraitsSpecialization(time, state, input, ...) \
  template <typename System> \
  struct SystemOutputMethodTraits<System, typename std::enable_if<std::is_same<decltype(&System::template output<double>), typename System::template OutputVector<double> (System::*)( __VA_ARGS__ ) const>::value >::type> { \
    static const bool hasOutputMethod = true; \
    static const bool hasTimeArgument = time; \
    static const bool hasStateArgument = state; \
    static const bool hasInputArgument = input; \
  }; \
  template <typename System> \
  struct SystemOutputMethodTraits<System, typename std::enable_if<std::is_same<decltype(&System::output), typename System::template OutputVector<double> (System::*)( __VA_ARGS__ ) const>::value >::type> { \
    static const bool hasOutputMethod = true; \
    static const bool hasTimeArgument = time; \
    static const bool hasStateArgument = state; \
    static const bool hasInputArgument = input; \
  };

  DrakeOutputMethodTraitsSpecialization(true,true,false,const double&, const typename System::template StateVector<double>&)
  DrakeOutputMethodTraitsSpecialization(true,false,true,const double&, const typename System::template InputVector<double>&)
  DrakeOutputMethodTraitsSpecialization(true,false,false,const double&)
  DrakeOutputMethodTraitsSpecialization(false,true,true,const typename System::template StateVector<double>&, const typename System::template InputVector<double>&)
  DrakeOutputMethodTraitsSpecialization(false,true,false,const typename System::template StateVector<double>&)
  DrakeOutputMethodTraitsSpecialization(false,false,true,const typename System::template InputVector<double>&)
  DrakeOutputMethodTraitsSpecialization(false,false,false)

#undef DrakeOutputMethodTraitsSpecialization
};

#endif //DRAKE_SYSTEMSPECIALIZATIONS_H
