#ifndef DRAKE_SYSTEM_H
#define DRAKE_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "drake/core/Core.h"

namespace Drake {

/** @defgroup modeling Modeling Dynamical Systems
 * @{
 * @brief Algorithms for combining sub-systems into a (potentially complex)
 * system
 * @}
 */

/**
 * @defgroup system_concept System Concept
 * @ingroup concepts
 * @ingroup modeling
 * @{
 * @brief Describes a dynamical system that is compatible with most of our tools
 *for design and analysis
 *
 * @nbsp
 *
 * | Every model of this concept must implement |  |
 * ---------------------|------------------------------------------------------------|
 * | X::StateVector     | type for the internal state of the system, which
 *models the Vector<ScalarType> concept |
 * | X::InputVector     | type for the input to the system, which models the
 *Vector<ScalarType> concept |
 * | X::OutputVector    | type for the output from the system, which models the
 *Vector<ScalarType> concept |
 * | template <ScalarType> StateVector<ScalarType> X::dynamics(const ScalarType&
 *t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) | @f$
 *\dot{x} = \text{dynamics}(t, x, u) @f$ |
 * | template <ScalarType> OutputVector<ScalarType> X::output(const ScalarType&
 *t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) | @f$ y
 *= \text{output}(t, x, u) @f$  |
 * | bool isTimeVarying()  | should return false if output() and dynamics()
 *methods do not depend on time.  @default true |
 * | bool isDirectFeedthrough() | should return false if output() does not
 *depend directly on the input u.  @default true |
 * | size_t getNumStates() | only required if the state vector is
 *dynamically-sized |
 * | size_t getNumInputs() | only required if the input vector is
 *dynamically-sized |
 * | size_t getNumOutputs() | only required if the output vector is
 *dynamically-sized |
 *
 * (always try to label your methods with const if possible)
 *
 * todo: dynamics and output should be implemented as Drake::Function(s) with
 *input-output relationships defined.  then we would no longer specify
 *isTimeVarying and isDirectFeedthrough (we could extract them from the
 *input-output relationship)
 * todo: move xdot and y to be arguments instead of return values, to be
 *consistent with Drake::Function.
 *
 * @nbsp
 *
 * @par Coming soon.  Support for:
 *   - deterministic discrete update
 *   - input limits
 *   - state constraints (must also allow for slack variables)
 *   - zero-crossings (to inform the tools of discontinuities in the dynamics)
 * @}
 */

namespace internal {
template <typename System, bool Enable = false>
struct NumStatesDispatch {
  static std::size_t eval(const System& sys) {
    return System::template StateVector<double>::RowsAtCompileTime;
  }
};

template <typename System>
struct NumStatesDispatch<System, true> {
  static std::size_t eval(const System& sys) { return sys.getNumStates(); }
};
}
/** getNumStates()
 * @brief Retrieve the size of the state vector
 * @concept{system_concept}
 *
 * @retval RowsAtCompileTime or the result of getNumStates() for dynamically
 *sized vectors
 */
template <typename System>
std::size_t getNumStates(const System& sys) {
  return internal::NumStatesDispatch<
      System,
      System::template StateVector<double>::RowsAtCompileTime == -1>::eval(sys);
}

namespace internal {
template <typename System, bool Enable = false>
struct NumInputsDispatch {
  static std::size_t eval(const System& sys) {
    return System::template InputVector<double>::RowsAtCompileTime;
  }
};

template <typename System>
struct NumInputsDispatch<System, true> {
  static std::size_t eval(const System& sys) { return sys.getNumInputs(); }
};
}
/** getNumInputs()
 * @brief Retrieve the size of the input vector
 * @concept{system_concept}
 *
 * @retval RowsAtCompileTime or the result of getNumInputs() for dynamically
 *sized vectors
 */
template <typename System>
std::size_t getNumInputs(const System& sys) {
  return internal::NumInputsDispatch<
      System,
      System::template InputVector<double>::RowsAtCompileTime == -1>::eval(sys);
}

namespace internal {
template <typename System, bool Enable = false>
struct NumOutputsDispatch {
  static std::size_t eval(const System& sys) {
    return System::template OutputVector<double>::RowsAtCompileTime;
  }
};

template <typename System>
struct NumOutputsDispatch<System, true> {
  static std::size_t eval(const System& sys) { return sys.getNumOutputs(); }
};
}
/** getNumOutputs()
 * @brief Retrieve the size of the output vector
 * @concept{system_concept}
 *
 * @retval RowsAtCompileTime or the result of getNumOutputs() for dynamically
 *sized vectors
 */
template <typename System>
std::size_t getNumOutputs(const System& sys) {
  return internal::NumOutputsDispatch<
      System, System::template OutputVector<double>::RowsAtCompileTime ==
                  -1>::eval(sys);
}

namespace internal {
template <typename System, typename Enable = void>
struct RandomVectorDispatch {
  static typename System::template StateVector<double> getRandomState(
      const System& sys) {
    return getRandomVector<System::template StateVector>();
  }
};
template <typename System>
struct RandomVectorDispatch<
    System, typename std::enable_if<
                System::template StateVector<double>::RowsAtCompileTime ==
                Eigen::Dynamic>::type> {
  static typename System::template StateVector<double> getRandomState(
      const System& sys) {
    return System::template StateVector<double>(
        Eigen::VectorXd::Random(getNumStates(sys)));
  }
};
}
/** getInitialState()
 * @brief Returns a random feasible initial condition
  */
template <typename System>
typename System::template StateVector<double> getInitialState(
    const System& sys) {
  return internal::RandomVectorDispatch<System>::getRandomState(sys);
};

/** @brief Create a new, uninitialized state vector for the system.
 * @concept{system_concept}
 * @return a new, uninitialized state vector for the system.
 */
template <typename Scalar, typename System>
typename System::template StateVector<Scalar> createStateVector(
    const System& sys);

namespace internal {
template <typename System, typename Scalar, class Enable = void>
struct CreateStateVectorDispatch {
  static typename System::template StateVector<Scalar> eval(const System& sys) {
    return typename System::template StateVector<Scalar>();
  }
};

// case: Eigen vector
template <typename System, typename Scalar>
struct CreateStateVectorDispatch<
    System, Scalar,
    typename std::enable_if<is_eigen_vector<
        typename System::template StateVector<Scalar>>::value>::type> {
  static typename System::template StateVector<Scalar> eval(const System& sys) {
    return
        typename System::template StateVector<Scalar>(Drake::getNumStates(sys));
  }
};

// case: Combined vector
template <typename System, typename Scalar>
struct CreateStateVectorDispatch<
    System, Scalar,
    typename std::enable_if<is_combined_vector<
        typename System::template StateVector<Scalar>>::value>::type> {
  static typename System::template StateVector<Scalar> eval(const System& sys) {
    auto x1 = createStateVector<Scalar>(*sys.getSys1());
    auto x2 = createStateVector<Scalar>(*sys.getSys2());
    return typename System::template StateVector<Scalar>(x1, x2);
  }
};
}

template <typename Scalar, typename System>
typename System::template StateVector<Scalar> createStateVector(
    const System& sys) {
  return internal::CreateStateVectorDispatch<System, Scalar>::eval(sys);
};

}  // end namespace Drake

#endif  // DRAKE_SYSTEM_H
