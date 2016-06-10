#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/state_vector_interface.h"

namespace drake {
namespace systems {

/** Compute `x += scale * y` with x and y equal-length state vectors.

@tparam T   A mathematical type compatible with Eigen's Scalar.

If you know that `scale`==1, use the provided `operator+=()` or the simpler 
PlusEq() signature.
@relates drake::systems::StateVectorInterface **/
template <class T>
void PlusEq(StateVectorInterface<T>* x, const T& scale,
            const StateVectorInterface<T>& y) {
  assert(x->size() == y.size());
  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const T& xi = x->GetAtIndex(i);
    const T& yi = y.GetAtIndex(i);
    x->SetAtIndex(i, xi + scale * yi);
  }
}

/** Compute `x += y` with `x` and `y` equal-length state vectors. See the other
PlusEq() signature if you want to scale `y`.

Equivalently, use the provided `operator+=()`.
@relates drake::systems::StateVectorInterface **/
template <class T>
void PlusEq(StateVectorInterface<T>* x, 
            const StateVectorInterface<T>& y) {
  assert(x->size() == y.size());
  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const T& xi = x->GetAtIndex(i);
    const T& yi = y.GetAtIndex(i);
    x->SetAtIndex(i, xi + yi);
  }
}

/** Compute `x -= scale * y` with `x` and `y` equal-length state vectors.

@tparam T   A mathematical type compatible with Eigen's Scalar.

If you know that `scale`==1, use the provided `operator-=()` or the simpler 
MinusEq() signature.
@relates drake::systems::StateVectorInterface **/
template <class T>
void MinusEq(StateVectorInterface<T>* x, const T& scale,
             const StateVectorInterface<T>& y) {
  PlusEq(x, -scale, y);
}

/** Compute `x -= y` with `x` and `y` equal-length state vectors. See the other
MinusEq() signature if you want to scale `y`.

Equivalently, use the provided `operator-=()`.
@relates drake::systems::StateVectorInterface **/
template <class T>
void MinusEq(StateVectorInterface<T>* x, 
            const StateVectorInterface<T>& y) {
  assert(x->size() == y.size());
  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const T& xi = x->GetAtIndex(i);
    const T& yi = y.GetAtIndex(i);
    x->SetAtIndex(i, xi - yi);
  }
}

/** Compute `x = scale * x` where `x` is a state vector and `scale` is
a compatible scalar.

@tparam T   A mathematical type compatible with Eigen's Scalar.

Equivalently, use the provided `operator*=()`.
@relates drake::systems::StateVectorInterface **/
template <class T>
void TimesEq(StateVectorInterface<T>* x, const T& scale) {
  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const T& xi = x->GetAtIndex(i);
    x->SetAtIndex(i, scale * xi);
  }
}


/** Compute `x = (1/d) * x` where `x` is a state vector and `d` is a compatible
non-zero scalar. Note that the division is performed just once.

@tparam T   A mathematical type compatible with Eigen's Scalar.

Equivalently, use the provided `operator*=()`.
@relates drake::systems::StateVectorInterface **/
template <class T>
void DivideEq(StateVectorInterface<T>* x, const T& d) {
  const T ood = T(1) / d;
  TimesEq(x, ood);
}

/** Compute `x = x + y`. 
@see drake::systems::PlusEq
@relates drake::systems::StateVectorInterface **/
template <class T>
StateVectorInterface<T>& operator+=(StateVectorInterface<T>& x,
                                    const StateVectorInterface<T>& y) {
    PlusEq(&x, T(1), y);
    return x;
}

/** Compute `x = x - y`. 
@see drake::systems::MinusEq
@relates drake::systems::StateVectorInterface **/
template <class T>
StateVectorInterface<T>& operator-=(StateVectorInterface<T>& x,
                                    const StateVectorInterface<T>& y) {
    MinusEq(&x, T(1), y);
    return x;
}

/** Compute `x = scale * x`. 
@see drake::systems::TimesEq
@relates drake::systems::StateVectorInterface **/
template <class T>
StateVectorInterface<T>& operator*=(StateVectorInterface<T>& x,
                                    const T& scale) {
    TimesEq(&x, scale);
    return x;
}

/** Compute `x = (1/d) * x`. 
@see drake::systems::DivideEq
@relates drake::systems::StateVectorInterface **/
template <class T>
StateVectorInterface<T>& operator/=(StateVectorInterface<T>& x,
                                    const T& divisor) {
    DivideEq(&x, divisor);
    return x;
}

}  // namespace systems
}  // namespace drake
