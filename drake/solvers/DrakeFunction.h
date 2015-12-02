#ifndef DRAKEFUNCTION_H
#define DRAKEFUNCTION_H

#include "DrakeCore.h"

namespace Drake {

// note: uses CRTP
  template <typename Derived, template<typename> class InputVector, template<typename> class OutputVector >
  class Function {
  public:
    virtual OutputVector<double> operator()(const InputVector<double>& x)  { // todo: add const (w/o compile error)?
      return static_cast<Derived*>(this)->eval(x);
    };

    // derived classes must implement, e.g.
    // template <typename ScalarType>
    // OutputVector<ScalarType> eval(const InputVector<ScalarType>& u) const;

    // sparsity (aka input/output dependencies in https://github.com/RussTedrake/drake/blob/drake_function/drake/solvers/DrakeFunction.h)
  };

/*
template <template<typename> class InputVector, template<typename> class OutputVector >
class DifferentiableFunction : public Function {
public:
//  OutputVector<> operator()(InputVector<double> x);  // adds autodiff (size is known from the Vector template argument
};
*/

// class TrigPolyFunction : public DifferentiableFunction
// class PolynomialFunction : public TrigPolyFunction
// class QuadraticFunction : public PolynomialFunction
// class AffineFunction : public QuadraticFunction
// class LinearFunction :  public AffineFunction
// class ConstantFunction : public AffineFunction


}


#endif // DRAKEFUNCTION_H
