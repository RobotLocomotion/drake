#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Drake
{

// useful refs on generic programming: http://www.generic-programming.org/languages/cpp/techniques.php
//
// Concept: Vector<ScalarType>.
// Requirements:
//  - has a static const size_t size_at_compile (with -1 for DYNAMIC)
//  - implements size_t size(), which equals size_at_compile except when DYNAMIC
//  - implements the copy constructor from Eigen::Matrix<ScalarType,size_at_compile,1>
//  - implements the typecast method operator Eigen::Matrix<ScalarType,size_at_compile,1>()
//  - implements std::ostream& operator<<(std::ostream& os, const Vector& x)
// (Note that the Eigen::Vector_d classes do all of these (by

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


#endif //DRAKE_DRAKECORE_H
