#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Drake
{

/***
 * A vector who's type defines the coordinate system.
 * Can have named coordinates, but must also provide it's value as an Eigen::Vector
 */

template <typename ScalarType, int Dim = Eigen::Dynamic>
class Vector {
public:
  typedef Eigen::Matrix<ScalarType,Dim,1> EigenType;
  virtual EigenType eigen() const = 0;

  virtual unsigned int size() const {
//    static_assert(Dim>=0,"CoordinateSystem classes with dynamic (runtime) dimension specification must overload size()");
    return static_cast<int>(Dim);
  };
  virtual std::string getCoordinateName(unsigned int i) const { return "x"+std::to_string(i); };
  virtual std::vector<std::string> getCoordinateNames() const {
    std::vector<std::string> coordinates;
    for (int i=0; i<size(); i++) {
      coordinates.push_back(getCoordinateName(i));
    }
    return coordinates;
  };

  virtual std::ostream& print(std::ostream& os) const {
    auto val = eigen();
    for (int i=0; i<size(); i++) {
      os << "  " << getCoordinateName(i) << " = " << val(i) << std::endl;
    }
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const Vector& x)
  {
    return x.print(os);
  }
};

// useful refs on generic programming: http://www.generic-programming.org/languages/cpp/techniques.php

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

  // sparsity
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
