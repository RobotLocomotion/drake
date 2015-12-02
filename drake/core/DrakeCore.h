#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Drake {

// useful refs on generic programming: http://www.generic-programming.org/languages/cpp/techniques.php
//
// Concept: Vector<ScalarType>.
// Requirements:
//  - Defines a VectorTraits<VectorType> with a static constant (or enum) RowsAtCompileTime (see Pendulum.h for an example).  Can be Eigen::DYNAMIC
//  - implements size_t size(), which equals size_at_compile except when DYNAMIC
//  - implements the copy constructor from Eigen::Matrix<ScalarType,size_at_compile,1>
//  - implements the typecast method operator Eigen::Matrix<ScalarType,size_at_compile,1>()
//  - implements std::ostream& operator<<(std::ostream& os, const Vector& x)
// (The methods/defs below make it possible (and hopefully easy) to simply use Eigen::Matrix types to model this concept)

  template<typename T>
  struct VectorTraits;

  template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  struct VectorTraits<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> > {
    enum {
      RowsAtCompileTime = _Rows
    };
  };

// really want GenericVector<Rows> to result in a template <typename ScalarType> Eigen::Matrix<ScalarType,Rows,1>
#define GenericVector(Rows) template<typename ScalarType> using GenericVector##Rows = Eigen::Matrix<ScalarType,Rows,1>

  template <typename ScalarType> using EmptyVector = Eigen::Matrix<ScalarType,0,1>;
  GenericVector(0);
  GenericVector(1);
  GenericVector(2);
  GenericVector(3);
  GenericVector(4);

}

#endif //DRAKE_DRAKECORE_H
