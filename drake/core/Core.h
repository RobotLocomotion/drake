#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Drake {

#include "Path.h"

// useful refs on generic programming: http://www.generic-programming.org/languages/cpp/techniques.php
//
// Concept: Vector<ScalarType>.
// Requirements:
//  - Defines a static constant (or enum) RowsAtCompileTime (see Pendulum.h for an example).  Can be Eigen::DYNAMIC
//  - implements size_t size(), which equals RowsAtCompileTime except when DYNAMIC
//  - implements the copy constructor and the assignment operator= from const Eigen::MatrixBase<Derived>& (e.g. using "copy and swap")
//  - overload the (non-class) method
//        template <typename ScalarType> Eigen::Matrix<ScalarType,YourVectorType::RowsAtCompileTime,1> toEigen(const YourVectorType& vec);
//    with the appropriate Rows filled in.
// Optional:
//  - overload the (non-class) method
//       template <> const std::string& getCoordinateName<YourVectorType>(const Vector& vec, int index)
// (The methods/defs below make it possible (and hopefully easy) to simply use Eigen::Matrix types to model this concept)

template <typename ScalarType, int Rows> Eigen::Matrix<ScalarType,Rows,1> toEigen(const Eigen::Matrix<ScalarType,Rows,1>& vec) { return vec; }

  // In order to use Eigen types to model the Drake::Vector concept, they must accept only a single template parameter (ScalarType)
  // The following structures provide a means for doing it. (e.g. via Drake::VectorBuilder<Rows>::VecType )

  template <int Rows>
  struct VectorBuilder {
    template <typename ScalarType> using VecType = Eigen::Matrix<ScalarType,Rows,1>;
  };
  template <typename ScalarType> using UnusedVector = Eigen::Matrix<ScalarType,0,1>;
#define EmptyVector UnusedVector


  // Methods for building complicated vectors out of simpler vectors

  template <typename ScalarType, template <typename> class Vector1, template <typename> class Vector2>
  class CombinedVector {
  public:
    const static size_t vec1_rows = Vector1<ScalarType>::RowsAtCompileTime,
                        vec2_rows = Vector2<ScalarType>::RowsAtCompileTime;
    CombinedVector() {};  // allow use of default constructors for vec1 and vec2, also
    CombinedVector(const Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1>& x) : vec1(x.topRows(vec1_rows)), vec2(x.bottomRows(vec2_rows)) {};
    CombinedVector(const Eigen::Matrix<ScalarType,vec1_rows,1>& x1, const Eigen::Matrix<ScalarType,vec2_rows,1>& x2) : vec1(x1), vec2(x2) {};

    CombinedVector& operator=(const Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1>& x) {
      vec1 = x.topRows(vec1_rows);
      vec2 = x.bottomRows(vec2_rows);
      return *this;
    }

    const Vector1<ScalarType>& first(void) const { return vec1; }
    const Vector2<ScalarType>& second(void) const { return vec2; }

    friend std::ostream& operator<<(std::ostream& os, const CombinedVector& x)
    {
      os << x.vec1 << std::endl;
      os << x.vec2 << std::endl;
      return os;
    }

    enum {
      RowsAtCompileTime = vec1_rows + vec2_rows
    };
    std::size_t size() { return vec1.size()+vec2.size(); }

    friend Eigen::Matrix<ScalarType,Vector1<ScalarType>::RowsAtCompileTime + Vector2<ScalarType>::RowsAtCompileTime,1> toEigen(const CombinedVector<ScalarType,Vector1,Vector2>& vec) {
      Eigen::Matrix<ScalarType,Vector1<ScalarType>::RowsAtCompileTime + Vector2<ScalarType>::RowsAtCompileTime,1> x;
      x << toEigen(vec.vec1), toEigen(vec.vec2);
      return x;
    }

  private:
    Vector1<ScalarType> vec1;
    Vector2<ScalarType> vec2;
  };


  template <template <typename> class Vector1, template <typename> class Vector2>
  struct CombinedVectorBuilder {
    template <typename ScalarType> using VecType = CombinedVector<ScalarType,Vector1,Vector2>;
  };

  template <template <typename> class Vector1>
  struct CombinedVectorBuilder<Vector1,UnusedVector> {
    template <typename ScalarType> using VecType = Vector1<ScalarType>;
  };

  template <template <typename> class Vector2>
  struct CombinedVectorBuilder<UnusedVector,Vector2> {
    template <typename ScalarType> using VecType = Vector2<ScalarType>;
  };
}


#endif //DRAKE_DRAKECORE_H
