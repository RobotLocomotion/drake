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
//  - implements the copy constructor and the assignment operator= from Eigen::Matrix<ScalarType,size_at_compile,1> (e.g. using "copy and swap")
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

  // In order to use Eigen types to model the Drake::Vector concept, they must accept only a single template parameter (ScalarType)
  // The following structures provide a means for doing it.

  template <int Rows>
  struct VectorBuilder {
    template <typename ScalarType> using VecType = Eigen::Matrix<ScalarType,Rows,1>;
  };
  template <typename ScalarType> using EmptyVector = Eigen::Matrix<ScalarType,0,1>;
  template <typename ScalarType> using UnusedVector = Eigen::Matrix<ScalarType,0,1>;


  template <typename ScalarType, template <typename> class Vector1, template <typename> class Vector2>
  class CombinedVector {
  public:
    const static size_t vec1_rows = VectorTraits<Vector1<ScalarType> >::RowsAtCompileTime,
                        vec2_rows = VectorTraits<Vector2<ScalarType> >::RowsAtCompileTime;
    CombinedVector() {};  // allow use of default constructors for vec1 and vec2, also
    CombinedVector(const Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1>& x) : vec1(x.topRows(vec1_rows)), vec2(x.bottomRows(vec2_rows)) {};

    CombinedVector& operator=(const Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1>& x) {
      vec1 = x.topRows(vec1_rows);
      vec2 = x.bottomRows(vec2_rows);
      return *this;
    }

    operator Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1> () const {
      Eigen::Matrix<ScalarType,vec1_rows + vec2_rows,1> x;
      x << vec1, vec2;
      return x;
    }

    friend std::ostream& operator<<(std::ostream& os, const CombinedVector& x)
    {
      os << x.vec1 << std::endl;
      os << x.vec2 << std::endl;
      return os;
    }

    std::size_t size() { return vec1.size()+vec2.size(); }

    Vector1<ScalarType> vec1;
    Vector2<ScalarType> vec2;
  };

  template <template <typename> class Vector1, template <typename> class Vector2>
  struct CombinedVectorBuilder {
    template <typename ScalarType> using VecType = CombinedVector<ScalarType,Vector1,Vector2>;
  };


  // a few tools/tricks from Modern C++ Design (by Alexandrescu)

  template <int v>
  struct Int2Type
  {
    enum { value = v };
  };

}


#endif //DRAKE_DRAKECORE_H
