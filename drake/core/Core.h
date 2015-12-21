#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Drake {
  #include "Path.h"

  /** @defgroup vector_concept Vector<ScalarType>
   * @ingroup concepts
   * @{
   * @brief Describes a (potentially structured) data which can be operated on as a (finite-dimensional) column vector
   *
   * @nbsp
   *
   * | Valid Expressions (which must be implemented) |  |
   * ------------------|-------------------------------------------------------------|
   * | RowsAtCompileTime  | defined as a static constant int (or enum).  Can be Eigen::DYNAMIC. |
   * | size_t size()      | only required if RowsAtCompileTime==Eigen::DYNAMIC |
   * | template<Derived> Vector(const Eigen::MatrixBase<Derived>&)  | constructor taking an Eigen object |
   * | template<Derived> Vector& operator=(const Eigen::MatrixBase<Derived>&)   | assignment operator from an Eigen object |
   * | Eigen::Matrix<ScalarType,RowsAtCompileTime,1> toEigen(const Vector<ScalarType>&) | non-member namespace method which converts to the Eigen type |
   *
   * @nbsp
   *
   * | Valid Expressions (which may be overloaded) |   |
   * |-----------------------|-------------------------|
   * | const std::string& getCoordinateName(const Vector& vec, int index) | returns a textual name for the index coordinate |
   * | Vector<double> getRandomVector() | returns a random vector |
   *
   * @}
   */

  /** EigenVector<Rows>::type<ScalarType>
   * @brief provides an alias for Eigen::Matrix<ScalarType,Rows,1> which is templated on only a single argument (the ScalarType)
   * @concept{vector_concept}
   */
  template <int Rows>
  struct EigenVector {
    template <typename ScalarType> using type = Eigen::Matrix<ScalarType,Rows,1>;
  };

  /** NullVector<ScalarType>
   * @brief provides the empty vector (templated by ScalarType)
   * @concept{vector_concept}
   */
  template <typename ScalarType> using NullVector = Eigen::Matrix<ScalarType,0,1>;

  template <typename ScalarType, int Rows> Eigen::Matrix<ScalarType,Rows,1> toEigen(const Eigen::Matrix<ScalarType,Rows,1>& vec) { return vec; }

  template <template <typename> class VecType>
  VecType<double> getRandomVector(void) {
    static_assert(VecType<double>::RowsAtCompileTime>=0,"still need to handle dynamic case");
    VecType<double> rand(Eigen::Matrix<double,VecType<double>::RowsAtCompileTime,1>::Random());
    return rand;
  }

  namespace internal {
    template<typename VecType, bool Enable = false>
    struct SizeDispatch {
      static std::size_t eval(const VecType &vec) { return VecType::RowsAtCompileTime; }
    };

    template<typename VecType>
    struct SizeDispatch<VecType,true > {
      static std::size_t eval(const VecType &vec) { return vec.size(); }
    };
  }
  template <typename VecType> std::size_t size(const VecType& vec) { return internal::SizeDispatch<VecType,VecType::RowsAtCompileTime==-1>::eval(vec); }



  /** CombinedVector<ScalarType,Vector1,Vector2>
   *
   * @brief produces a new vector type which is the columnwise composition of vector1 and vector2
   */
  template <typename ScalarType, template <typename> class Vector1, template <typename> class Vector2>
  class CombinedVector {
  public:
    const static size_t vec1_rows = Vector1<ScalarType>::RowsAtCompileTime,
                        vec2_rows = Vector2<ScalarType>::RowsAtCompileTime;
    CombinedVector() {};  // allow use of default constructors for vec1 and vec2, also
    CombinedVector(const Vector1<ScalarType>& first, const Vector2<ScalarType>& second) : vec1(first), vec2(second) {};
    template <typename Derived> CombinedVector(const Eigen::MatrixBase<Derived>& x) : vec1(x.topRows(vec1_rows)), vec2(x.bottomRows(vec2_rows)) {};
    template <typename Derived1, typename Derived2> CombinedVector(const Eigen::MatrixBase<Derived1>& x1, const Eigen::MatrixBase<Derived2>& x2) : vec1(x1), vec2(x2) {};

    template <typename Derived>
    CombinedVector& operator=(const Eigen::MatrixBase<Derived>& x) {
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

  /** CombinedVectorBuilder<Vector1,Vector2>::type<ScalarType>
   * @brief Provides an alias to use the CombinedVector class as a model of the Vector concept
   *
   * @concept{vector_concept}
   */

  template <template <typename> class Vector1, template <typename> class Vector2, typename Enable = void>
  struct CombinedVectorBuilder {
    template <typename ScalarType> using type = CombinedVector<ScalarType,Vector1,Vector2>;
  };
/*
  * Uses template aliasing so that combining a vector with the NullVector simply returns the orginal vector type.
  *
  // note: if I decide to leave out this the template aliasing trick, then this type could potentially go right into the CombinedVector class
  template <template <typename> class Vector1, template <typename> class Vector2>
  struct CombinedVectorBuilder<Vector1,Vector2,typename std::enable_if<Vector2<double>::RowsAtCompileTime==0>::type > {
    template <typename ScalarType> using type = Vector1<ScalarType>;
  };

  template <template <typename> class Vector1, template <typename> class Vector2>
  struct CombinedVectorBuilder<Vector1,Vector2,typename std::enable_if<Vector1<double>::RowsAtCompileTime==0>::type > {
    template <typename ScalarType> using type = Vector2<ScalarType>;
  };
*/

  namespace FunctionalForm {
    // these are meant as tags which can be used to inform algorithms about underlying structure in a function (or system)
    // e.g., linear, affine, polynomial, analytic, differentiable, continuous, measurable, and -- lastly -- arbritrary
    struct Arbitrary {};
    struct Polynomial : public Arbitrary {};
    struct Affine : public Polynomial {};
    struct Linear : public Affine {};
  }

}


#endif //DRAKE_DRAKECORE_H
