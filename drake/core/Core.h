#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "Path.h"

namespace Drake {

  /** @defgroup vector_concept Vector<ScalarType> Concept
   * @ingroup concepts
   * @{
   * @brief Describes a (potentially structured) data which can be operated on as a (finite-dimensional) column vector
   *
   * @nbsp
   *
   * | Valid Expressions (which must be implemented) |  |
   * ------------------|-------------------------------------------------------------|
   * | RowsAtCompileTime  | defined as a static constant int (or enum).  Can be Eigen::Dynamic. |
   * | size_t size()      | only required if RowsAtCompileTime==Eigen::Dynamic |
   * | template<Derived> Vector(const Eigen::MatrixBase<Derived>&)  | constructor taking an Eigen object |
   * | template<Derived> Vector& operator=(const Eigen::MatrixBase<Derived>&)   | assignment operator from an Eigen object |
   * | Eigen::Matrix<ScalarType,RowsAtCompileTime,1> toEigen(const Vector<ScalarType>&) | non-member namespace method which converts to the Eigen type |
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

  /** getRandomVector()
   * @brief Returns a random vector of the desired type using Eigen::Random()
   * @concept{vector_concept}
   */
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
  /** size()
   * @brief Evaluate the size of a Vector
   * @concept{vector_concept}
   *
   * @retval RowsAtCompileTime or the result of size() for dynamically sized vectors
   */
  template <typename VecType> std::size_t size(const VecType& vec) { return internal::SizeDispatch<VecType,VecType::RowsAtCompileTime==-1>::eval(vec); }

  /** getCoordinateName()
   * @brief Implements default coordinate names for a generic vector.  Overload this to name your coordinates.
   * @concept{vector_concept}
   */
  template <typename Vector>
  std::string getCoordinateName(const Vector& vec, unsigned int index) { return "x"+std::to_string(index); }

  /** operator<<()
   * @brief Implements the ostream operator for your vector types.
   * @concept{vector_concept}
   */
/*
  template <typename Vector>  // warning: this will match almost anything!
  std::ostream& operator<<(std::ostream& os, const Vector& vec)
  {
    for (int i=0; i<=size(vec); i++)
      os << getCoordinateName(vec,i) << " = " << vec(i) << std::endl;
    return os;
  }
*/
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

  namespace internal {
    template <template <typename> class Vector1, template <typename> class Vector2, bool Vec2IsNull = false>
    struct CombinedVectorHelper {
      // Vec1 and Vec2 both non-empty
      template <typename ScalarType> using type = CombinedVector<ScalarType,Vector1,Vector2>;
      template <typename ScalarType> static const Vector1<ScalarType>& first(const CombinedVector<ScalarType,Vector1,Vector2>& vec) { return vec.first(); }
      template <typename ScalarType> static const Vector2<ScalarType>& second(const CombinedVector<ScalarType,Vector1,Vector2>& vec) { return vec.second(); }
      template <typename ScalarType> static CombinedVector<ScalarType,Vector1,Vector2> combine(const Vector1<ScalarType>& vec1, const Vector2<ScalarType>& vec2) { return CombinedVector<ScalarType,Vector1,Vector2>(vec1,vec2); };
    };

    template <template <typename> class Vector1, template <typename> class Vector2>
    struct CombinedVectorHelper<Vector1,Vector2,true> {
      // Vec1 non-empty, Vec2 empty
      template <typename ScalarType> using type = Vector1<ScalarType>;
      template <typename ScalarType> static const Vector1<ScalarType>& first(const Vector1<ScalarType>& vec) { return vec; }
      template <typename ScalarType> static NullVector<ScalarType> second(const Vector1<ScalarType>& vec) { return NullVector<ScalarType>(); }
      template <typename ScalarType> static Vector1<ScalarType> combine(const Vector1<ScalarType>& vec1, const Vector2<ScalarType>& vec2) { return vec1; };
    };
  }

  /** CombinedVectorUtil
   * @brief provides logic to build combined vectors and access the first and second elements of the combined vector, handling the \
case when the combined vector builder could have returned the original type
   *
   * Uses template aliasing so that combining a vector with the NullVector simply returns the orginal vector type.
   */
  template <template <typename> class Vector1, template <typename> class Vector2, typename Vec1IsNull = void>
  struct CombinedVectorUtil {
    // Vec1 is non-empty (Vec2 could be either)
    typedef typename internal::template CombinedVectorHelper<Vector1,Vector2,Vector2<double>::RowsAtCompileTime==0> helper;
    template <typename ScalarType> using type = typename helper::template type<ScalarType>;
    template <typename ScalarType> static Vector1<ScalarType> first(const typename helper::template type<ScalarType>& vec ) { return helper::first(vec); }
    template <typename ScalarType> static Vector2<ScalarType> second(const typename helper::template type<ScalarType>& vec ) { return helper::second(vec); }
    template <typename ScalarType> static typename helper::template type<ScalarType> combine(const Vector1<ScalarType>& vec1, const Vector2<ScalarType>& vec2) { return helper::combine(vec1,vec2); }
  };
  template <template <typename> class Vector1, template <typename> class Vector2>
  struct CombinedVectorUtil<Vector1,Vector2,typename std::enable_if<Vector1<double>::RowsAtCompileTime==0>::type> {
    // Vec1 is empty (Vec2 could be either)
    template <typename ScalarType> using type = Vector2<ScalarType>;
    template <typename ScalarType> static NullVector<ScalarType> first(const Vector2<ScalarType>& vec) { return NullVector<ScalarType>(); }
    template <typename ScalarType> static const Vector2<ScalarType>& second(const Vector2<ScalarType>& vec) { return vec; }
    template <typename ScalarType> static Vector2<ScalarType> combine(const Vector1<ScalarType>& vec1, const Vector2<ScalarType>& vec2) { return vec2; };
  };

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
