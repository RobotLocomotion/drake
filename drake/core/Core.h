#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include "drake/Path.h"

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

  /**
   * @brief whether or not the given type is an Eigen column vector
   */
  template <typename StateVector>
  struct is_eigen_vector : public std::false_type {};

  template <typename Scalar, int Rows, int Options, int MaxRows>
  struct is_eigen_vector<Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, 1>> : public std::true_type {};

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
    template<typename VecType, class Enable = void>
    struct SizeDispatch {
      static std::size_t eval(const VecType &vec) { return VecType::RowsAtCompileTime; }
    };

    template<typename VecType>
    struct SizeDispatch<VecType, typename std::enable_if<VecType::RowsAtCompileTime == Eigen::Dynamic>::type > {
      static std::size_t eval(const VecType &vec) { return vec.size(); }
    };
  }
  /** size()
   * @brief Evaluate the size of a Vector
   * @concept{vector_concept}
   *
   * @retval RowsAtCompileTime or the result of size() for dynamically sized vectors
   */
  template <typename VecType> std::size_t size(const VecType& vec) { return internal::SizeDispatch<VecType>::eval(vec); }

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
    CombinedVector() {};  // allow use of default constructors for vec1 and vec2, also
    CombinedVector(const Vector1<ScalarType>& first, const Vector2<ScalarType>& second) : vec1(first), vec2(second) {};

    template <typename Derived> CombinedVector(const Eigen::MatrixBase<Derived>& x) : vec1(x.topRows(Vector1<ScalarType>::RowsAtCompileTime)), vec2(x.bottomRows(Vector2<ScalarType>::RowsAtCompileTime)) {
      static_assert(RowsAtCompileTime != Eigen::Dynamic, "Cannot determine sizes of subvectors because sizes are not known at compile time."); // TODO: could handle cases where only one of the subvectors has dynamic size
    };

    template <typename Derived1, typename Derived2> CombinedVector(const Eigen::MatrixBase<Derived1>& x1, const Eigen::MatrixBase<Derived2>& x2) : vec1(x1), vec2(x2) {};

    template <typename Derived>
    CombinedVector& operator=(const Eigen::MatrixBase<Derived>& x) {
      vec1 = x.topRows(Drake::size(vec1));
      vec2 = x.bottomRows(Drake::size(vec2));
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
      RowsAtCompileTime = (Vector1<ScalarType>::RowsAtCompileTime == Eigen::Dynamic || Vector2<ScalarType>::RowsAtCompileTime == Eigen::Dynamic) ? Eigen::Dynamic : Vector1<ScalarType>::RowsAtCompileTime + Vector2<ScalarType>::RowsAtCompileTime
    };

    std::size_t size() const {
      return Drake::size(vec1) + Drake::size(vec2);
    }

    friend Eigen::Matrix<ScalarType,RowsAtCompileTime,1> toEigen(const CombinedVector<ScalarType,Vector1,Vector2>& vec) {
      Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> x(vec.size());
      x << toEigen(vec.vec1), toEigen(vec.vec2);
      return x;
    }

  private:
    Vector1<ScalarType> vec1;
    Vector2<ScalarType> vec2;
  };

  /**
 * @brief whether or not the given type is a CombinedVector
 */
  template <typename StateVector>
  struct is_combined_vector : public std::false_type {};

  template <typename Scalar, template <typename> class Vector1, template <typename> class Vector2>
  struct is_combined_vector<CombinedVector<Scalar, Vector1, Vector2> > : public std::true_type {};

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

  /** InputOutputRelation
   * \brief Tags which can be used to inform algorithms about underlying structure in a function
   * e.g., linear, affine, polynomial, analytic, differentiable, continuous, measurable, and -- lastly -- arbitrary
   *
   * note: i considered using inheritance to capture the relationship, but passing around types at runtime
   * was more of a pain than simply capturing the inheritance with the helper functions below.
   */
  struct InputOutputRelation {
    enum Form {
      ARBITRARY = 0,
      POLYNOMIAL = 1,
      AFFINE = 2,
      LINEAR = 3,
      CONSTANT = 4,
      ZERO = 5
      // todo: add more (rational, ...)
    };
    Form form;
    // todo: add sparsity info

    InputOutputRelation(Form f) : form(f) {};

    static bool isa(const Form& f, const Form& base) {
      if (f==base || base == ARBITRARY) return true;
      if (f == ARBITRARY) return false;
      return isa(derivesFrom(f), base);
    }
    bool isa(Form base) { return isa(form,base); }

    static Form leastCommonAncestor(const Form& f1, const Form& f2) {
      if (f1==ARBITRARY || f2==ARBITRARY) return ARBITRARY;
      if (isa(f2,f1)) return f1;
      return leastCommonAncestor(derivesFrom(f1),f2);
    }
    static Form leastCommonAncestor(std::initializer_list<Form> forms) {
      if (forms.size()<1) throw std::runtime_error("leastCommonAncestor requires at least one argument");
      if (forms.size()==1) return *forms.begin();

      std::initializer_list<Form>::const_iterator iter = forms.begin();
      Form ret = *iter;
      while (++iter != forms.end()) {
        ret = leastCommonAncestor(ret,*iter);
      }
      return ret;
    }

    static InputOutputRelation composeWith(const InputOutputRelation& g, const InputOutputRelation& f) { // composition of functions y = g(f(x))
      return InputOutputRelation(leastCommonAncestor(g.form,f.form));
    }
    /*
    static InputOutputRelation linearCombination(const InputOutputRelation& g, const InputOutputRelation& f) {
      return InputOutputRelation(leastCommonAncestor({g.form,f.form,LINEAR}));
    }*/
    static InputOutputRelation combine(const InputOutputRelation& a, const InputOutputRelation& b) { // vertical concatenation
      return InputOutputRelation(leastCommonAncestor(a.form,b.form));
    }
    static InputOutputRelation combine(std::initializer_list<InputOutputRelation> args) {
      if (args.size()<1) throw std::runtime_error("combine requires at least one argument");
      if (args.size()==1) return *args.begin();

      std::initializer_list<InputOutputRelation>::const_iterator iter = args.begin();
      InputOutputRelation ret = *iter;
      while (++iter != args.end()) {
        ret = combine(ret,*iter);
      }
      return ret;
    }

  private:
    static Form derivesFrom(const Form& f) { // capture the inheritance relationships (without resorting to using types)
      switch (f) {
        case POLYNOMIAL: return ARBITRARY;
        case AFFINE: return POLYNOMIAL;
        case LINEAR: return AFFINE;
        case CONSTANT: return AFFINE;
        case ZERO: return LINEAR; // note: really want multiple inheritance here, since it's also constant
        default: return ARBITRARY;
      }
    }
  };





}


#endif //DRAKE_DRAKECORE_H
