#ifndef GRADIENTVAR_H_
#define GRADIENTVAR_H_

#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <exception>
//#include <Core/GeneralProduct.h>

template <typename Derived>
class GradientVar
{
  typedef typename Gradient<Derived, Eigen::Dynamic>::type GradientMatrixType;

private:
  Derived mat;
  GradientMatrixType gradient;

public:
  GradientVar(const Derived& mat, const GradientMatrixType& gradient) :
    mat(mat), gradient(gradient){
    // empty
  }

  GradientVar(int numVars, int rows = Derived::RowsAtCompileTime, int cols = Derived::ColsAtCompileTime) :
    mat(rows, cols), gradient(mat.size(), numVars) {
    // empty
  }

  virtual ~GradientVar()
  {
    // empty
  }

  template <typename OtherDerived>
  GradientVar<Derived>& operator=(const GradientVar<OtherDerived>& other) {
    mat = other.val();
    gradient = other.getGradient();
    return *this;
  }

  template<typename OtherDerived>
  const GradientVar<typename Eigen::ProductReturnType<Derived,OtherDerived>::Type>
  operator*(const GradientVar<OtherDerived> &other) const {
    return GradientVar<typename Eigen::ProductReturnType<Derived,OtherDerived>::Type>(val() * other.val(), matGradMultMat(val(), other.val(), getGradient(), other.getGradient()));
  }

  template<typename OtherDerived>
  const GradientVar< typename Eigen::CwiseBinaryOp< Eigen::internal::scalar_sum_op <typename Derived::Scalar>, const Derived, const OtherDerived> > operator+ ( const GradientVar< OtherDerived >& other) const {
    return GradientVar< typename Eigen::CwiseBinaryOp< Eigen::internal::scalar_sum_op <typename Derived::Scalar>, const Derived, const OtherDerived> >(val() + other.val(), getGradient() + other.getGradient());
  }

  GradientVar<Eigen::Transpose<Derived>> transpose() {
    return GradientVar<Eigen::Transpose<Derived>>(mat.transpose(), transposeGrad(gradient, mat.rows()));
  }

  const Derived& val() const
  {
    return mat;
  }

  Derived& val()
  {
    return mat;
  }

  int getNumVars() const
  {
    return gradient.cols();
  }

  const GradientMatrixType& getGradient() const
  {
    return gradient;
  }

  GradientMatrixType& getGradient()
  {
    return gradient;
  }
};


#endif /* GRADIENTVAR_H_ */
