#ifndef DRAKE_UTIL_GRADIENTVAR_H_
#define DRAKE_UTIL_GRADIENTVAR_H_

#include <stdexcept>
#include <memory>
#include <Eigen/Core>
#include "drakeGradientUtil.h"
#include <iostream>


template <typename Scalar, int Rows, int Cols>
class GradientVar
{
public:
  typedef typename Eigen::Matrix<Scalar, Rows, Cols> DataType;
  typedef typename Gradient<DataType, Eigen::Dynamic>::type GradientDataType;
  typedef GradientVar<Scalar, GradientDataType::RowsAtCompileTime, GradientDataType::ColsAtCompileTime> ChildGradientVarType;

private:
  DataType val;
  std::unique_ptr<ChildGradientVarType> grad;

private:
  GradientVar& operator= (const GradientVar& other); // disallow assignment
//  GradientVar(const GradientVar& other);

public:
  GradientVar(int rows, int cols, int nq = 0, int order = 0) :
    val(rows, cols),
    grad(order > 0 ? new ChildGradientVarType(rows * cols, nq, nq, order - 1) : nullptr)
  {
//    std::cout << "rows: " << rows << ", cols: " << cols << std::endl;
    // empty
  }

  // copy constructor
  GradientVar(const GradientVar& other) :
     val(other.val),
     grad(other.grad == nullptr ? nullptr : new ChildGradientVarType(*other.grad))
  {
//    std::cout << "copy constructor called" << std::endl;
    // empty
  }

  virtual ~GradientVar()
  {
  }

  DataType& value()
  {
    return val;
  }

  const DataType& value() const
  {
    return val;
  }

  bool hasGradient() const
  {
    return grad.operator bool();
  }

  ChildGradientVarType& gradient()
  {
    if (hasGradient()) {
      return *grad;
    }
    else {
      throw std::runtime_error("gradient not available");
    }
  }

  const ChildGradientVarType& gradient() const
  {
    if (hasGradient()) {
      return *grad;
    }
    else {
      throw std::runtime_error("gradient not available");
    }
  }

  int maxOrder() const
  {
    return hasGradient() ? 1 + grad->maxOrder() : 0;
  }

  int getNumVariables() const
  {
    return hasGradient() ? grad->value().cols() : -1;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar, DataType::SizeAtCompileTime)
};

#endif /* DRAKE_UTIL_GRADIENTVAR_H_ */
