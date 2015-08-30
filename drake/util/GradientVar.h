#ifndef DRAKE_UTIL_GRADIENTVAR_H_
#define DRAKE_UTIL_GRADIENTVAR_H_

#include <stdexcept>
#include <Eigen/Core>
#include "drakeGradientUtil.h"
#include <iostream>
#include <cstdint>


template <typename Scalar, int Rows, int Cols, int MaxRows = Rows, int MaxCols = Cols>
class GradientVar
{
public:
  typedef typename Eigen::Matrix<Scalar, Rows, Cols, 0, MaxRows, MaxCols> DataType;
  typedef typename Gradient<DataType, Eigen::Dynamic>::type GradientDataType;
  typedef GradientVar<Scalar, GradientDataType::RowsAtCompileTime, GradientDataType::ColsAtCompileTime> ChildGradientVarType;

private:
  DataType val;
  ChildGradientVarType* grad;

private:

public:
  GradientVar(typename DataType::Index rows, typename DataType::Index cols, typename DataType::Index nq = 0, int order = 0) :
    val(rows, cols),
    grad(order > 0 ? new ChildGradientVarType(rows * cols, nq, nq, order - 1) : nullptr)
  {
  }

  // copy constructor
  GradientVar(const GradientVar& other) :
     val(other.val),
     grad(other.grad == nullptr ? nullptr : new ChildGradientVarType(*other.grad))
  {
  }

  GradientVar& operator= (const GradientVar& other) {
    // Check for self-assignment
    if (this == &other)
      return *this;

    val = other.val;

    if (hasGradient() != other.hasGradient())
      throw std::runtime_error("Refusing to assign this GradientVar based on a right hand side that does not have the same gradient order to avoid large, hidden allocations.");
    else
      *grad = *(other.grad); // perform recursive deep copy

    return *this;
  }

  virtual ~GradientVar()
  {
    if (hasGradient())
      delete grad;
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
    return grad != nullptr;
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

  typename DataType::Index getNumVariables() const
  {
    return hasGradient() ? grad->value().cols() : -1;
  }

  void resize(int rows, int cols, int nq = 0, int order = 0)
  {
    val.resize(rows, cols);
    if (order > 0) {
      if (hasGradient()) {
        grad->resize(rows * cols, nq, nq, order - 1);
      }
      else {
        grad = new ChildGradientVarType(rows * cols, nq, nq, order - 1);
      }
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(bool(((DataType::SizeAtCompileTime)!=Eigen::Dynamic) && ((static_cast<int>(sizeof(Scalar))*(DataType::SizeAtCompileTime))%16==0)))
};

#endif /* DRAKE_UTIL_GRADIENTVAR_H_ */
