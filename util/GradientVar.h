#ifndef GRADIENTVAR_H_
#define GRADIENTVAR_H_

#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <exception>

template <typename MatrixType>
class GradientVar
{
  typedef typename Gradient<MatrixType, Eigen::Dynamic>::type GradientMatrixType;

private:
  MatrixType mat;
  const int numVars;
  std::unique_ptr<GradientVar<GradientMatrixType>> gradient;


public:
  GradientVar(int numVars, int order = 0, int rows = MatrixType::RowsAtCompileTime, int cols = MatrixType::ColsAtCompileTime) :
    mat(rows, cols), numVars(numVars), gradient(order > 0 ? new GradientVar<GradientMatrixType>(numVars, order - 1, rows * cols, numVars) : nullptr) {
    // empty
  }

  virtual ~GradientVar() {
    // empty
  }

  const GradientVar<GradientMatrixType>& getGradient() const
  {
    if (gradient) {
      return *gradient;
    }
    else {
      throw std::runtime_error("This GradientVar has no gradient");
    }
  }

  GradientVar<GradientMatrixType>& getGradient()
  {
    if (gradient) {
      return *gradient;
    }
    else {
      throw std::runtime_error("This GradientVar has no gradient");
    }
  }

  const MatrixType& val() const
  {
    return mat;
  }

  MatrixType& val()
  {
    return mat;
  }

  const int getNumVars() const
  {
    return numVars;
  }
};


#endif /* GRADIENTVAR_H_ */
