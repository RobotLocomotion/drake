#ifndef GRADIENTVAR_H_
#define GRADIENTVAR_H_

#include "drakeGradientUtil.h"
#include <Eigen/Core>
#include <exception>
//#include <Core/GeneralProduct.h>

template <typename Derived>
class GradientVar
{
public:
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

  template<int BlockRows, int BlockCols>
  inline const GradientVar<const Eigen::Block<const Derived, BlockRows, BlockCols>> getBlock(int startRow, int startCol) const
  {
    std::array<int, BlockRows> rows;
    for (int i = 0; i < BlockRows; ++i) {
      rows[i] = startRow + i;
    }

    std::array<int, BlockCols> cols;
    for (int i = 0; i < BlockCols; ++i) {
      cols[i] = startCol + i;
    }

    return GradientVar<const Eigen::Block<const Derived, BlockRows, BlockCols>>(mat.template block<BlockRows, BlockCols>(startRow, startCol), getSubMatrixGradient(gradient, rows, cols, mat.rows()));
  }

  inline const GradientVar<const Eigen::Block<const Derived>> getBlock(int startRow, int startCol, int blockRows, int blockCols) const
  {
    std::vector<int> rows(blockRows);
    for (int i = 0; i < blockRows; ++i) {
      rows.push_back(startRow + i);
    }

    std::vector<int> cols(blockCols);
    for (int i = 0; i < blockCols; ++i) {
      cols.push_back(startCol + i);
    }

    return GradientVar<const Eigen::Block<const Derived>>(mat.block(startRow, startCol, blockRows, blockCols), getSubMatrixGradient(gradient, rows, cols, mat.rows()));
  }

  template<int BlockRows, int BlockCols, typename OtherDerived>
  void setBlock(const GradientVar<OtherDerived>& block, int start_row, int start_col, int block_rows = BlockRows, int block_cols = BlockCols)
  {
    std::array<int, BlockRows> rows;
    for (int i = 0; i < BlockRows; ++i) {
      rows[i] = start_row + i;
    }

    std::array<int, BlockCols> cols;
    for (int i = 0; i < BlockCols; ++i) {
      cols[i] = start_col + i;
    }

    mat.template block<BlockRows, BlockCols>(start_row, start_col) = block.val();
    setSubMatrixGradient(gradient, block.getGradient(), rows, cols, rows());
  }

  template<typename OtherDerived>
  void setBlock(const GradientVar<OtherDerived>& block, int start_row, int start_col, int block_rows, int block_cols)
  {
    std::vector<int> rows(block_rows);
    for (int i = 0; i < block_rows; ++i) {
      rows.push_back(start_row + i);
    }

    std::vector<int> cols(block_cols);
    for (int i = 0; i < block_cols; ++i) {
      cols.push_back(start_col + i);
    }

    mat.block(start_row, start_col, block_rows, block_cols) = block.val();
    setSubMatrixGradient(gradient, block.getGradient(), rows, cols, this->rows());
  }

  const Derived& val() const
  {
    return mat;
  }

  Derived& val()
  {
    return mat;
  }

  int cols() const
  {
    return mat.cols();
  }

  int rows() const
  {
    return mat.rows();
  }

  int size() const
  {
    return mat.size();
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
