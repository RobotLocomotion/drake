#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <array>
#include <assert.h>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeGradientUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

template<std::size_t Size>
std::array<int, Size> intRange(int start)
{
  std::array<int, Size> ret;
  for (unsigned int i = 0; i < Size; i++) {
    ret[i] = i + start;
  }
  return ret;
}

/*
 * Recursively defined template specifying a matrix type of the correct size for a gradient of a matrix function with respect to Nq variables, of any order.
 */
template<typename Derived, int Nq, int DerivativeOrder = 1>
struct Gradient {
  typedef typename Eigen::Matrix<typename Derived::Scalar, ((Derived::SizeAtCompileTime == Eigen::Dynamic || Nq == Eigen::Dynamic) ? Eigen::Dynamic : Gradient<Derived, Nq, DerivativeOrder - 1>::type::SizeAtCompileTime), Nq> type;
};

/*
 * Base case for recursively defined gradient template.
 */
template<typename Derived, int Nq>
struct Gradient<Derived, Nq, 1> {
  typedef typename Eigen::Matrix<typename Derived::Scalar, Derived::SizeAtCompileTime, Nq> type;
};

/*
 * Output type of matGradMultMat
 */
template<typename DerivedA, typename DerivedB, typename DerivedDA>
struct MatGradMultMat {
  typedef typename Eigen::Matrix<typename DerivedA::Scalar, (DerivedA::RowsAtCompileTime == Eigen::Dynamic || DerivedB::ColsAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : DerivedA::RowsAtCompileTime * DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime> type;
};

/*
 * Output type of matGradMult
 */
template<typename DerivedDA, typename DerivedB>
struct MatGradMult {
  typedef typename Eigen::Matrix<typename DerivedDA::Scalar, (DerivedDA::RowsAtCompileTime == Eigen::Dynamic || DerivedB::SizeAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : DerivedDA::RowsAtCompileTime / DerivedB::RowsAtCompileTime * DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime> type;
};

/*
 * Output type and array types of getSubMatrixGradient for std::arrays specifying rows and columns
 */
template<int QSubvectorSize, typename Derived, std::size_t NRows, std::size_t NCols>
struct GetSubMatrixGradientArray {
  typedef typename Eigen::Matrix<typename Derived::Scalar, (NRows * NCols), ((QSubvectorSize == Eigen::Dynamic) ? Derived::ColsAtCompileTime : QSubvectorSize)> type;
};

/*
 * Output type of getSubMatrixGradient for a single element of the input matrix
 */
template<int QSubvectorSize, typename Derived>
struct GetSubMatrixGradientSingleElement {
  typedef typename Eigen::Block<const Derived, 1, ((QSubvectorSize == Eigen::Dynamic) ? Derived::ColsAtCompileTime : QSubvectorSize)> type;
};
 
/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
DLLEXPORT typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, typename Derived::Index rows_X);

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
DLLEXPORT typename MatGradMultMat<DerivedA, DerivedB, DerivedDA>::type
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB);


template<typename DerivedDA, typename DerivedB>
DLLEXPORT typename MatGradMult<DerivedDA, DerivedB>::type
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<DerivedB>& b);

// TODO: could save copies once http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::vector<int>& rows, const std::vector<int>& cols,
    typename Derived::Index M_rows, int q_start = 0, typename Derived::Index q_subvector_size = -1);

template<int QSubvectorSize, typename Derived, std::size_t NRows, std::size_t NCols>
DLLEXPORT typename GetSubMatrixGradientArray<QSubvectorSize, Derived, NRows, NCols>::type
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM,
  const std::array<int, NRows>& rows,
  const std::array<int, NCols>& cols,
  typename Derived::Index M_rows, int q_start = 0, typename Derived::Index q_subvector_size = QSubvectorSize);

template<int QSubvectorSize, typename Derived>
DLLEXPORT typename GetSubMatrixGradientSingleElement<QSubvectorSize, Derived>::type
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, int row, int col, typename Derived::Index M_rows,
    typename Derived::Index q_start = 0, typename Derived::Index q_subvector_size = QSubvectorSize);

template<typename DerivedA, typename DerivedB>
DLLEXPORT void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start = 0, typename DerivedA::Index q_subvector_size = -1);

template<int QSubvectorSize, typename DerivedA, typename DerivedB, std::size_t NRows, std::size_t NCols>
DLLEXPORT void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start = 0, typename DerivedA::Index q_subvector_size = QSubvectorSize);

template<int QSubvectorSize, typename DerivedDM, typename DerivedDMSub>
DLLEXPORT void setSubMatrixGradient(Eigen::MatrixBase<DerivedDM>& dM, const Eigen::MatrixBase<DerivedDMSub>& dM_submatrix, int row, int col, typename DerivedDM::Index M_rows,
    typename DerivedDM::Index q_start = 0, typename DerivedDM::Index q_subvector_size = QSubvectorSize);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> ret(auto_diff_matrix.rows(), auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); i++) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      ret(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return ret;
};

template<typename Derived>
typename Gradient<Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>, Eigen::Dynamic>::type autoDiffToGradientMatrix(
        const Eigen::MatrixBase<Derived>& auto_diff_matrix, int num_variables = Eigen::Dynamic)
{
  int num_variables_from_matrix = 0;
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    num_variables_from_matrix = std::max(num_variables_from_matrix, static_cast<int>(auto_diff_matrix(i).derivatives().size()));
  }
  if (num_variables == Eigen::Dynamic) {
    num_variables = num_variables_from_matrix;
  }
  else if (num_variables_from_matrix != 0 && num_variables_from_matrix != num_variables) {
    std::stringstream buf;
    buf << "Input matrix has derivatives w.r.t " << num_variables_from_matrix << " variables, whereas num_variables is " << num_variables << ".\n";
    buf << "Either num_variables_from_matrix should be zero, or it should match num_variables.";
    throw std::runtime_error(buf.str());
  }

  typename Gradient<Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>, Eigen::Dynamic>::type gradient(auto_diff_matrix.size(), num_variables);
  for (int row = 0; row < auto_diff_matrix.rows(); row++) {
    for (int col = 0; col < auto_diff_matrix.cols(); col++) {
      auto gradient_row = gradient.row(row + col * auto_diff_matrix.rows()).transpose();
      if (auto_diff_matrix(row, col).derivatives().size() == 0) {
        gradient_row.setZero();
      } else {
        gradient_row = auto_diff_matrix(row, col).derivatives();
      }
    }
  }
  return gradient;
}

template<typename DerivedGradient, typename DerivedAutoDiff>
void gradientMatrixToAutoDiff(const Eigen::MatrixBase<DerivedGradient>& gradient, Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix)
{
  int nx = gradient.cols();
  for (size_t row = 0; row < auto_diff_matrix.rows(); row++) {
    for (size_t col = 0; col < auto_diff_matrix.cols(); col++) {
      auto_diff_matrix(row, col).derivatives().resize(nx, 1);
      auto_diff_matrix(row, col).derivatives() = gradient.row(row + col * auto_diff_matrix.rows()).transpose();
    }
  }
}

#endif /* DRAKEGRADIENTUTIL_H_ */
