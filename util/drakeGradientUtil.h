#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <array>
#include <assert.h>

/* Turn 'default template arguments are only allowed on a class template' error (C4519) into a warning
   According to Stroustrup:
   The prohibition of default template arguments for function templates is a misbegotten remnant of the
   time where freestanding functions were treated as second class citizens and required all template 
   arguments to be deduced from the function arguments rather than specified.
*/
#if defined(_WIN32) || defined(_WIN64)
#pragma warning(1 : 4519)
#endif

template<unsigned int Size>
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
  typedef typename Eigen::Matrix<typename Derived::Scalar, ((Derived::SizeAtCompileTime == Eigen::Dynamic || Nq == Eigen::Dynamic) ? Eigen::Dynamic : Gradient<typename Derived, Nq, DerivativeOrder - 1>::template type::SizeAtCompileTime), Nq> type;
};

/*
 * Base case for recursively defined gradient template.
 */
template<typename Derived, int Nq>
struct Gradient<Derived, Nq, 1> {
  typedef typename Eigen::Matrix<typename Derived::Scalar, Derived::SizeAtCompileTime, Nq> type;
};

#define matGradMultMatNumRows(rows_A, cols_B) ((((rows_A) == Eigen::Dynamic) || ((cols_B) == Eigen::Dynamic)) ? Eigen::Dynamic : ((rows_A) * (cols_B)))

// #define matGradMultNumRows(rows_dA, rows_b) ((((rows_dA) == Eigen::Dynamic) || ((rows_b) == Eigen::Dynamic)) ? Eigen::Dynamic : ((rows_dA) / (rows_b)))
template<typename DerivedDA, typename Derivedb>
struct MatGradMult {
  typedef typename Eigen::Matrix<typename DerivedDA::Scalar, (DerivedDA::RowsAtCompileTime == Eigen::Dynamic || Derivedb::RowsAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : DerivedDA::RowsAtCompileTime / Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime> type;
};

#define getRowBlockGradientNumRows(M_cols, block_rows) (((M_cols) == Eigen::Dynamic) ? Eigen::Dynamic : ((M_cols) * (block_rows)))

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, int rows_X);

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
Eigen::Matrix<typename DerivedA::Scalar, matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB);


template<typename DerivedDA, typename Derivedb>
typename MatGradMult<DerivedDA, Derivedb>::type
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<Derivedb>& b);

// TODO: could save copies once http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::vector<int>& rows, const std::vector<int>& cols,
    int M_rows, int q_start = 0, int q_subvector_size = -1);

template<int QSubvectorSize = -1, typename Derived, std::size_t NRows, std::size_t NCols>
Eigen::Matrix<typename Derived::Scalar, (int) (NRows * NCols), (QSubvectorSize == Eigen::Dynamic ? Derived::ColsAtCompileTime : QSubvectorSize)>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows, int q_start = 0, int q_subvector_size = QSubvectorSize);

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 1, Derived::ColsAtCompileTime>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, int row, int col, int M_rows);

template<typename DerivedA, typename DerivedB>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, int M_rows, int q_start = 0, int q_subvector_size = -1);

template<int QSubvectorSize = -1, typename DerivedA, typename DerivedB, std::size_t NRows, std::size_t NCols>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows, int q_start = 0, int q_subvector_size = QSubvectorSize);

#endif /* DRAKEGRADIENTUTIL_H_ */
