#include "drakeGradientUtil.h"
#include <cassert>


/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, int rows_X)
{
  typename Derived::PlainObject dX_transpose(dX.rows(), dX.cols());
  int numel = dX.rows();
  int index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
  return dX_transpose;
}

template <typename DerivedA, typename DerivedB, typename DerivedDA, typename DerivedDB>
Eigen::Matrix<typename DerivedA::Scalar, matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB) {
  assert(dA.cols() == dB.cols());
  const int nq = dA.cols();
  const int nq_at_compile_time = DerivedDA::ColsAtCompileTime;

  Eigen::Matrix<typename DerivedA::Scalar,
  matGradMultMatNumRows(DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime),
  DerivedDA::ColsAtCompileTime> ret(A.rows() * B.cols(), nq);

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<DerivedA::RowsAtCompileTime, nq_at_compile_time>(col * A.rows(), 0, A.rows(), nq);

    // A * dB part:
    block.noalias() = A * dB.template block<DerivedA::ColsAtCompileTime, nq_at_compile_time>(col * A.cols(), 0, A.cols(), nq);

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) * dA.template block<DerivedA::RowsAtCompileTime, nq_at_compile_time>(row * A.rows(), 0, A.rows(), nq);
    }
  }
  return ret;

  // much slower and requires eigen/unsupported:
//  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}

template<typename DerivedDA, typename Derivedb>
Eigen::Matrix<typename DerivedDA::Scalar, matGradMultNumRows(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime>
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<Derivedb>& b) {
  const int nq = dA.cols();
  assert(b.cols() == 1);
  assert(dA.rows() % b.rows() == 0);
  const int A_rows = dA.rows() / b.rows();

  Eigen::Matrix<typename DerivedDA::Scalar, matGradMultNumRows(DerivedDA::RowsAtCompileTime, Derivedb::RowsAtCompileTime), DerivedDA::ColsAtCompileTime> ret(A_rows, nq);

  ret.setZero();
  for (int row = 0; row < b.rows(); row++) {
    ret += b(row, 0) * dA.block(row * A_rows, 0, A_rows, nq);
  }
  return ret;
}

// TODO: could save copies once http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::vector<int>& rows, const std::vector<int>& cols,
    int M_rows, int q_start, int q_subvector_size) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::MatrixXd dM_submatrix(rows.size() * cols.size(), q_subvector_size);
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM_submatrix.row(index) = dM.block(row + col * M_rows, q_start, 1, q_subvector_size);
      index++;
    }
  }
  return dM_submatrix;
}

template<int QSubvectorSize, typename Derived, unsigned long NRows, unsigned long NCols>
Eigen::Matrix<typename Derived::Scalar, NRows * NCols, QSubvectorSize == Eigen::Dynamic ? Derived::ColsAtCompileTime : QSubvectorSize>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows, int q_start, int q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::Matrix<typename Derived::Scalar, NRows * NCols, Derived::ColsAtCompileTime> dM_submatrix(NRows * NCols, q_subvector_size);
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM_submatrix.row(index++) = dM.template block<1, QSubvectorSize> (row + col * M_rows, q_start, 1, q_subvector_size);
    }
  }
  return dM_submatrix;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 1, Derived::ColsAtCompileTime>
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, int row, int col, int M_rows) {
  return dM.row(row + col * M_rows);
}

template<typename DerivedA, typename DerivedB>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, int M_rows, int q_start, int q_subvector_size) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM.block(row + col * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
}

template<int QSubvectorSize, typename DerivedA, typename DerivedB, unsigned long NRows, unsigned long NCols>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, int M_rows, int q_start, int q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (int col : cols) {
    for (int row : rows) {
      dM.template block<1, QSubvectorSize> (row + col * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
}

// explicit instantiations
#define MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows) \
		template Eigen::Matrix<Type, matGradMultNumRows(DARows, BRows), DACols>\
		matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >& dA, const Eigen::MatrixBase< Eigen::Matrix<Type, BRows, 1> >& b);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 6)
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols, BBlockRows) \
    template Eigen::Matrix<Type, matGradMultNumRows(DARows, BBlockRows), DACols> \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix<Type, BRows, BCols> const, BBlockRows, 1> >&);
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, Eigen::Dynamic, 3)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 4, 4, 3)
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
		template Eigen::Matrix<Type, matGradMultMatNumRows(ARows, BCols), NQ> matGradMultMat(\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 1, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 3, 4, 4, 12, 16, 4)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 4, 3, 3, 12, 9, 4)
#undef MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template Eigen::Matrix<Type, matGradMultMatNumRows(ARows, BCols), NQ> matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, ACols, ARows>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(double, 3, 3, 4, 9, 12, 4)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template Eigen::Matrix<Type, matGradMultMatNumRows(ARows, BCols), NQ> matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, BCols, ACols>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 3, 1, 3, 3, 3, 3)
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 4, 1, 4, 4, 4, 4)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, NRows, NCols) \
		template void setSubMatrixGradient<QSubvectorSize,Eigen::Matrix<Type, DMRows, DMCols>, Eigen::Matrix<Type, DMSubRows, DMSubCols>, NRows, NCols>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const Eigen::MatrixBase< Eigen::Matrix<Type, DMSubRows, DMSubCols> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, int, int, int);
//MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 9, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 3, 4)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 4, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 4, 3)
#undef MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION


#define MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, NRows, NCols, QSubvectorSize) \
		template Eigen::Matrix<Type, (NRows) * (NCols), QSubvectorSize == Eigen::Dynamic ? DMCols : QSubvectorSize> \
		getSubMatrixGradient(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, \
		    const std::array<int, NRows>&, const std::array<int, NCols>&, int, int, int);
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 3, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 3, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols) \
		template Eigen::Matrix<Type, 1, DMCols> \
		getSubMatrixGradient(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, int, int, int);
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION

#define MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(Type, Rows, Cols) \
        template Eigen::Matrix<Type, Rows, Cols>::PlainObject transposeGrad(const Eigen::MatrixBase<Eigen::Matrix<Type, Rows, Cols>>&, int);
//MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 3, 3)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 4, 4)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 48, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, 4)
#undef MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION
