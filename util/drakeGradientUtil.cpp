#include "drakeGradientUtil.h"
#include <cassert>

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(
    const Eigen::MatrixBase<Derived>& dX, typename Derived::Index rows_X)
{
  typename Derived::PlainObject dX_transpose(dX.rows(), dX.cols());
  typename Derived::Index numel = dX.rows();
  typename Derived::Index index = 0;
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
typename MatGradMultMat<DerivedA, DerivedB, DerivedDA>::type
matGradMultMat(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    const Eigen::MatrixBase<DerivedDA>& dA,
    const Eigen::MatrixBase<DerivedDB>& dB) {
  assert(dA.cols() == dB.cols());

  typename MatGradMultMat<DerivedA, DerivedB, DerivedDA>::type ret(A.rows() * B.cols(), dA.cols());

  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<DerivedA::RowsAtCompileTime, DerivedDA::ColsAtCompileTime>(col * A.rows(), 0, A.rows(), dA.cols());

    // A * dB part:
    block.noalias() = A * dB.template block<DerivedA::ColsAtCompileTime, DerivedDA::ColsAtCompileTime>(col * A.cols(), 0, A.cols(), dA.cols());

    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) * dA.template block<DerivedA::RowsAtCompileTime, DerivedDA::ColsAtCompileTime>(row * A.rows(), 0, A.rows(), dA.cols());
    }
  }
  return ret;

  // much slower and requires eigen/unsupported:
//  return Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(B.cols(), B.cols()), A) * dB + Eigen::kroneckerProduct(B.transpose(), Eigen::MatrixXd::Identity(A.rows(), A.rows())) * dA;
}

template<typename DerivedDA, typename DerivedB>
typename MatGradMult<DerivedDA, DerivedB>::type
matGradMult(const Eigen::MatrixBase<DerivedDA>& dA, const Eigen::MatrixBase<DerivedB>& B) {
  assert(dA.rows() % B.rows() == 0);
  typename DerivedDA::Index A_rows = dA.rows() / B.rows();
  const int A_rows_at_compile_time = (DerivedDA::RowsAtCompileTime == Eigen::Dynamic || DerivedB::RowsAtCompileTime == Eigen::Dynamic) ?
      Eigen::Dynamic :
      static_cast<int>(DerivedDA::RowsAtCompileTime / DerivedB::RowsAtCompileTime);

  typename MatGradMult<DerivedDA, DerivedB>::type ret(A_rows * B.cols(), dA.cols());
  ret.setZero();
  for (int col = 0; col < B.cols(); col++) {
    auto block = ret.template block<A_rows_at_compile_time, DerivedDA::ColsAtCompileTime>(col * A_rows, 0, A_rows, dA.cols());
    for (int row = 0; row < B.rows(); row++) {
      // B * dA part:
      block.noalias() += B(row, col) * dA.template block<A_rows_at_compile_time, DerivedDA::ColsAtCompileTime>(row * A_rows, 0, A_rows, dA.cols());
    }
  }
  return ret;
}

// TODO: could save copies once http://eigen.tuxfamily.org/bz/show_bug.cgi?id=329 is fixed
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(
    const Eigen::MatrixBase<Derived>& dM, const std::vector<int>& rows, const std::vector<int>& cols,
    typename Derived::Index M_rows, int q_start, typename Derived::Index q_subvector_size) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::MatrixXd dM_submatrix(rows.size() * cols.size(), q_subvector_size);
  int index = 0;
  for (std::vector<int>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (std::vector<int>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM_submatrix.row(index) = dM.block(*row + *col * M_rows, q_start, 1, q_subvector_size);
      index++;
    }
  }
  return dM_submatrix;
}

template<int QSubvectorSize, typename Derived, std::size_t NRows, std::size_t NCols>
typename GetSubMatrixGradientArray<QSubvectorSize, Derived, NRows, NCols>::type
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM,
  const std::array<int, NRows>& rows,
  const std::array<int, NCols>& cols,
  typename Derived::Index M_rows, int q_start, typename Derived::Index q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  Eigen::Matrix<typename Derived::Scalar, NRows * NCols, Derived::ColsAtCompileTime> dM_submatrix(NRows * NCols, q_subvector_size);
  int index = 0;
  for (typename std::array<int, NCols>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (typename std::array<int, NRows>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM_submatrix.row(index++) = dM.template block<1, QSubvectorSize> ((*row) + (*col) * M_rows, q_start, 1, q_subvector_size);
    }
  }
  return dM_submatrix;
}

template<int QSubvectorSize, typename Derived>
typename GetSubMatrixGradientSingleElement<QSubvectorSize, Derived>::type
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, int row, int col, typename Derived::Index M_rows,
    typename Derived::Index q_start, typename Derived::Index q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  return dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1, q_subvector_size);
}

template<typename DerivedA, typename DerivedB>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start, typename DerivedA::Index q_subvector_size) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::vector<int>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (typename std::vector<int>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM.block((*row) + (*col) * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
}

template<int QSubvectorSize, typename DerivedA, typename DerivedB, std::size_t NRows, std::size_t NCols>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start, typename DerivedA::Index q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::array<int, NCols>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (typename std::array<int, NRows>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM.template block<1, QSubvectorSize> ((*row) + (*col) * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
}

template<int QSubvectorSize, typename DerivedDM, typename DerivedDMSub>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedDM>& dM, const Eigen::MatrixBase<DerivedDMSub>& dM_submatrix, int row, int col, typename DerivedDM::Index M_rows,
    typename DerivedDM::Index q_start, typename DerivedDM::Index q_subvector_size) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1, q_subvector_size) = dM_submatrix;
}

// explicit instantiations
#define MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols) \
		template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Matrix<Type, BRows, BCols>>::type\
		matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >& dA, const Eigen::MatrixBase< Eigen::Matrix<Type, BRows, BCols> >& b);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 6, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 4, 4)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, Eigen::Dynamic)
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Map< Eigen::Matrix<Type, BRows, 1> > >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Map< Eigen::Matrix<Type, BRows, 1> > >&);
MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols, BBlockRows, BBlockCols, InnerPanel) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Block< Eigen::Matrix<Type, BRows, BCols> const, BBlockRows, BBlockCols, InnerPanel> >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix<Type, BRows, BCols> const, BBlockRows, BBlockCols, InnerPanel> >&);
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 1, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 4, 4, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1, true)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, 1, 3, 1, true)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 6, 3, 1, false)
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols, BBlockRows, BBlockCols, InnerPanel) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Block< Eigen::Transpose< Eigen::Matrix<Type, BRows, BCols> > const, BBlockRows, BBlockCols, InnerPanel> >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Block< Eigen::Transpose< Eigen::Matrix<Type, BRows, BCols> > const, BBlockRows, BBlockCols, InnerPanel> >&);
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 6, 3, 1, false)
#undef MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
		template DLLEXPORT MatGradMultMat<Eigen::Matrix<Type, ARows, ACols>, Eigen::Matrix<Type, ACols, BCols>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 1, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 3, 4, 4, 12, 16, 4)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 4, 3, 3, 12, 9, 4)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 4, 4, 4, 16, 16, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 3, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 6, 6, Eigen::Dynamic, 36, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template DLLEXPORT MatGradMultMat<Eigen::Transpose<Eigen::Matrix<Type, ACols, ARows>>, Eigen::Matrix<Type, ACols, BCols>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, ACols, ARows>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(double, 3, 3, 4, 9, 12, 4)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template DLLEXPORT MatGradMultMat<Eigen::Matrix<Type, ARows, ACols>, Eigen::Transpose<Eigen::Matrix<Type, BCols, ACols>>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, BCols, ACols>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 3, 1, 3, 3, 3, 3)
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 4, 1, 4, 4, 4, 4)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, NRows, NCols) \
		template DLLEXPORT void setSubMatrixGradient<QSubvectorSize,Eigen::Matrix<Type, DMRows, DMCols>, Eigen::Matrix<Type, DMSubRows, DMSubCols>, NRows, NCols>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const Eigen::MatrixBase< Eigen::Matrix<Type, DMSubRows, DMSubCols> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index);
//MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 6, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 9, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 3, 4)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 4, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 4, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 36, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
#undef MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, DMSubRows, DMSubCols) \
  template DLLEXPORT void setSubMatrixGradient(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const Eigen::MatrixBase< Eigen::Matrix<Type, DMSubRows, DMSubCols> >&, const std::vector<int>&, const std::vector<int>&, Eigen::Matrix<Type, DMRows, DMCols>::Index, Eigen::Matrix<Type, DMRows, DMCols>::Index, Eigen::Matrix<Type, DMRows, DMCols>::Index);
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 9, Eigen::Dynamic)
#undef MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubCols) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Matrix< Type, 1, DMSubCols > >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3)
#undef MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, DMSubBlockCols, InnerPanel) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix< Type, DMSubRows, DMSubCols> const, 1, DMSubBlockCols, InnerPanel> >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, false)
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, Eigen::Dynamic, false)
#undef MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, DMSubBlockCols, InnerPanel) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix< Type, DMSubRows, DMSubCols>, 1, DMSubBlockCols, InnerPanel> >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, false)
#undef MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, NRows, NCols, QSubvectorSize) \
	template DLLEXPORT GetSubMatrixGradientArray<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols>, NRows, NCols>::type getSubMatrixGradient<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols>, NRows, NCols>(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, Eigen::Matrix<Type, DMRows, DMCols>::Index, int, Eigen::Matrix<Type, DMRows, DMCols>::Index);
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 3, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 3, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 36, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, 3, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols) \
  template DLLEXPORT Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const std::vector<int>& rows, const std::vector<int>& cols, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, int q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols) \
  template DLLEXPORT GetSubMatrixGradientSingleElement<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols> >::type \
getSubMatrixGradient<QSubvectorSize>(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, \
    Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 9, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 3, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION

#define MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(Type, Rows, Cols) \
        template DLLEXPORT Eigen::Matrix<Type, Rows, Cols>::PlainObject transposeGrad(const Eigen::MatrixBase<Eigen::Matrix<Type, Rows, Cols>>&, Eigen::Matrix<Type, Rows, Cols>::Index);
//MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 3, 3)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 4, 4)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 48, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, 4)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 36, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION
