#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include <cmath>
#include <vector>
#include <array>
#include <cassert>
#include <stdexcept>


template<std::size_t Size>
std::array<int, Size> intRange(int start)
{
  std::array<int, Size> ret;
  for (unsigned int i = 0; i < Size; i++) {
    ret[i] = i + start;
  }
  return ret;
}

namespace Drake {
  // todo: recursive template to get arbitrary gradient order

  // note: tried using template default values (e.g. Eigen::Dynamic), but they didn't seem to work on my mac clang
  template <int num_vars> using TaylorVar = Eigen::AutoDiffScalar< Eigen::Matrix<double,num_vars,1> >;
  template <int num_vars, int rows> using TaylorVec = Eigen::Matrix< TaylorVar<num_vars>, rows, 1>;
  template <int num_vars, int rows, int cols> using TaylorMat = Eigen::Matrix< TaylorVar<num_vars>, rows, cols>;

  typedef TaylorVar<Eigen::Dynamic> TaylorVarX;
  typedef TaylorVec<Eigen::Dynamic,Eigen::Dynamic> TaylorVecX;
  typedef TaylorMat<Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic> TaylorMatX;

  // initializes the vector with x=val and dx=eye(numel(val))
  template <typename Derived>
  TaylorVecX initTaylorVecX(const Eigen::MatrixBase<Derived>& val) {
    TaylorVecX x(val.rows());
    Eigen::MatrixXd der = Eigen::MatrixXd::Identity(val.rows(),val.rows());
    for (int i=0; i<val.rows(); i++) {
      x(i).value() = val(i);
      x(i).derivatives() = der.col(i);
    }
    return x;
  }
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

template <typename Derived>
struct AutoDiffToValueMatrix {
  typedef typename Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> type;
};

template <typename Derived>
struct AutoDiffToGradientMatrix {
  typedef typename Gradient<Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>, Eigen::Dynamic>::type type;
};

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
  assert(B.rows() == 0 ? dA.rows() == 0 : dA.rows() % B.rows() == 0);
  typename DerivedDA::Index A_rows = B.rows() == 0 ? 0 : dA.rows() / B.rows();
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
    typename Derived::Index M_rows, int q_start = 0, typename Derived::Index q_subvector_size = -1) {
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
  typename Derived::Index M_rows, int q_start = 0, typename Derived::Index q_subvector_size = QSubvectorSize) {
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
};

template<int QSubvectorSize, typename Derived>
typename GetSubMatrixGradientSingleElement<QSubvectorSize, Derived>::type
getSubMatrixGradient(const Eigen::MatrixBase<Derived>& dM, int row, int col, typename Derived::Index M_rows,
    typename Derived::Index q_start = 0, typename Derived::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  return dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1, q_subvector_size);
};

template<typename DerivedA, typename DerivedB>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::vector<int>& rows, const std::vector<int>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start = 0, typename DerivedA::Index q_subvector_size = -1) {
  if (q_subvector_size < 0) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::vector<int>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (typename std::vector<int>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM.block((*row) + (*col) * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
};

template<int QSubvectorSize, typename DerivedA, typename DerivedB, std::size_t NRows, std::size_t NCols>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedA>& dM, const Eigen::MatrixBase<DerivedB>& dM_submatrix,
    const std::array<int, NRows>& rows, const std::array<int, NCols>& cols, typename DerivedA::Index M_rows, typename DerivedA::Index q_start = 0, typename DerivedA::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  int index = 0;
  for (typename std::array<int, NCols>::const_iterator col = cols.begin(); col != cols.end(); ++col) {
    for (typename std::array<int, NRows>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
      dM.template block<1, QSubvectorSize> ((*row) + (*col) * M_rows, q_start, 1, q_subvector_size) = dM_submatrix.row(index++);
    }
  }
};

template<int QSubvectorSize, typename DerivedDM, typename DerivedDMSub>
void setSubMatrixGradient(Eigen::MatrixBase<DerivedDM>& dM, const Eigen::MatrixBase<DerivedDMSub>& dM_submatrix, int row, int col, typename DerivedDM::Index M_rows,
    typename DerivedDM::Index q_start = 0, typename DerivedDM::Index q_subvector_size = QSubvectorSize) {
  if (q_subvector_size == Eigen::Dynamic) {
    q_subvector_size = dM.cols() - q_start;
  }
  dM.template block<1, QSubvectorSize>(row + col * M_rows, q_start, 1, q_subvector_size) = dM_submatrix;
};

template <typename Derived>
typename AutoDiffToValueMatrix<Derived>::type autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  typename AutoDiffToValueMatrix<Derived>::type ret(auto_diff_matrix.rows(), auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); i++) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      ret(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return ret;
};

template<typename Derived>
typename AutoDiffToGradientMatrix<Derived>::type autoDiffToGradientMatrix(
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

  typename AutoDiffToGradientMatrix<Derived>::type gradient(auto_diff_matrix.size(), num_variables);
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
  typedef typename Eigen::MatrixBase<DerivedGradient>::Index Index;
  auto nx = gradient.cols();
  for (Index row = 0; row < auto_diff_matrix.rows(); row++) {
    for (Index col = 0; col < auto_diff_matrix.cols(); col++) {
      auto_diff_matrix(row, col).derivatives().resize(nx, 1);
      auto_diff_matrix(row, col).derivatives() = gradient.row(row + col * auto_diff_matrix.rows()).transpose();
    }
  }
}

#endif /* DRAKEGRADIENTUTIL_H_ */
