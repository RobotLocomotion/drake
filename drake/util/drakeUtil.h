/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#ifndef DRAKE_UTIL_H_
#define DRAKE_UTIL_H_

#include <stdexcept>
#include <vector>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>
#include "drake/drakeUtil_export.h"


template <typename Key, typename T>
using eigen_aligned_unordered_map = std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>, Eigen::aligned_allocator<std::pair<Key const, T > > >;

template <typename Derived>
inline void sizecheck(const Eigen::MatrixBase<Derived>& mat, size_t rows, size_t cols){
  if ((mat.rows() != rows) || (mat.cols() != cols))
    throw std::runtime_error("Wrong-sized matrix:  Expected " + std::to_string(rows) + "-by-" + std::to_string(cols) + " but got " + std::to_string(mat.rows()) + "-by-" + std::to_string(mat.cols()));
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
// can only be used when the dimension information of the array is known at compile time
template <size_t Rows, size_t Cols, typename Derived>
void eigenToCArrayOfArrays(const Eigen::MatrixBase<Derived>& source, double (&destination)[Rows][Cols]) {
  if (Rows != source.rows())
    throw std::runtime_error("Number of rows of source doesn't match destination");
  if (Cols != source.cols())
    throw std::runtime_error("Number of columns of source doesn't match destination");
  for (size_t row = 0; row < Rows; ++row) {
    for (size_t col = 0; col < Cols; ++col) {
      destination[row][col] = source(row, col);
    }
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
// can only be used when the dimension information of the array is known at compile time
template <size_t Size, typename Derived>
void eigenVectorToCArray(const Eigen::MatrixBase<Derived>& source, double (&destination)[Size]) {
  if (Size != source.size())
    throw std::runtime_error("Size of source doesn't match destination");
  for (size_t i = 0; i < Size; ++i) {
    destination[i] = source(i);
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
template <typename Derived>
void eigenVectorToStdVector(const Eigen::MatrixBase<Derived>& source, std::vector<typename Derived::Scalar>& destination) {
  assert(source.rows() == 1 || source.cols() == 1);
  destination.resize(static_cast<size_t>(source.size()));
  for (Eigen::DenseIndex i = 0; i < source.size(); i++) {
    destination[static_cast<size_t>(i)] = source(i);
  }
}

// note for if/when we split off all Matlab related stuff into a different file: this function is not Matlab related
template <typename Derived>
void eigenToStdVectorOfStdVectors(const Eigen::MatrixBase<Derived>& source, std::vector< std::vector<typename Derived::Scalar> >& destination) {
  destination.resize(source.rows());
  for (Eigen::DenseIndex row = 0; row < source.rows(); ++row) {
    auto& destination_row = destination[row];
    destination_row.resize(source.cols());
    for (Eigen::DenseIndex col = 0; col < source.cols(); ++col) {
      destination_row[col] = source(row, col);
    }
  }
}



template <typename T>
void addOffset(std::vector<T>& v, const T& offset)
{
  std::transform(v.begin(), v.end(), v.begin(), std::bind2nd(std::plus<double>(), offset));
}

DRAKEUTIL_EXPORT void baseZeroToBaseOne(std::vector<int>& vec);

DRAKEUTIL_EXPORT double angleAverage(double theta1, double theta2);

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal, typename DerivedPoint>
DRAKEUTIL_EXPORT std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(const Eigen::MatrixBase<DerivedTorque> & torque, const Eigen::MatrixBase<DerivedForce> & force, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane);


//Based on the Matrix Sign Function method outlined in this paper:
//http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf
template <typename DerivedA, typename DerivedB, typename DerivedQ, typename DerivedR, typename DerivedX>
void care(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B, Eigen::MatrixBase<DerivedQ> const& Q, Eigen::MatrixBase<DerivedR> const& R,  Eigen::MatrixBase<DerivedX> & X)
{
  using namespace std;
  using namespace Eigen;
  const size_t n = A.rows();

  LLT<MatrixXd> R_cholesky(R);

  MatrixXd H(2 * n, 2 * n);
  H << A, B * R_cholesky.solve(B.transpose()), Q, -A.transpose();

  MatrixXd Z = H;
  MatrixXd Z_old;

  //these could be options
  const double tolerance = 1e-9;
  const double max_iterations = 100;

  double relative_norm;
  size_t iteration = 0;

  const double p = static_cast<double>(Z.rows());

  do {
    Z_old = Z;
    //R. Byers. Solving the algebraic Riccati equation with the matrix sign function. Linear Algebra Appl., 85:267–279, 1987
    //Added determinant scaling to improve convergence (converges in rough half the iterations with this)
    double ck = pow(abs(Z.determinant()), -1.0/p);
    Z *= ck;
    Z = Z - 0.5 * (Z - Z.inverse());
    relative_norm = (Z - Z_old).norm();
    iteration ++;
  } while(iteration < max_iterations && relative_norm > tolerance);

  MatrixXd W11 = Z.block(0, 0, n, n);
  MatrixXd W12 = Z.block(0, n, n, n);
  MatrixXd W21 = Z.block(n, 0, n, n);
  MatrixXd W22 = Z.block(n, n, n, n);

  MatrixXd lhs(2 * n, n);
  MatrixXd rhs(2 * n, n);
  MatrixXd eye = MatrixXd::Identity(n, n);
  lhs << W12, W22 + eye;
  rhs << W11 + eye, W21;

  JacobiSVD<MatrixXd> svd(lhs, ComputeThinU | ComputeThinV);

  X = svd.solve(rhs);
}

template <typename DerivedA, typename DerivedB, typename DerivedQ, typename DerivedR, typename DerivedK, typename DerivedS>
void lqr(Eigen::MatrixBase<DerivedA> const& A, Eigen::MatrixBase<DerivedB> const& B, Eigen::MatrixBase<DerivedQ> const& Q, Eigen::MatrixBase<DerivedR> const& R, Eigen::MatrixBase<DerivedK> & K, Eigen::MatrixBase<DerivedS> & S)
{
  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  care(A, B, Q, R, S);
  K = R_cholesky.solve(B.transpose() * S);
}


#endif /* DRAKE_UTIL_H_ */
