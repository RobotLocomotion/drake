/*
* drakeUtil.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "drakeUtil.h"
#include <string>
#include <math.h>
#include <limits>
#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <functional>

using namespace std;
using namespace Eigen;

void baseZeroToBaseOne(std::vector<int>& vec)
{
  for (std::vector<int>::iterator iter=vec.begin(); iter!=vec.end(); iter++)
    (*iter)++;
}

double angleAverage(double theta1, double theta2) {
  // Computes the average between two angles by averaging points on the unit
  // circle and taking the arctan of the result.
  //   see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
  // theta1 is a scalar or column vector of angles (rad)
  // theta2 is a scalar or column vector of angles (rad)

  double x_mean = 0.5 * (cos(theta1) + cos(theta2));
  double y_mean = 0.5 * (sin(theta1) + sin(theta2));

  double angle_mean = atan2(y_mean, x_mean);

  return angle_mean;
}

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal, typename DerivedPoint>
std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(const Eigen::MatrixBase<DerivedTorque> & torque, const Eigen::MatrixBase<DerivedForce> & force, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane)
{
  // TODO: implement multi-column version
  using namespace Eigen;

  if (abs(normal.squaredNorm() - 1.0) > 1e-12) {
    throw std::runtime_error("Drake:resolveCenterOfPressure:BadInputs: normal should be a unit vector");
  }

  Vector3d cop;
  double normal_torque_at_cop;

  double fz = normal.dot(force);
  bool cop_exists = abs(fz) > 1e-12;

  if (cop_exists) {
    auto torque_at_point_on_contact_plane = torque - point_on_contact_plane.cross(force);
    double normal_torque_at_point_on_contact_plane = normal.dot(torque_at_point_on_contact_plane);
    auto tangential_torque = torque_at_point_on_contact_plane - normal * normal_torque_at_point_on_contact_plane;
    cop = normal.cross(tangential_torque) / fz + point_on_contact_plane;
    auto torque_at_cop = torque - cop.cross(force);
    normal_torque_at_cop = normal.dot(torque_at_cop);
  }
  else {
    cop.setConstant(std::numeric_limits<double>::quiet_NaN());
    normal_torque_at_cop = std::numeric_limits<double>::quiet_NaN();
  }
  return std::pair<Vector3d, double>(cop, normal_torque_at_cop);
}

//Based on the Matrix Sign Function method outlined in this paper:
//http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf
template <typename DerivedA, typename DerivedB, typename DerivedQ, typename DerivedR, typename DerivedX>
void care(MatrixBase<DerivedA> const& A, MatrixBase<DerivedB> const& B, MatrixBase<DerivedQ> const& Q, MatrixBase<DerivedR> const& R,  MatrixBase<DerivedX> & X)
{
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
void lqr(MatrixBase<DerivedA> const& A, MatrixBase<DerivedB> const& B, MatrixBase<DerivedQ> const& Q, MatrixBase<DerivedR> const& R, MatrixBase<DerivedK> & K, MatrixBase<DerivedS> & S)
{
  LLT<MatrixXd> R_cholesky(R);
  care(A, B, Q, R, S);
  K = R_cholesky.solve(B.transpose() * S);
}

template DLLEXPORT void lqr(MatrixBase<MatrixXd> const& A, MatrixBase<MatrixXd> const& B, MatrixBase<MatrixXd> const& Q, MatrixBase<MatrixXd> const& R, MatrixBase<MatrixXd> & K, MatrixBase<MatrixXd> & S);
template DLLEXPORT void lqr(MatrixBase< Map<MatrixXd> > const& A, MatrixBase< Map<MatrixXd> > const& B, MatrixBase< Map<MatrixXd> > const& Q, MatrixBase< Map<MatrixXd> > const& R, MatrixBase< Map<MatrixXd> > & K, MatrixBase< Map<MatrixXd> > & S);
template DLLEXPORT void lqr(MatrixBase<MatrixXd> const& A, MatrixBase<MatrixXd> const& B, MatrixBase<MatrixXd> const& Q, MatrixBase<MatrixXd> const& R, MatrixBase< Map<MatrixXd> > & K, MatrixBase< Map<MatrixXd> > & S);
template DLLEXPORT void lqr<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, -1, true>, Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, -1, true>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, -1, true> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, -1, true> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);

template DLLEXPORT void care(MatrixBase<MatrixXd> const& A, MatrixBase<MatrixXd> const& B, MatrixBase<MatrixXd> const& Q, MatrixBase<MatrixXd> const& R, MatrixBase<MatrixXd> & X);
template DLLEXPORT void care(MatrixBase< Map<MatrixXd> > const& A, MatrixBase< Map<MatrixXd> > const& B, MatrixBase< Map<MatrixXd> > const& Q, MatrixBase< Map<MatrixXd> > const& R, MatrixBase< Map<MatrixXd> > & X);

template DLLEXPORT std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double> resolveCenterOfPressure<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> >, Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> >, Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> >, Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> > >(Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> > > const&, Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> > > const&, Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> > > const&, Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0, Eigen::Stride<0, 0> > > const&);
template DLLEXPORT std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double> resolveCenterOfPressure<Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Matrix<double, 3, 1, 0, 3, 1> >(Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&);
template DLLEXPORT std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double> resolveCenterOfPressure<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>, Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>, Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Matrix<double, 3, 1, 0, 3, 1> >(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false> > const&, Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&);
