#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/core/Gradient.h"
#include "drake/core/Vector.h"
#include "drake/math/autodiff.h"
#include "drake/systems/LinearSystem.h"
#include "drake/util/drakeGradientUtil.h"
#include "drake/util/drakeUtil.h"

using drake::math::autoDiffToGradientMatrix;

namespace drake {

template <typename System>
std::shared_ptr<AffineSystem<NullVector, System::template StateVector,
                             System::template InputVector>>
timeInvariantLQR(const System& sys,
                 const typename System::template StateVector<double>& x0,
                 const typename System::template InputVector<double>& u0,
                 const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  const int num_states =
      System::template StateVector<double>::RowsAtCompileTime;
  const int num_inputs =
      System::template InputVector<double>::RowsAtCompileTime;
  DRAKE_ASSERT(!sys.isTimeVarying());
  static_assert(num_states != 0, "This system has no continuous states");
  using namespace std;
  using namespace Eigen;

  auto autodiff_args = initializeAutoDiffTuple(toEigen(x0), toEigen(u0));
  typedef typename std::tuple_element<0, decltype(autodiff_args)>::type::Scalar
      AutoDiffType;
  typename System::template StateVector<AutoDiffType> x_taylor(
      std::get<0>(autodiff_args));
  typename System::template InputVector<AutoDiffType> u_taylor(
      std::get<1>(autodiff_args));
  auto xdot = autoDiffToGradientMatrix(
      toEigen(sys.dynamics(AutoDiffType(0), x_taylor, u_taylor)));
  auto A = xdot.leftCols(num_states);
  auto B = xdot.rightCols(num_inputs);

  Eigen::MatrixXd K(num_inputs, num_states), S(num_states, num_states);
  lqr(A, B, Q, R, K, S);

  SPDLOG_TRACE(drake::log(), "K = {} S = {}", K, S);

  // todo: return the linear system with the affine transform.  But for now,
  // just give the affine controller:

  // u = u0 - K(x-x0)
  Matrix<double, 0, 0> nullmat;
  Matrix<double, 0, 1> nullvec;

  return std::make_shared<AffineSystem<NullVector, System::template StateVector,
                                       System::template InputVector>>(
      nullmat, Matrix<double, 0, num_states>::Zero(), nullvec,
      Matrix<double, num_inputs, 0>::Zero(), -K, toEigen(u0) + K * toEigen(x0));
}
}
