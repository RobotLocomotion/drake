#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/vector.h"
#include "drake/util/drakeGradientUtil.h"
#include "drake/util/drakeUtil.h"

using drake::math::autoDiffToGradientMatrix;

namespace drake {

/**
 * Calculate the matrices for an LQR controller.  @p x0 is the target
 * system state vector, and @p u0 is the target system input vector.
 */
template <typename System, typename DerivedA, typename DerivedB,
          typename DerivedQ, typename DerivedR, typename DerivedK,
          typename DerivedS>
void CalculateLqrMatrices(
    const System& sys,
    const Eigen::MatrixBase<DerivedA>& x0,
    const Eigen::MatrixBase<DerivedB>& u0,
    const Eigen::MatrixBase<DerivedQ>& Q,
    const Eigen::MatrixBase<DerivedR>& R,
    Eigen::MatrixBase<DerivedK>* K,
    Eigen::MatrixBase<DerivedS>* S) {

  auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);
  typedef typename std::tuple_element<0, decltype(autodiff_args)>::type::Scalar
      AutoDiffType;
  typename System::template StateVector<AutoDiffType> x_taylor(
      std::get<0>(autodiff_args));
  typename System::template InputVector<AutoDiffType> u_taylor(
      std::get<1>(autodiff_args));
  auto xdot = autoDiffToGradientMatrix(
      toEigen(sys.dynamics(AutoDiffType(0), x_taylor, u_taylor)));
  auto A = xdot.leftCols(x0.size());
  auto B = xdot.rightCols(u0.size());

  lqr(A, B, Q, R, *K, *S);
}

template <typename System>
std::shared_ptr<AffineSystem<NullVector, System::template StateVector,
                             System::template InputVector>>
MakeTimeInvariantLqrSystem(
    const System& sys,
    // NOLINTNEXTLINE(runtime/references) This code will be deleted soon.
    const typename System::template StateVector<double>& x0,
    // NOLINTNEXTLINE(runtime/references) This code will be deleted soon.
    const typename System::template InputVector<double>& u0,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  const int num_states =
      System::template StateVector<double>::RowsAtCompileTime;
  const int num_inputs =
      System::template InputVector<double>::RowsAtCompileTime;
  DRAKE_ASSERT(!sys.isTimeVarying());
  static_assert(num_states != 0, "This system has no continuous states");

  Eigen::MatrixXd K(num_inputs, num_states), S(num_states, num_states);
  CalculateLqrMatrices(sys, toEigen(x0), toEigen(u0), Q, R, &K, &S);

  SPDLOG_TRACE(drake::log(), "K = {} S = {}", K, S);

  // todo: return the linear system with the affine transform.  But for now,
  // just give the affine controller:

  // u = u0 - K(x-x0)
  Eigen::Matrix<double, 0, 0> nullmat;
  Eigen::Matrix<double, 0, 1> nullvec;

  return std::make_shared<AffineSystem<NullVector, System::template StateVector,
                                       System::template InputVector>>(
      nullmat, Eigen::Matrix<double, 0, num_states>::Zero(), nullvec,
      Eigen::Matrix<double, num_inputs, 0>::Zero(),
      -K, toEigen(u0) + K * toEigen(x0));
}

}  // namespace drake
