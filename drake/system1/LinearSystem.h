#pragma once

#include "drake/common/drake_assert.h"
#include "drake/system1/System.h"

namespace drake {

/**
 * Builds an affine system from its state-space matrix coefficients *A*, *B*,
 * *C*, and *D*. This is done by implementing the following equations:
 *
 * @f[
 *   \dot{x} = Ax + Bu + \dot{x}_0 \\
 *   y = Cx + Du + y_0
 * @f]
 *
 * Where *x* is the system state, *u* is the system inputs, and *y* is the
 * system output.
 *
 * @concept{system_concept}
 */

template <template <typename> class StateVec,
          template <typename> class InputVec,
          template <typename> class OutputVec>
class AffineSystem {
 public:
  template <typename ScalarType>
  using StateVector = StateVec<ScalarType>;
  template <typename ScalarType>
  using OutputVector = OutputVec<ScalarType>;
  template <typename ScalarType>
  using InputVector = InputVec<ScalarType>;

  /**
   * A constructor that creates an Affine System based on its state-space matrix
   * coefficients.
   *
   * @param[in] A The matrix coefficients of the system state *x* in the
   * equation for the time derivative of the system's state *xdot*.
   *
   * @param[in] B The matrix coefficients of the system input *u* in the
   * equation for the time derivative of the system's state *xdot*.
   *
   * @param[in] xdot0 The initial value of the system state's time derivative.
   *
   * @param[in] C The matrix coefficients of the system state *x* in the
   * equation for the system's output *y*.
   *
   * @param[in] D The matrix coefficients of the system input *u* in the
   * equation for the system's output *y*.
   *
   * @param[in] y0 The initial value of the system's output.
   */
  template <typename DerivedA, typename DerivedB, typename Derivedxdot0,
            typename DerivedC, typename DerivedD, typename Derivedy0>
  AffineSystem(const Eigen::MatrixBase<DerivedA>& A,
               const Eigen::MatrixBase<DerivedB>& B,
               const Eigen::MatrixBase<Derivedxdot0>& xdot0,
               const Eigen::MatrixBase<DerivedC>& C,
               const Eigen::MatrixBase<DerivedD>& D,
               const Eigen::MatrixBase<Derivedy0>& y0)
      : A_(A), B_(B), C_(C), D_(D), xdot0_(xdot0), y0_(y0) {
    DRAKE_ASSERT(A.rows() == A.cols());
    DRAKE_ASSERT(B.rows() == A.cols());
    DRAKE_ASSERT(xdot0.rows() == A.cols());
    DRAKE_ASSERT(C.cols() == A.cols());
    DRAKE_ASSERT(y0.rows() == C.rows());
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    if (A_.rows() == 0) return StateVector<ScalarType>();
    StateVector<ScalarType> xdot = A_ * toEigen(x) + B_ * toEigen(u) + xdot0_;
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    OutputVector<ScalarType> y = C_ * toEigen(x) + D_ * toEigen(u) + y0_;
    return y;
  }

  bool isTimeVarying() const { return false; }

  /**
   * A system is direct feedthrough if the outputs of the system (*y*) directly
   * depend on the inputs of the system (*u*). In this case, *y* will directly
   * depend on *u* only if *D* is non-zero.
   */
  bool isDirectFeedthrough() const { return !D_.isZero(); }
  size_t getNumStates() const { return static_cast<size_t>(A_.cols()); }
  size_t getNumInputs() const { return static_cast<size_t>(B_.cols()); }
  size_t getNumOutputs() const { return static_cast<size_t>(C_.rows()); }

 private:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> xdot0_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> y0_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // the num_states, etc can cause alignment
                                   // issues if they are one of the known fixed
                                   // sizes.
};

/**
 * This is a specialization of AffineSystem where the initial time derivative
 * of the system inputs (*xdot0*) and the system outputs (*y0*) are both zero.
 * In other words, this system implements:
 *
 *
 * @f[
 *   \dot{x} = Ax + Bu\\
 *   y = Cx + Du
 * @f]
 *
 * Where *x* is the system state, *u* is the system inputs, and *y* is the
 * system output.
 */
template <template <typename> class StateVec,
          template <typename> class InputVec,
          template <typename> class OutputVec>
class LinearSystem : public AffineSystem<StateVec, InputVec, OutputVec> {
 public:
  template <typename ScalarType>
  using StateVector = StateVec<ScalarType>;
  template <typename ScalarType>
  using OutputVector = OutputVec<ScalarType>;
  template <typename ScalarType>
  using InputVector = InputVec<ScalarType>;

  /**
   * A constructor that simply instantiates an AffineSystem where *xdot0* and
   * *y0* are zero.
   *
   * @param[in] A The matrix coefficients of the system state *x* in the
   * equation for the time derivative of the system's state *xdot*.
   *
   * @param[in] B The matrix coefficients of the system input *u* in the
   * equation for the time derivative of the system's state *xdot*.
   *
   * @param[in] C The matrix coefficients of the system state *x* in the
   * equation for the system's output *y*.
   *
   * @param[in] D The matrix coefficients of the system input *u* in the
   * equation for the system's output *y*.
   */
  template <typename DerivedA, typename DerivedB, typename DerivedC,
            typename DerivedD>
  LinearSystem(const Eigen::MatrixBase<DerivedA>& A,
               const Eigen::MatrixBase<DerivedB>& B,
               const Eigen::MatrixBase<DerivedC>& C,
               const Eigen::MatrixBase<DerivedD>& D)
      : AffineSystem<StateVec, InputVec, OutputVec>(
            A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
            Eigen::VectorXd::Zero(C.rows())) {}
};

/**
 * A specialization of LinearSystem in which *A*, *B*, and *C* are all zero.
 * In other words, this system implements:
 *
 *
 * @f[ y = Du @f]
 *
 * Where *y* is the system's output, *u* is the system's input, and *D* is the
 * matrix coefficient for *u*.
 */
template <template <typename> class InputVec,
          template <typename> class OutputVec>
class Gain : public LinearSystem<NullVector, InputVec, OutputVec> {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = OutputVec<ScalarType>;
  template <typename ScalarType>
  using InputVector = InputVec<ScalarType>;

  /**
   * The constructor that simply instantiates a LinearSystem where the
   * coefficient matrices *A*, *B*, and *C* are all zero.
   *
   * @param[in] D The matrix coefficients of the system input *u* in the
   * equation for the system's output *y*.
   */
  template <typename Derived>
  explicit Gain(const Eigen::MatrixBase<Derived>& D)
      : LinearSystem<NullVector, InputVec, OutputVec>(
            Eigen::Matrix<double, 0, 0>(), Eigen::Matrix<double, 0, 0>(),
            Eigen::Matrix<double, Eigen::Dynamic, 0>(D.rows(), 0), D) {}
};

}  // end namespace drake
