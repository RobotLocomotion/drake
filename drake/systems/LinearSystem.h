#ifndef DRAKE_LINEARSYSTEM_H
#define DRAKE_LINEARSYSTEM_H

#include "drake/systems/System.h"

namespace Drake {


/** AffineSystem<StateVector,InputVector,OutputVector>
 * @brief Builds an affine system from it's state-space matrix coefficients
 * @concept{system_concept}
 *
 * Implements @f[
 *   \dot{x} = Ax + B*u + \dot{x}_0 \\
 *   y = Cx + Du + y_0
 *  @f]
 */

template <template<typename> class StateVec, template<typename> class InputVec, template<typename> class OutputVec>
class AffineSystem  {
public:
  template <typename ScalarType> using StateVector = StateVec<ScalarType>;
  template <typename ScalarType> using OutputVector = OutputVec<ScalarType>;
  template <typename ScalarType> using InputVector = InputVec<ScalarType>;

  template <typename DerivedA, typename DerivedB, typename Derivedxdot0, typename DerivedC, typename DerivedD, typename Derivedy0>
  AffineSystem(const Eigen::MatrixBase<DerivedA>& A,const Eigen::MatrixBase<DerivedB>& B,const Eigen::MatrixBase<Derivedxdot0>& xdot0,
               const Eigen::MatrixBase<DerivedC>& C,const Eigen::MatrixBase<DerivedD>& D,const Eigen::MatrixBase<Derivedy0>& y0)
          : A(A),B(B),C(C),D(D),xdot0(xdot0),y0(y0) {
    assert(A.rows() == A.cols());
    assert(B.rows() == A.cols());
    assert(xdot0.rows() == A.cols());
    assert(C.cols() == A.cols());
    assert(y0.rows() == C.rows());
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
    if (A.rows() == 0) return StateVector<ScalarType>();
    StateVector<ScalarType> xdot = A * toEigen(x) + B * toEigen(u) + xdot0;
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const  {
    OutputVector<ScalarType> y = C*toEigen(x) + D*toEigen(u) + y0;
    return y;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return !D.isZero(); }
  size_t getNumStates() const {return static_cast<size_t>(A.cols()); };
  size_t getNumInputs() const {return static_cast<size_t>(B.cols()); };
  size_t getNumOutputs() const {return static_cast<size_t>(C.rows()); };

private:
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> B;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> C;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> D;
  Eigen::Matrix<double,Eigen::Dynamic,1> xdot0;
  Eigen::Matrix<double,Eigen::Dynamic,1> y0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // the num_states, etc can cause alignment issues if they are one of the known fixed sizes.
};

template <template<typename> class StateVec, template<typename> class InputVec, template<typename> class OutputVec>
class LinearSystem : public AffineSystem<StateVec,InputVec,OutputVec> {
public:
  template<typename ScalarType> using StateVector = StateVec<ScalarType>;
  template<typename ScalarType> using OutputVector = OutputVec<ScalarType>;
  template<typename ScalarType> using InputVector = InputVec<ScalarType>;

  template<typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
  LinearSystem(const Eigen::MatrixBase<DerivedA> &A, const Eigen::MatrixBase<DerivedB> &B,
               const Eigen::MatrixBase<DerivedC> &C, const Eigen::MatrixBase<DerivedD> &D)
          : AffineSystem<StateVec,InputVec,OutputVec>(A, B, Eigen::VectorXd::Zero(A.rows()), C, D, Eigen::VectorXd::Zero(C.rows())) { }
};

template <template<typename> class InputVec, template<typename> class OutputVec>
class Gain : public LinearSystem<NullVector,InputVec,OutputVec> {
public:
  template<typename ScalarType> using StateVector = NullVector<ScalarType>;
  template<typename ScalarType> using OutputVector = OutputVec<ScalarType>;
  template<typename ScalarType> using InputVector = InputVec<ScalarType>;

  template<typename Derived>
  Gain(const Eigen::MatrixBase<Derived> &D)
          : LinearSystem<NullVector,InputVec,OutputVec>(Eigen::Matrix<double,0,0>(), Eigen::Matrix<double,0,0>(), Eigen::Matrix<double,Eigen::Dynamic,0>(D.rows(),0), D) { }
};


} // end namespace Drake

#endif //DRAKE_LINEARSYSTEM_H
