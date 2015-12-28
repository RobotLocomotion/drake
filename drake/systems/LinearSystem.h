#ifndef DRAKE_LINEARSYSTEM_H
#define DRAKE_LINEARSYSTEM_H

#include "System.h"

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
  const static int num_states = StateVector<double>::RowsAtCompileTime;
  const static int num_inputs = InputVector<double>::RowsAtCompileTime;
  const static int num_outputs = OutputVector<double>::RowsAtCompileTime;

  AffineSystem(const Eigen::Matrix<double,num_states,num_states>& A,const Eigen::Matrix<double,num_states,num_inputs>& B,const Eigen::Matrix<double,num_states,1>& xdot0,
               const Eigen::Matrix<double,num_outputs,num_states>& C,const Eigen::Matrix<double,num_outputs,num_inputs>& D,const Eigen::Matrix<double,num_outputs,1>& y0)
          : A(A),B(B),C(C),D(D),xdot0(xdot0),y0(y0) { }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
    StateVector<ScalarType> xdot = A*toEigen(x) + B*toEigen(u) + xdot0;
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const  {
    OutputVector<ScalarType> y = C*toEigen(x) + D*toEigen(u) + y0;
    return y;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return !D.isZero(); }

private:
  Eigen::Matrix<double,num_states,num_states> A;
  Eigen::Matrix<double,num_states,num_inputs> B;
  Eigen::Matrix<double,num_outputs,num_states> C;
  Eigen::Matrix<double,num_outputs,num_inputs> D;
  Eigen::Matrix<double,num_states,1> xdot0;
  Eigen::Matrix<double,num_outputs,1> y0;
};


/*
class LinearSystem : public AffineSystem {
public:
  LinearSystem(const std::string& name,
               const Eigen::MatrixXd& _Ac,const Eigen::MatrixXd& _Bc,
               const Eigen::MatrixXd& _Ad,const Eigen::MatrixXd& _Bd,
               const Eigen::MatrixXd& _C,const Eigen::MatrixXd& _D);
};
*/

} // end namespace Drake

#endif //DRAKE_LINEARSYSTEM_H
