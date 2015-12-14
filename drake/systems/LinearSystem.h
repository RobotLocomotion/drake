#ifndef DRAKE_LINEARSYSTEM_H
#define DRAKE_LINEARSYSTEM_H

#include "System.h"
#include <drakeSystem_export.h>
// TODO: exports

namespace Drake {

/// Implements
///   xcdot = Ac*x + Bc*u + xcdot0
///   xdn = Ad*x + Bd*u + xdn0
///   y = C*x + D*u + y0

template <template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool _isDirectFeedthrough = true >
class AffineSystem : public System< AffineSystem<StateVector,InputVector,OutputVector,_isDirectFeedthrough>, StateVector, InputVector, OutputVector, false, _isDirectFeedthrough> {
public:
  typedef System< AffineSystem<StateVector,InputVector,OutputVector,_isDirectFeedthrough>, StateVector, InputVector, OutputVector, false, _isDirectFeedthrough> BaseType;
  using BaseType::num_states;
  using BaseType::num_inputs;
  using BaseType::num_outputs;

  AffineSystem(const Eigen::Matrix<double,num_states,num_states>& A,const Eigen::Matrix<double,num_states,num_inputs>& B,const Eigen::Matrix<double,num_states,1>& xdot0,
               const Eigen::Matrix<double,num_outputs,num_states>& C,const Eigen::Matrix<double,num_outputs,num_inputs>& D,const Eigen::Matrix<double,num_outputs,1>& y0)
          : A(A),B(B),C(C),D(D),xdot0(xdot0),y0(y0) {
    assert(_isDirectFeedthrough || D.isZero() );
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamicsImplementation(const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
    using namespace Eigen;
    StateVector<ScalarType> xdot = A*static_cast<Matrix<ScalarType,num_states,1> >(x) + B*static_cast<Matrix<ScalarType,num_inputs,1> >(u) + xdot0;
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> outputImplementation(const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const  {
    using namespace Eigen;
    OutputVector<ScalarType> y = C*static_cast<Matrix<ScalarType,num_states,1> >(x) + D*static_cast<Matrix<ScalarType,num_inputs,1> >(u) + y0;
    return y;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> outputImplementation(const InputVector<ScalarType>& u) const  {
//    static_assert(num_states!=0,"This system has internal state, but somehow we called output(u).");
    using namespace Eigen;
    Matrix<ScalarType,num_outputs,1> y = D*static_cast<Matrix<ScalarType,num_inputs,1> >(u) + y0;

    return y;
  }


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
