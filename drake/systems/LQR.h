#ifndef DRAKE_LQR_H
#define DRAKE_LQR_H

#include "Core.h"
#include "LinearSystem.h"
#include "drakeGradientUtil.h"
#include "drakeUtil.h"

namespace Drake {

  template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool isTimeVarying, bool isDirectFeedthrough>
  std::shared_ptr<AffineSystem<UnusedVector,StateVector,InputVector> > timeInvariantLQR(const System<Derived,StateVector,InputVector,OutputVector,isTimeVarying,isDirectFeedthrough>& sys, const StateVector<double>& x0, const InputVector<double>& u0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    const unsigned int num_states = VectorTraits<StateVector<double> >::RowsAtCompileTime;
    const unsigned int num_inputs = VectorTraits<InputVector<double> >::RowsAtCompileTime;
    static_assert(!isTimeVarying,"timeInvariantLQR only makes sense for time-invariant systems");
    static_assert(num_states != 0,"This system has no continuous states");
    using namespace std;
    using namespace Eigen;

    // todo: clean this up
    typedef TaylorVar<-1> AutoDiffType;
    VectorXd xu(num_states+num_inputs);
    xu << static_cast<Eigen::Matrix<double,num_states,1> >(x0), static_cast<Eigen::Matrix<double,num_inputs,1> >(u0);
    TaylorVecX xu_taylor = initTaylorVecX(xu);
    StateVector<AutoDiffType> x_taylor(xu_taylor.head(num_states));
    InputVector<AutoDiffType> u_taylor(xu_taylor.tail(num_inputs));

    auto xdot = autoDiffToGradientMatrix(static_cast< Matrix<AutoDiffType,num_states,1> >(sys.dynamics(x_taylor,u_taylor)));
    auto A = xdot.leftCols(num_states);
    auto B = xdot.rightCols(num_inputs);

    Eigen::MatrixXd K(num_inputs,num_states), S(num_states,num_states);
    lqr(A, B, Q, R, K, S);

//    cout << "K = " << K << endl;
//    cout << "S = " << S << endl;

    // todo: return the linear system with the affine transform.  But for now, just give the affine controller:

    // u = u0 - K(x-x0)
    Matrix<double,0,0> nullmat;
    Matrix<double,0,1> nullvec;

    return std::make_shared<AffineSystem<UnusedVector,StateVector,InputVector> >(
                                                           nullmat,Matrix<double,0,num_states>::Zero(),nullvec,
                                                           Matrix<double,num_inputs,0>::Zero(),-K,static_cast<Matrix<double,num_inputs,1> >(u0)+K*static_cast<Matrix<double,num_states,1> >(x0));
  }

}


#endif //DRAKE_LQR_H
