#ifndef DRAKE_LQR_H
#define DRAKE_LQR_H

#include "drake/core/Core.h"
#include "drake/systems/LinearSystem.h"
#include "drake/util/drakeGradientUtil.h"
#include "drake/util/drakeUtil.h"

namespace Drake {

  template <typename System>
  std::shared_ptr<AffineSystem<NullVector,System::template StateVector,System::template InputVector>> timeInvariantLQR(const System& sys, const typename System::template StateVector<double>& x0, const typename System::template InputVector<double>& u0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    const int num_states = System::template StateVector<double>::RowsAtCompileTime;
    const int num_inputs = System::template InputVector<double>::RowsAtCompileTime;
    assert(!sys.isTimeVarying());
    static_assert(num_states != 0,"This system has no continuous states");
    using namespace std;
    using namespace Eigen;

    // todo: clean this up
    typedef TaylorVar<-1> AutoDiffType;
    VectorXd xu(num_states+num_inputs);
    xu << toEigen(x0), toEigen(u0);
    TaylorVecX xu_taylor = initTaylorVecX(xu);
    typename System::template StateVector<AutoDiffType> x_taylor(xu_taylor.head(num_states));
    typename System::template InputVector<AutoDiffType> u_taylor(xu_taylor.tail(num_inputs));

    auto xdot = autoDiffToGradientMatrix(toEigen(sys.dynamics(AutoDiffType(0),x_taylor,u_taylor)));
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

    return std::make_shared<AffineSystem<NullVector,System::template StateVector,System::template InputVector> >(
                                                           nullmat,Matrix<double,0,num_states>::Zero(),nullvec,
                                                           Matrix<double,num_inputs,0>::Zero(),-K,toEigen(u0)+K*toEigen(x0));
  }

}


#endif //DRAKE_LQR_H
