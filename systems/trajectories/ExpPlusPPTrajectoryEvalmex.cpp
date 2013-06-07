// faster alternative to ExpPlusPPTrajectory.eval() in Matlab
// Michael Kaess, June 2013

#include <mex.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <iostream>

#include "ObjectHandle.h"

using namespace Eigen;
using namespace std;


// Initialization:
//   obj = ExpPlusPPTrajectoryEvalmex(breaks,K,A,alpha,gamma)
// Evaluation:
//   [y,jj] = ExpPlusPPTrajectoryEvalmex(obj, t)
//
// t:      m      vector of evaluation points
//
// breaks: n+1
// K:      d x a
// A:      a x a
// alpha:  a x n
// gamma:  dxn x p - reshape!
//
// y:      p x m
// jj:     m



class Eval {

private:

  const VectorXd m_breaks;
  const MatrixXd m_K, m_A, m_alpha, m_gamma;
  const int m_n, m_d, m_p;

public:

  MatrixXd expm(const MatrixXd& A) {
    MatrixXd F;
    MatrixExponential<MatrixXd>(A).compute(F);
    return F;
  }

  Eval(const VectorXd& breaks, const MatrixXd& K, const MatrixXd& A, const MatrixXd& alpha, const MatrixXd& gamma) : m_breaks(breaks), m_K(K), m_A(A), m_alpha(alpha), m_gamma(gamma), m_n(alpha.cols()), m_d(K.rows()), m_p(gamma.cols()) {}

  int dim() {
    return m_d;
  }

  VectorXd term(int j, double trel) {
    VectorXd trels(m_p);
    trels(0) = (trel<0)?-1.:1.;
    for (int i=1; i<m_p; i++) {
      trels(i) = trels(i-1) * trel; 
    }
    const MatrixXd gammaj = m_gamma.block(j*m_d,0,m_d,m_p);
    return m_K*expm(m_A*trel)*m_alpha.col(j) + gammaj*trels;
  }

  void compute(const VectorXd& t, Map<MatrixXd>& y, Map<MatrixXd>& jj) {
    int m = t.rows();
    for(int k=0; k<m; k++) {
      double tk = t(k);
      // find the right interval
      int j = 0;
      for (int b=1; b<m_n; b++) { // todo: binary search
        if (tk >= m_breaks(b)) j=b;
      }
      double trel = tk - m_breaks(j);
      y.col(k) = term(j,trel);
      jj(k) = j+1; // convert to Matlab convention
    }
  }

};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 5 && nlhs == 1) {

    Map<VectorXd> breaks(mxGetPr(prhs[0]), mxGetNumberOfElements(prhs[0]));
    Map<MatrixXd>      K(mxGetPr(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));
    Map<MatrixXd>      A(mxGetPr(prhs[2]), mxGetM(prhs[2]), mxGetN(prhs[2]));
    Map<MatrixXd>  alpha(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
    Map<MatrixXd>  gamma(mxGetPr(prhs[4]), mxGetM(prhs[4]), mxGetN(prhs[4]));

    Eval *eval = new Eval(breaks, K, A, alpha, gamma);
    ObjectHandle<Eval> *handle = new ObjectHandle<Eval>(eval);
    plhs[0] = handle->to_mex_handle();

  } else {

    if (nrhs != 2) {
      mexErrMsgIdAndTxt("Drake:ExpPlusPPTrajectoryEval:WrongNumberOfInputs","Usage ExpPlusPPTrajectoryEvalmex(t,breaks,K,A,alpha,gamma)");
    }
    if (nlhs != 2) {
      mexErrMsgIdAndTxt("Drake:ExpPlusPPTrajectoryEval:WrongNumberOfOutputs","Returns [y,jj]");
    }

    Eval& eval = get_object<Eval>(prhs[0]);

    Map<VectorXd> t(mxGetPr(prhs[1]), mxGetNumberOfElements(prhs[1]));

    int m = t.rows();
    int d = eval.dim();

    plhs[0] = mxCreateDoubleMatrix(d,m,mxREAL);
    Map<MatrixXd> y(mxGetPr(plhs[0]),d,m);
    plhs[1] = mxCreateDoubleMatrix(m,1,mxREAL);
    Map<MatrixXd> jj(mxGetPr(plhs[1]),m,1);

    eval.compute(t, y, jj);

  }

}
