// faster alternative to ExpPlusPPTrajectory.eval() in Matlab
// Michael Kaess, June 2013

#include <mex.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <iostream>

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

  Eval(const VectorXd& breaks, const MatrixXd& K, const MatrixXd& A, const MatrixXd& alpha, const MatrixXd& gamma) : m_breaks(breaks), m_K(K), m_A(A), m_alpha(alpha), m_gamma(gamma), m_n(static_cast<int>(alpha.cols())), m_d(static_cast<int>(K.rows())), m_p(static_cast<int>(gamma.cols())) {}

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
    int m = static_cast<int>(t.rows());
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

    // create object
    Map<VectorXd> breaks(mxGetPr(prhs[0]), mxGetNumberOfElements(prhs[0]));
    Map<MatrixXd>      K(mxGetPr(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));
    Map<MatrixXd>      A(mxGetPr(prhs[2]), mxGetM(prhs[2]), mxGetN(prhs[2]));
    Map<MatrixXd>  alpha(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
    Map<MatrixXd>  gamma(mxGetPr(prhs[4]), mxGetM(prhs[4]), mxGetN(prhs[4]));

    Eval *eval = new Eval(breaks, K, A, alpha, gamma);
    mxClassID cid;
    if (sizeof(eval)==4) cid = mxUINT32_CLASS;
    else if (sizeof(eval)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:ExpPlusPPTmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&eval,sizeof(eval));
    
    //    mexPrintf("constructor\n"); mexCallMATLAB(0,NULL,0,NULL,"drawnow");

  } else {

    // retrieve object
    Eval *eval = NULL;
    if (nrhs==0 || !mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
      mexErrMsgIdAndTxt("Drake:ExpPlusPPTmex:BadInputs","the first argument should be the mex_ptr");
    memcpy(&eval,mxGetData(prhs[0]),sizeof(eval));

    if (nrhs == 1) {
      //      mexPrintf("delete\n"); mexCallMATLAB(0,NULL,0,NULL,"drawnow");

      // delete object   
      if (eval)
	delete eval;

    } else {

      //      mexPrintf("eval\n"); mexCallMATLAB(0,NULL,0,NULL,"drawnow");

      // eval() function call
      if (nrhs != 2 || nlhs != 2) {
        mexErrMsgIdAndTxt("Drake:ExpPlusPPTmex:WrongNumberOfInputs","Usage obj = ExpPlusPPTmex(breaks,K,A,alpha,gamma) or [y,jj] = ExpPlusPPTmex(obj,t)");
      }

      Map<VectorXd> t(mxGetPr(prhs[1]), mxGetNumberOfElements(prhs[1]));

      int m = static_cast<int>(t.rows());
      int d = eval->dim();

      plhs[0] = mxCreateDoubleMatrix(d,m,mxREAL);
      Map<MatrixXd> y(mxGetPr(plhs[0]),d,m);
      plhs[1] = mxCreateDoubleMatrix(m,1,mxREAL);
      Map<MatrixXd> jj(mxGetPr(plhs[1]),m,1);

      eval->compute(t, y, jj);

    }

  }

}
