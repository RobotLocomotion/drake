// faster alternative to PPTrajectory.eval() in Matlab
// Michael Kaess, June 2013

#include <mex.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

// Initialization:
//   obj = PPTmex(breaks, coefs, order, dims)
// Evaluation:
//   y = PPTmex(obj, 1, t)
//
// t:      1      single evaluation point
//
// breaks: n+1
// coefs:  nxd1xd2 x p
// order:  1       p
// dims:   2       d1, d2
//
// y:      p

class PPTrajectory {
 private:
  VectorXd m_breaks;
  MatrixXd m_coefs;
  const int m_d1, m_d2, m_order;

  int m_cached_idx;

 public:
  PPTrajectory(const VectorXd &breaks, const MatrixXd &coefs, int order, int d1,
               int d2)
      : m_breaks(breaks),
        m_coefs(coefs),
        m_d1(d1),
        m_d2(d2),
        m_order(order),
        m_cached_idx(-1) {}

  int d1() { return m_d1; }

  int d2() { return m_d2; }

  void eval(double t, Map<MatrixXd> &y) {
    if (t < m_breaks(0)) t = m_breaks(0);
    if (t > m_breaks(m_breaks.rows() - 1)) t = m_breaks(m_breaks.rows() - 1);

    Map<VectorXd> r(y.data(), m_d1 * m_d2);  // reshape

    // use cached index if possible
    int idx;
    if (m_cached_idx >= 0 &&
        m_cached_idx < m_breaks.rows()     // valid m_cached_idx?
        && t < m_breaks[m_cached_idx + 1]  // still in same interval?
        && ((m_cached_idx == 0) ||
            (t >= m_breaks[m_cached_idx]))) {  // left up to -infinity
      idx = m_cached_idx;
    } else {
      // search for the correct interval
      idx = static_cast<int>(m_breaks.rows()) - 2;
      // todo: binary search
      for (int b = static_cast<int>(m_breaks.rows()) - 2; b >= 1;
           b--) {  // ignore first and last break
        if (t < m_breaks(b)) idx = b - 1;
      }
      m_cached_idx = idx;
    }

    int base = idx * m_d1 * m_d2;
    double local = t - m_breaks[idx];
    r = m_coefs.block(base, 0, m_d1 * m_d2, 1);
    for (int i = 2; i <= m_order; i++) {
      r = local * r + m_coefs.block(base, i - 1, m_d1 * m_d2, 1);
    }
  }
};

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 4 && nlhs == 1) {
    // create object
    Map<VectorXd> breaks(mxGetPr(prhs[0]), mxGetNumberOfElements(prhs[0]));
    Map<MatrixXd> coefs(mxGetPr(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));
    int order = static_cast<int>(mxGetScalar(prhs[2]));
    Map<VectorXd> dims(mxGetPr(prhs[3]), mxGetNumberOfElements(prhs[3]));

    int d1 = static_cast<int>(dims(0));
    int d2 = 1;
    if (dims.rows() > 1) d2 = static_cast<int>(dims(1));
    PPTrajectory *ppt = new PPTrajectory(breaks, coefs, order, d1, d2);
    mxClassID cid;
    if (sizeof(ppt) == 4)
      cid = mxUINT32_CLASS;
    else if (sizeof(ppt) == 8)
      cid = mxUINT64_CLASS;
    else
      mexErrMsgIdAndTxt("Drake:PPTmex:PointerSize",
                        "Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1, 1, cid, mxREAL);
    memcpy(mxGetData(plhs[0]), &ppt, sizeof(ppt));

  } else {
    // retrieve object
    PPTrajectory *ppt = NULL;
    if (nrhs == 0 || !mxIsNumeric(prhs[0]) ||
        mxGetNumberOfElements(prhs[0]) != 1)
      mexErrMsgIdAndTxt("Drake:PPTmex:BadInputs",
                        "the first argument should be the mex_ptr");
    memcpy(&ppt, mxGetData(prhs[0]), sizeof(ppt));

    if (nrhs == 1) {
      // delete object
      delete ppt;

    } else {
      // eval() function call
      if (nrhs < 2) {
        mexErrMsgIdAndTxt(
            "Drake:PPTmex:WrongNumberOfInputs",
            "Usage obj = PPTmex(breaks, coefs, order, dims) "
            "or y = PPTmex(obj, t)");
      }

      double *command = mxGetPr(prhs[1]);

      switch ((int)(*command)) {
        case 1: {  // eval
          double t = mxGetScalar(prhs[2]);
          int d1 = ppt->d1();
          int d2 = ppt->d2();
          plhs[0] = mxCreateDoubleMatrix(d1, d2, mxREAL);
          Map<MatrixXd> y(mxGetPr(plhs[0]), d1, d2);
          ppt->eval(t, y);
          break;
        }

        default:
          mexErrMsgIdAndTxt("Drake:PPTmex:UnknownCommand", "Check arguments");
      }
    }
  }
}
