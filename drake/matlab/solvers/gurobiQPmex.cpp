#include <mex.h>

#include <math.h>
#include <iostream>
#include "drake/solvers/gurobi_qp.h"

using namespace Eigen;
using namespace std;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:gurobiQP:NotEnoughInputs",
                      "Usage [x, status, active] = "
                      "gurobiQP(Q, f[, Ain, bin, Aeq, beq, lb, ub, active])");
  }
  if (nlhs < 1) return;

  GRBenv* env = NULL;
  CGE(GRBloadenv(&env, NULL), env);

  // set solver params
  // (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  // todo: take these as an argument?
  CGE(GRBsetintparam(env, "outputflag", 0), env);
  CGE(GRBsetintparam(env, "method", 2), env);
  CGE(GRBsetintparam(env, "presolve", 0), env);
  CGE(GRBsetintparam(env, "bariterlimit", 20), env);
  CGE(GRBsetintparam(env, "barhomogeneous", 0), env);
  CGE(GRBsetdblparam(env, "barconvtol", 0.0005), env);

  int arg = 0, nblks = 1;

  if (mxIsCell(prhs[arg])) nblks = mxGetNumberOfElements(prhs[arg]);
  MatrixXd* Q = new MatrixXd[nblks];

  vector<MatrixXd*> QblkMat;

  /*
   * NOTE: I am copying memory from the matlab inputs to the MatrixXd structures
   * in the loop below.
   * This could be avoided by passing Map<>* through to fastQP, but the getting
   * all of the templates
   *  right is a pain and I'm out of time.  :)  Will finish it later.
   */

  if (mxIsCell(prhs[arg])) {
    mxArray* QblkDiagCellArray = (mxArray*)prhs[arg++];
    for (int i = 0; i < nblks; i++) {
      mxArray* Qblk = mxGetCell(QblkDiagCellArray, i);
      int m = mxGetM(Qblk), n = mxGetN(Qblk);
      if (m * n == 0)
        continue;
      else if (m == 1 || n == 1)  // then it's a vector
        Q[i] = Map<MatrixXd>(mxGetPr(Qblk), m * n, 1);
      else
        Q[i] = Map<MatrixXd>(mxGetPr(Qblk), m, n);
      QblkMat.push_back(&Q[i]);
    }
    if (QblkMat.size() < 1)
      mexErrMsgIdAndTxt("Drake:FastQP:BadInputs", "Q is empty");
  } else {
    int m = mxGetM(prhs[arg]), n = mxGetN(prhs[arg]);
    if (m * n == 0)
      mexErrMsgIdAndTxt("Drake:FastQP:BadInputs", "Q is empty");
    else if (m == 1 || n == 1)  // then it's a vector
      Q[0] = Map<MatrixXd>(mxGetPr(prhs[arg]), m * n,
                           1);  // always want a column vector
    else
      Q[0] = Map<MatrixXd>(mxGetPr(prhs[arg]), m, n);
    arg++;
    QblkMat.push_back(&Q[0]);
  }

  int nparams = mxGetNumberOfElements(prhs[arg]);

  VectorXd f(nparams);
  memcpy(f.data(), mxGetPr(prhs[arg++]), sizeof(double) * nparams);

  Map<MatrixXd> Aeq(NULL, 0, nparams);
  Map<VectorXd> beq(NULL, 0);
  Map<MatrixXd> Ain(NULL, 0, nparams);
  Map<VectorXd> bin(NULL, 0);
  VectorXd lb(nparams);
  VectorXd ub(nparams);

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0)
    new (&Ain)
        Map<MatrixXd>(mxGetPr(prhs[arg]), mxGetM(prhs[arg]), mxGetN(prhs[arg]));
  arg++;

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0)
    new (&bin)
        Map<VectorXd>(mxGetPr(prhs[arg]), mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0)
    new (&Aeq)
        Map<MatrixXd>(mxGetPr(prhs[arg]), mxGetM(prhs[arg]), mxGetN(prhs[arg]));
  arg++;

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0)
    new (&beq)
        Map<VectorXd>(mxGetPr(prhs[arg]), mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0) {
    if (nparams != static_cast<int>(mxGetNumberOfElements(prhs[arg])))
      mexErrMsgTxt("lb must be the same size as f");
    memcpy(lb.data(), mxGetPr(prhs[arg]), sizeof(double) * lb.rows());
  } else {
    lb = VectorXd::Constant(nparams, -1e8);  // -inf
  }
  arg++;

  if (nrhs > arg && mxGetNumberOfElements(prhs[arg]) > 0) {
    if (nparams != static_cast<int>(mxGetNumberOfElements(prhs[arg])))
      mexErrMsgTxt("ub must be the same size as f");
    memcpy(ub.data(), mxGetPr(prhs[arg]), sizeof(double) * ub.rows());
  } else {
    ub = VectorXd::Constant(nparams, 1e8);  // inf
  }
  arg++;

  set<int> active;
  if (nrhs > arg) {
    double* pact = mxGetPr(prhs[arg]);
    for (int i = 0; i < static_cast<int>(mxGetNumberOfElements(prhs[arg])); i++)
      active.insert((int)pact[i]);
  }

  VectorXd x(f.rows());
  GRBmodel* model =
      gurobiQP(env, QblkMat, f, Aeq, beq, Ain, bin, lb, ub, active, x);

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  memcpy(mxGetPr(plhs[0]), x.data(), sizeof(double) * f.rows());

  int status;
  CGE(GRBgetintattr(model, "Status", &status), env);
  if (nlhs > 1) plhs[1] = mxCreateDoubleScalar((double)status);
  if (nlhs > 2) {
    plhs[2] = mxCreateDoubleMatrix(active.size(), 1, mxREAL);
    int i = 0;
    double* pact = mxGetPr(plhs[2]);
    for (set<int>::iterator iter = active.begin(); iter != active.end(); iter++)
      pact[i++] = (double)*iter;
  }
  GRBfreemodel(model);
}
