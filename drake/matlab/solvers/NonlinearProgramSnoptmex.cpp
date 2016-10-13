#include <mex.h>
#include <matrix.h>

#include <cstdlib>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
}

using namespace std;

static char* snopt_userfun_name;

// NOTE: all snopt calls will use this shared memory... so this code is NOT
// THREAD SAFE
static unique_ptr<snopt::doublereal[]> rw;
static unique_ptr<snopt::integer[]> iw;
static unique_ptr<char[]> cw;
static snopt::integer lenrw = 0;
static snopt::integer leniw = 0;
static snopt::integer lencw = 0;

static int snopt_userfun(snopt::integer* Status, snopt::integer* n,
                         snopt::doublereal x[], snopt::integer* needF,
                         snopt::integer* neF, snopt::doublereal F[],
                         snopt::integer* needG, snopt::integer* neG,
                         snopt::doublereal G[], char* cu, snopt::integer* lencu,
                         snopt::integer iu[], snopt::integer* leniu,
                         snopt::doublereal ru[], snopt::integer* lenru) {
  mxArray* snopt_userfun_plhs[2];
  mxArray* snopt_userfun_prhs[1];
  snopt_userfun_prhs[0] = mxCreateDoubleMatrix(*n, 1, mxREAL);
  for (snopt::integer i = 0; i < *n; i++) {
    *(mxGetPr(snopt_userfun_prhs[0]) + i) = static_cast<double>(x[i]);
  }
  mexCallMATLAB(2, snopt_userfun_plhs, 1, snopt_userfun_prhs,
                snopt_userfun_name);
  for (snopt::integer i = 0; i < *neF; i++) {
    F[i] =
        static_cast<snopt::doublereal>(*(mxGetPr(snopt_userfun_plhs[0]) + i));
  }
  for (snopt::integer i = 0; i < *neG; i++) {
    G[i] =
        static_cast<snopt::doublereal>(*(mxGetPr(snopt_userfun_plhs[1]) + i));
  }
  mxDestroyArray(snopt_userfun_prhs[0]);
  return 0;
}

void mysnseti(const char* strOpt, snopt::integer val, snopt::integer* iPrint,
              snopt::integer* iSumm, snopt::integer* INFO_snopt, char* cw,
              snopt::integer* lencw, snopt::integer* iw, snopt::integer* leniw,
              snopt::doublereal* rw, snopt::integer* lenrw) {
  snopt::integer strOpt_len = static_cast<snopt::integer>(strlen(strOpt));
  snopt::snseti_(strOpt, &val, iPrint, iSumm, INFO_snopt, cw, lencw, iw, leniw,
                 rw, lenrw, strOpt_len, 8 * (*lencw));
}
void mysnsetr(const char* strOpt, snopt::doublereal val, snopt::integer* iPrint,
              snopt::integer* iSumm, snopt::integer* INFO_snopt, char* cw,
              snopt::integer* lencw, snopt::integer* iw, snopt::integer* leniw,
              snopt::doublereal* rw, snopt::integer* lenrw) {
  snopt::integer strOpt_len = static_cast<snopt::integer>(strlen(strOpt));
  snopt::snsetr_(strOpt, &val, iPrint, iSumm, INFO_snopt, cw, lencw, iw, leniw,
                 rw, lenrw, strOpt_len, 8 * (*lencw));
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 13 && nlhs != 5) {
    mexErrMsgIdAndTxt(
        "Drake:NonlinearProgramSnoptmex:InvalidInputs",
        "The usage is [x, F, info, xmul, Fmul] = "
        "NonlinearProgramSnoptmex(x, xlow, xupp, Flow, Fupp, userfun,"
        "ObjAdd, ObjRow, A, iAfun, jAvar, iGfun, jGvar, options)");
  }

  if (lenrw == 0) {  // then initialize (sninit needs some default allocation)
    lenrw = 500000;
    rw.reset(new snopt::doublereal[lenrw]);
    leniw = 500000;
    iw.reset(new snopt::integer[leniw]);
    lencw = 500;
    cw.reset(new char[8 * lencw]);
  }

  snopt::integer nx = static_cast<snopt::integer>(mxGetM(prhs[0]));
  snopt::integer nF = static_cast<snopt::integer>(mxGetM(prhs[3]));
  snopt::doublereal* x = new snopt::doublereal[nx];
  snopt::doublereal* xlow = new snopt::doublereal[nx];
  snopt::doublereal* xupp = new snopt::doublereal[nx];
  double* px = mxGetPr(prhs[0]), * pxlow = mxGetPr(prhs[1]),
          * pxupp = mxGetPr(prhs[2]);
  for (int i = 0; i < nx; i++) {
    x[i] = static_cast<snopt::doublereal>(*px++);
    xlow[i] = static_cast<snopt::doublereal>(*pxlow++);
    xupp[i] = static_cast<snopt::doublereal>(*pxupp++);
  }
  snopt::doublereal* Flow = new snopt::doublereal[nF];
  snopt::doublereal* Fupp = new snopt::doublereal[nF];
  double* pFlow = mxGetPr(prhs[3]), * pFupp = mxGetPr(prhs[4]);
  for (int i = 0; i < nF; i++) {
    Flow[i] = static_cast<snopt::doublereal>(*pFlow++);
    Fupp[i] = static_cast<snopt::doublereal>(*pFupp++);
  }
  int snopt_userfun_name_len =
      static_cast<int>(mxGetNumberOfElements(prhs[5])) + 1;
  snopt_userfun_name = new char[snopt_userfun_name_len];
  int userfun_name_status =
      mxGetString(prhs[5], snopt_userfun_name, snopt_userfun_name_len);
  if (userfun_name_status != 0) {
    mexErrMsgIdAndTxt(
        "Drake:NonlinearProgramSnoptmex:InvalidInputs",
        "userfun should be a string for NonlinearProgramSnoptmex");
  }
  snopt::integer lenA =
      static_cast<snopt::integer>(mxGetNumberOfElements(prhs[8]));
  snopt::doublereal* A = new snopt::doublereal[lenA];
  snopt::integer* iAfun = new snopt::integer[lenA];
  snopt::integer* jAvar = new snopt::integer[lenA];
  double* pA = mxGetPr(prhs[8]), * piAfun = mxGetPr(prhs[9]),
          * pjAvar = mxGetPr(prhs[10]);
  for (snopt::integer i = 0; i < lenA; i++) {
    A[i] = static_cast<snopt::doublereal>(*pA++);
    iAfun[i] = static_cast<snopt::integer>(*piAfun++);
    jAvar[i] = static_cast<snopt::integer>(*pjAvar++);
  }
  snopt::integer lenG =
      static_cast<snopt::integer>(mxGetNumberOfElements(prhs[11]));
  snopt::integer* iGfun = new snopt::integer[lenG];
  snopt::integer* jGvar = new snopt::integer[lenG];
  double* piGfun = mxGetPr(prhs[11]), * pjGvar = mxGetPr(prhs[12]);
  for (snopt::integer i = 0; i < lenG; i++) {
    iGfun[i] = static_cast<snopt::integer>(*piGfun++);
    jGvar[i] = static_cast<snopt::integer>(*pjGvar++);
  }
  snopt::integer INFO_snopt;
  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char xnames[8 * 1];  // should match nxname
  char Fnames[8 * 1];  // should match nFname
  char Prob[200] = "drake.out";

  snopt::integer iSumm = -1;
  snopt::integer iPrint = -1;

  snopt::integer nS, nInf;
  snopt::doublereal sInf;
  mxArray* pprint_name = mxGetField(prhs[13], 0, "print");
  snopt::integer print_file_name_len =
      static_cast<snopt::integer>(mxGetNumberOfElements(pprint_name)) + 1;
  char* print_file_name = NULL;
  if (print_file_name_len != 1) {
    iPrint = 9;
    print_file_name = new char[print_file_name_len];
    mxGetString(pprint_name, print_file_name, print_file_name_len);
    snopt::snopenappend_(&iPrint, print_file_name, &INFO_snopt,
                         print_file_name_len);

    // mysnseti("Major print level",
    //   static_cast<snopt::integer>(11),&iPrint,&iSumm,&INFO_snopt,
    //   cw,&lencw, iw,&leniw, rw,&lenrw);
    // mysnseti("Print file", iPrint,&iPrint,&iSumm,&INFO_snopt,
    //   cw,&lencw, iw,&leniw, rw,&lenrw);
  }

  snopt::integer minrw, miniw, mincw;
  snopt::sninit_(&iPrint, &iSumm, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);
  snopt::snmema_(&INFO_snopt, &nF, &nx, &nxname, &nFname, &lenA, &lenG, &mincw,
                 &miniw, &minrw, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);
  if (minrw > lenrw) {
    // mexPrintf("reallocation rw with size %d\n", minrw);
    lenrw = minrw;
    rw.reset(new snopt::doublereal[lenrw]);
  }
  if (miniw > leniw) {
    // mexPrintf("reallocation iw with size %d\n", miniw);
    leniw = miniw;
    iw.reset(new snopt::integer[leniw]);
  }
  if (mincw > lencw) {
    // mexPrintf("reallocation cw with size %d\n", mincw);
    lencw = mincw;
    cw.reset(new char[8 * lencw]);
  }
  snopt::sninit_(&iPrint, &iSumm, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);

  snopt::integer Cold = 0;
  snopt::doublereal* xmul = new snopt::doublereal[nx];
  snopt::integer* xstate = new snopt::integer[nx];
  for (snopt::integer i = 0; i < nx; i++) {
    xstate[i] = 0;
  }

  snopt::doublereal* F = new snopt::doublereal[nF];
  snopt::doublereal* Fmul = new snopt::doublereal[nF];
  snopt::integer* Fstate = new snopt::integer[nF];
  for (snopt::integer i = 0; i < nF; i++) {
    Fstate[i] = 0;
  }
  snopt::doublereal ObjAdd =
      static_cast<snopt::doublereal>(mxGetScalar(prhs[6]));
  snopt::integer ObjRow = static_cast<snopt::integer>(mxGetScalar(prhs[7]));

  mysnseti("Derivative option", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                    prhs[13], 0, "DerivativeOption"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Major iterations limit",
           static_cast<snopt::integer>(
               *mxGetPr(mxGetField(prhs[13], 0, "MajorIterationsLimit"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Minor iterations limit",
           static_cast<snopt::integer>(
               *mxGetPr(mxGetField(prhs[13], 0, "MinorIterationsLimit"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnsetr("Major optimality tolerance",
           static_cast<snopt::doublereal>(
               *mxGetPr(mxGetField(prhs[13], 0, "MajorOptimalityTolerance"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnsetr("Major feasibility tolerance",
           static_cast<snopt::doublereal>(
               *mxGetPr(mxGetField(prhs[13], 0, "MajorFeasibilityTolerance"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnsetr("Minor feasibility tolerance",
           static_cast<snopt::doublereal>(
               *mxGetPr(mxGetField(prhs[13], 0, "MinorFeasibilityTolerance"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Superbasics limit", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                    prhs[13], 0, "SuperbasicsLimit"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Verify level", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                               prhs[13], 0, "VerifyLevel"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Iterations Limit", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                   prhs[13], 0, "IterationsLimit"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Scale option", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                               prhs[13], 0, "ScaleOption"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("New basis file", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                 prhs[13], 0, "NewBasisFile"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Old basis file", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                 prhs[13], 0, "OldBasisFile"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnseti("Backup basis file", static_cast<snopt::integer>(*mxGetPr(mxGetField(
                                    prhs[13], 0, "BackupBasisFile"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);
  mysnsetr("Linesearch tolerance",
           static_cast<snopt::doublereal>(
               *mxGetPr(mxGetField(prhs[13], 0, "LinesearchTolerance"))),
           &iPrint, &iSumm, &INFO_snopt, cw.get(), &lencw, iw.get(), &leniw,
           rw.get(), &lenrw);

  snopt::snopta_(&Cold, &nF, &nx, &nxname, &nFname, &ObjAdd, &ObjRow, Prob,
                 snopt_userfun, iAfun, jAvar, &lenA, &lenA, A, iGfun, jGvar,
                 &lenG, &lenG, xlow, xupp, xnames, Flow, Fupp, Fnames, x,
                 xstate, xmul, F, Fstate, Fmul, &INFO_snopt, &mincw, &miniw,
                 &minrw, &nS, &nInf, &sInf, cw.get(), &lencw, iw.get(), &leniw,
                 rw.get(), &lenrw, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, npname, 8 * nxname, 8 * nFname, 8 * lencw, 8 * lencw);
  plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
  plhs[3] = mxCreateDoubleMatrix(nx, 1, mxREAL);
  for (int i = 0; i < nx; i++) {
    *(mxGetPr(plhs[0]) + i) = static_cast<double>(x[i]);
    *(mxGetPr(plhs[3]) + i) = static_cast<double>(xmul[i]);
  }
  plhs[1] = mxCreateDoubleMatrix(nF, 1, mxREAL);
  plhs[4] = mxCreateDoubleMatrix(nF, 1, mxREAL);
  for (int i = 0; i < nF; i++) {
    *(mxGetPr(plhs[1]) + i) = static_cast<double>(F[i]);
    *(mxGetPr(prhs[4]) + i) = static_cast<double>(Fmul[i]);
  }
  plhs[2] = mxCreateDoubleScalar(static_cast<double>(INFO_snopt));
  delete[] x;
  delete[] xlow;
  delete[] xupp;
  delete[] Flow;
  delete[] Fupp;
  delete[] snopt_userfun_name;
  delete[] A;
  delete[] iAfun;
  delete[] jAvar;
  delete[] iGfun;
  delete[] jGvar;
  delete[] xmul;
  delete[] xstate;
  delete[] F;
  delete[] Fmul;
  delete[] Fstate;

  if (print_file_name_len != 1) {
    snopt::snclose_(&iPrint);
    delete[] print_file_name;
  }
}
