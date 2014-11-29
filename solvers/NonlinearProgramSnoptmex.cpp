#include "mex.h"
#include "matrix.h"
#include <cstdlib>

#include <cstdio>
#include <cstring>
#include <iostream>

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
#include "snoptProblem.hh"
}

using namespace std;

static char* snopt_userfun_name;

static int snopt_userfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru)
{
  mxArray* snopt_userfun_plhs[2];
  mxArray* snopt_userfun_prhs[1];
  snopt_userfun_prhs[0] = mxCreateDoubleMatrix(*n,1,mxREAL);
  for(snopt::integer i = 0;i<*n;i++)
  {
    *(mxGetPr(snopt_userfun_prhs[0])+i) = static_cast<double>(x[i]);
  }
  mexCallMATLAB(2,snopt_userfun_plhs,1,snopt_userfun_prhs,snopt_userfun_name);
  for(snopt::integer i = 0;i<*neF;i++)
  {
    F[i] = static_cast<snopt::doublereal>( *(mxGetPr(snopt_userfun_plhs[0])+i));
  }
  for(snopt::integer i = 0;i<*neG;i++)
  {
    G[i] = static_cast<snopt::doublereal>( *(mxGetPr(snopt_userfun_plhs[1])+i));
  }
  mxDestroyArray(snopt_userfun_prhs[0]);
  return 0;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 13 && nlhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:NonlinearProgramSnoptmex:InvalidInputs","The usage is [x,F,info,xmul,Fmul] = NonlinearProgramSnoptmex(x,xlow,xupp,Flow,Fupp,userfun,ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar,options)");
  }
  snopt::integer nx = static_cast<snopt::integer>(mxGetM(prhs[0]));
  snopt::integer nF = static_cast<snopt::integer>(mxGetM(prhs[3]));
  snopt::doublereal* x    = new snopt::doublereal[nx];
  snopt::doublereal* xlow = new snopt::doublereal[nx];
  snopt::doublereal* xupp = new snopt::doublereal[nx];
  double* px = mxGetPr(prhs[0]), *pxlow = mxGetPr(prhs[1]), *pxupp = mxGetPr(prhs[2]);
  for(snopt::integer i = 0;i<nx;i++)
  {
    x[i] = static_cast<snopt::doublereal>(*px++);
    xlow[i] = static_cast<snopt::doublereal>(*pxlow++);
    xupp[i] = static_cast<snopt::doublereal>(*pxupp++);
  }
  snopt::doublereal* Flow = new snopt::doublereal[nF];
  snopt::doublereal* Fupp = new snopt::doublereal[nF];
  double* pFlow = mxGetPr(prhs[3]), *pFupp = mxGetPr(prhs[4]);
  for(snopt::integer i = 0;i<nF;i++)
  {
    Flow[i] = static_cast<snopt::doublereal>(*pFlow++);
    Fupp[i] = static_cast<snopt::doublereal>(*pFupp++);
  }
  int snopt_userfun_name_len = static_cast<int>(mxGetNumberOfElements(prhs[5]))+1;
  snopt_userfun_name = new char[snopt_userfun_name_len];
  int userfun_name_status = mxGetString(prhs[5],snopt_userfun_name,snopt_userfun_name_len);
  if(userfun_name_status != 0)
  {
    mexErrMsgIdAndTxt("Drake:NonlinearProgramSnoptmex:InvalidInputs","userfun should be a string for NonlinearProgramSnoptmex");
  }
  snopt::integer lenA = static_cast<snopt::integer>(mxGetNumberOfElements(prhs[8]));
  snopt::doublereal* A     = new snopt::doublereal[lenA];
  snopt::integer*    iAfun = new snopt::integer[lenA];
  snopt::integer*    jAvar = new snopt::integer[lenA];
  double* pA = mxGetPr(prhs[8]), *piAfun = mxGetPr(prhs[9]), *pjAvar = mxGetPr(prhs[10]);
  for(snopt::integer i = 0;i<lenA;i++)
  {
    A[i] = static_cast<snopt::doublereal>(*pA++);
    iAfun[i] = static_cast<snopt::integer>(*piAfun++);
    jAvar[i] = static_cast<snopt::integer>(*pjAvar++);
  }
  snopt::integer lenG = static_cast<snopt::integer>(mxGetNumberOfElements(prhs[11]));
  snopt::integer* iGfun = new snopt::integer[lenG];
  snopt::integer* jGvar = new snopt::integer[lenG];
  double *piGfun = mxGetPr(prhs[11]), *pjGvar = mxGetPr(prhs[12]);
  for(snopt::integer i = 0;i<lenG;i++)
  {
    iGfun[i] = static_cast<snopt::integer>(*piGfun++);
    jGvar[i] = static_cast<snopt::integer>(*pjGvar++);
  }
  snopt::integer INFO_snopt;
  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char xnames[8*1];  // should match nxname
  char Fnames[8*1];  // should match nFname
  char Prob[200]="";

  const int DEFAULT_LENRW = 100000;
  const int DEFAULT_LENIW = 50000;
  const int DEFAULT_LENCW = 500;
  snopt::integer minrw,miniw,mincw;
  snopt::integer lenrw = DEFAULT_LENRW, leniw = DEFAULT_LENIW, lencw = DEFAULT_LENCW;
  snopt::doublereal rw_static[DEFAULT_LENRW]; // = (snopt::doublereal*) std::calloc(lenrw,sizeof(snopt::doublereal));
  snopt::integer iw_static[DEFAULT_LENIW]; // = (snopt::integer*) std::calloc(leniw,sizeof(snopt::integer));
  char cw_static[8*DEFAULT_LENCW];
  snopt::doublereal *rw = rw_static;
  snopt::integer *iw = iw_static;
  char* cw = cw_static;

  snopt::snmema_(&INFO_snopt,&nF,&nx,&nxname,&nFname,&lenA,&lenG,&mincw,&miniw,&minrw,cw,&lencw,iw,&leniw,rw,&lenrw,8*lencw);
  if (minrw>lenrw) {
    //    mexPrintf("reallocation rw with size %d\n",minrw);
    lenrw = minrw;
    rw = new snopt::doublereal[lenrw];
  }
  if (miniw>leniw) {
    //    mexPrintf("reallocation iw with size %d\n",miniw);
    leniw = miniw;
    iw = new snopt::integer[leniw];
  }
  if (mincw>lencw) {
    //    mexPrintf("reallocation cw with size %d\n",mincw);
    lencw = mincw;
    cw = new char[8*lencw];
  }
  
  snopt::integer Cold = 0;
  snopt::doublereal *xmul = new snopt::doublereal[nx];
  snopt::integer *xstate = new snopt::integer[nx];
  for(snopt::integer i = 0;i<nx;i++)
  {
    xstate[i] = 0;
  }

  snopt::doublereal *F      = new snopt::doublereal[nF];
  snopt::doublereal *Fmul   = new snopt::doublereal[nF];
  snopt::integer    *Fstate = new snopt::integer[nF];
  for(snopt::integer i = 0;i<nF;i++)
  {
    Fstate[i] = 0;
  }
  snopt::doublereal ObjAdd = static_cast<snopt::doublereal>(mxGetScalar(prhs[6]));
  snopt::integer ObjRow = static_cast<snopt::integer>(mxGetScalar(prhs[7]));


  snopt::integer iSumm = -1;
  snopt::integer iPrint = 15;

  snopt::integer nS,nInf;
  snopt::doublereal sInf;
  snopt::integer strOpt_len;

  mxArray* pprint_name = mxGetField(prhs[13],0,"print");
  int print_file_name_len = static_cast<snopt::integer>(mxGetNumberOfElements(pprint_name))+1;
  if(print_file_name_len != 0)
  {
    char* print_file_name = new char[print_file_name_len];
    char strOpt10[200] = "Major print level";
    strOpt_len = static_cast<snopt::integer>(strlen(strOpt10));
    snopt::integer major_print_level = 11;
    snopt::snseti_(strOpt10,&major_print_level,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);
    mxGetString(pprint_name,print_file_name,print_file_name_len);
    snopt::snopenappend_(&iPrint,print_file_name,&INFO_snopt,print_file_name_len);
    char strOpt11[200] = "Print file";
    strOpt_len = static_cast<snopt::integer>(strlen(strOpt11));
    snopt::snseti_(strOpt11,&iPrint,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);
    delete[] print_file_name;
  }

  snopt::sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*lencw);
  char strOpt1[200] = "Derivative option";
  snopt::integer derivative_option = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"DerivativeOption")));
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt1));
  snopt::snseti_(strOpt1,&derivative_option,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt2[200] = "Major iterations limit";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt2));
  snopt::integer major_iterations_limit = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MajorIterationsLimit")));
  snopt::snseti_(strOpt2,&major_iterations_limit,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt3[200] = "Minor iterations limit";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt3));
  snopt::integer minor_iterations_limit = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MinorIterationsLimit")));
  snopt::snseti_(strOpt3,&minor_iterations_limit,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt4[200] = "Major optimality tolerance";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt4));
  snopt::doublereal major_optimality_tolerance= static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorOptimalityTolerance")));
  snopt::snsetr_(strOpt4,&major_optimality_tolerance,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt5[200] = "Major feasibility tolerance";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt5));
  snopt::doublereal major_feasibility_tolerance= static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorFeasibilityTolerance")));
  snopt::snsetr_(strOpt5,&major_feasibility_tolerance,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt6[200] = "Minor feasibility tolerance";
  strOpt_len = strlen(strOpt6);
  snopt::doublereal minor_feasibility_tolerance= static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MinorFeasibilityTolerance")));
  snopt::snsetr_(strOpt6,&minor_feasibility_tolerance,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt7[200] = "Superbasics limit";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt7));
  snopt::integer superbasics_limit = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"SuperbasicsLimit")));
  snopt::snseti_(strOpt7,&superbasics_limit,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt8[200] = "Verify level";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt8));
  snopt::integer verify_level = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"VerifyLevel")));
  snopt::snseti_(strOpt8,&verify_level,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt9[200] = "Iterations Limit";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt9));
  snopt::integer iterations_limit = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"IterationsLimit")));
  snopt::snseti_(strOpt9,&iterations_limit,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt11[200] = "Scale option";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt11));
  snopt::integer scale_option= static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"ScaleOption")));
  snopt::snseti_(strOpt11,&scale_option,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt12[200] = "New basis file";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt12));
  snopt::integer new_basis_file = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"NewBasisFile")));
  snopt::snseti_(strOpt12,&new_basis_file,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt13[200] = "Old basis file";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt13));
  snopt::integer old_basis_file = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"OldBasisFile")));
  snopt::snseti_(strOpt13,&old_basis_file,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt14[200] = "Backup basis file";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt14));
  snopt::integer backup_basis_file = static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"BackupBasisFile")));
  snopt::snseti_(strOpt14,&backup_basis_file,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  char strOpt15[200] = "Linesearch tolerance";
  strOpt_len = static_cast<snopt::integer>(strlen(strOpt15));
  snopt::doublereal line_search_tolerance= static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"LinesearchTolerance")));
  snopt::snsetr_(strOpt15,&line_search_tolerance,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw,strOpt_len,8*lencw);

  snopt::snopta_
    ( &Cold, &nF, &nx, &nxname, &nFname,
      &ObjAdd, &ObjRow, Prob, snopt_userfun,
      iAfun, jAvar, &lenA, &lenA, A,
      iGfun, jGvar, &lenG, &lenG,
      xlow, xupp, xnames, Flow, Fupp, Fnames,
      x, xstate, xmul, F, Fstate, Fmul,
      &INFO_snopt, &mincw, &miniw, &minrw,
      &nS, &nInf, &sInf,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      8*npname, 8*nxname, 8*nFname,
      8*lencw,8*lencw);

  plhs[0] = mxCreateDoubleMatrix(nx,1,mxREAL);
  plhs[3] = mxCreateDoubleMatrix(nx,1,mxREAL);
  for(snopt::integer i = 0;i<nx;i++)
  {
    *(mxGetPr(plhs[0])+i) = static_cast<double>(x[i]);
    *(mxGetPr(plhs[3])+i) = static_cast<double>(xmul[i]);
  }
  plhs[1] = mxCreateDoubleMatrix(nF,1,mxREAL);
  plhs[4] = mxCreateDoubleMatrix(nF,1,mxREAL);
  for(snopt::integer i = 0;i<nF;i++)
  {
    *(mxGetPr(plhs[1])+i) = static_cast<double>(F[i]);
    *(mxGetPr(prhs[4])+i) = static_cast<double>(Fmul[i]);
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

  if(print_file_name_len!= 0)
  {
    snopt::snclose_(&iPrint);
  }
  if (rw != rw_static) { delete[] rw; }
  if (iw != iw_static) { delete[] iw; }
  if (cw != cw_static) { delete[] cw; }
}
