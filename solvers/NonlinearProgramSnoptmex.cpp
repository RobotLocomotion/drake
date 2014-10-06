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

static int snopt_userfun_name_len;
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
  for(int i = 0;i<*n;i++)
  {
    *(mxGetPr(snopt_userfun_prhs[0])+i) = static_cast<double>(x[i]);
  }
  snopt_userfun_plhs[0] = mxCreateDoubleMatrix(*neF,1,mxREAL);
  snopt_userfun_plhs[1] = mxCreateDoubleMatrix(*neG,1,mxREAL);
  mexCallMATLAB(2,snopt_userfun_plhs,1,snopt_userfun_prhs,snopt_userfun_name);
  for(int i = 0;i<*neF;i++)
  {
    f[i] = static_cast<snopt::doublereal>( *(mxGetPr(snopt_userfun_plhs[0])+i));
  }
  for(int i = 0;i<*neG;i++)
  {
    G[i] = static_cast<snopt::doublereal>( *(mxGetPr(snopt_userfun_prhs[0])+i));
  }
  mxDestroyArray(snopt_userfun_plhs[0]);
  mxDestroyArray(snopt_userfun_plhs[1]);
  mxDestroyArray(snopt_userfun_prhs[0]);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 13 && nlhs != 5)
  {
    mexErrMsgIdAndTxt("Drake:NonlinearProgramSnoptmex:InvalidInputs","The usage is [x,F,info,xmul,Fmul] = NonlinearProgramSnoptmex(x,xlow,xupp,Flow,Fupp,userfun,ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar,options)");
  }
  int nx = mxGetM(prhs[0]);
  int nF = mxGetM(prhs[3]);
  snopt::doublereal* x    = new snopt::doublereal[nx];
  snopt::doublereal* xlow = new snopt::doublereal[nx];
  snopt::doublereal* xupp = new snopt::doublereal[nx];
  for(int i = 0;i<nx;i++)
  {
    x[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[0])+i));
    xlow[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[1])+i));
    xupp[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[2])+i));
  }
  snopt::doublereal* Flow = new snopt::doublereal[nF];
  snopt::doublereal* Fupp = new snopt::doublereal[nF];
  for(int i = 0;i<nx;i++)
  {
    Flow[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[3])+i));
    Fupp[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[4])+i));
  }
  snopt_userfun_name_len = mxGetNumberOfElements(prhs[5])+1;
  snopt_userfun_name = new char[userfun_name_len];
  int userfun_name_status = mxGetString(prhs[5],snopt_userfun_name,snopt_userfun_name_len);
  if(userfun_name_status != 0)
  {
    mexErrMsgIdAndTxt("Drake:NonlinearProgramSnoptmex:InvalidInputs","userfun should be a string for NonlinearProgramSnoptmex");
  }
  int lenA = mxGetNumberOfElements(prhs[8]);
  snopt::doublereal* A     = new snopt::doublereal[lenA];
  snopt::integer*    iAfun = new snopt::integer[lenA];
  snopt::integer*    jAvar = new snopt::integer[lenA];
  for(int i = 0;i<lenA;i++)
  {
    A[i] = static_cast<snopt::doublereal>(*(mxGetPr(prhs[8])+i));
    iAfun[i] = static_cast<snopt::integer>(*(mxGetPr(prhs[9])+i));
    jAvar[i] = static_cast<snopt::integer>(*(mxGetPr(prhs[10])+i));
  }
  int lenG = mxGetNumberOfElements(prhs[11]);
  snopt::integer* iGfun = new snopt::integer[lenG];
  snopt::integer* jGvar = new snopt::integer[lenG];
  for(int i = 0;i<lenG;i++)
  {
    iGfun[i] = static_cast<snopt::integer>(*(mxGetPr(prhs[11])+i));
    jGvar[i] = static_cast<snopt::integer>(*(mxGetPr(prhs[12])+i));
  }
  snopt::integer minrw,miniw,mincw;
  snopt::integer lenrw = 10000000, leniw = 500000, lencw = 500; 
  snopt::doublereal* rw = (snopt::doublereal*) std::calloc(lenrw,sizeof(snopt::doublereal));
  snopt::integer* iw = (snopt::integer*) std::calloc(leniw,sizeof(snopt::integer));
  char cw[8*lencw];

  snopt::integer Cold = 0;
  snopt::doublereal *xmul = new snopt::doublereal[nx];
  snopt::integer *xstate = new snopt::integer[nx];
  for(int i = 0;i<nx;i++)
  {
    xstate[i] = 0;
  }

  snopt::doublereal *F      = new snopt::doublereal[nF];
  snopt::doublereal *Fmul   = new snopt::doublereal[nF];
  snopt::integer    *Fstate = new snopt::integer[nF];
  for(int i = 0;i<nF;i++)
  {
    Fstate[i] = 0;
  }
  snopt::doublereal ObjAdd = static_cast<snopt::doublereal>(mxGetScalar(prhs[6]));
  snopt::integer ObjRow = static_cast<snopt::integer>(mxGetScalar(prhs[7]));
  
  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char* xnames[nxname*8];
  char* Fnames[nFname*8];
  char Prog[200];
  
  snopt::integer iSumm = -1;
  snopt::integer iPrint = -1;

  snopt::integer nS,nInf;
  snopt::doublereal sInf;
  snopt::integer INFO_snopt;

  snopt::sninit_(&iPring,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*500);
  snopt::snopta_
    ( &Cold, &nF, &nx, &nxname, &nFname,
      &ObjAdd, &ObjRow, Prob, snopt_userfun,
      iAfun, jAvar, &lenA, &lenA, A,
      iGfun, jGvar, &nG, &nG,
      xlow, xupp, xnames, Flow, Fupp, Fnames,
      x, xstate, xmul, F, Fstate, Fmul,
      &INFO_snopt, &mincw, &miniw, &minrw,
      &nS, &nInf, &sInf,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      cw, &lencw, iw, &leniw, rw, &lenrw,
      npname, 8*nxname, 8*nFname,
      8*500, 8*500);

  plhs[0] = mxCreateDoubleMatrix(nx,1,mxREAL);
  plhs[3] = mxCreateDoubleMatrix(nx,1,mxREAL);
  for(int i = 0;i<nx;i++)
  {
    *(mxGetPr(plhs[0])+i) = static_cast<double>(x[i]);
    *(mxGetPr(plhs[3])+i) = static_cast<double>(xmul[i]);
  }
  plhs[1] = mxCreateDoubleMatrix(nF,1,mxREAL);
  plhs[4] = mxCreateDoubleMatrix(nF,1,mxREAL);
  for(int i = 0;i<nF;i++)
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
  delete[] A,iAfun,jAvar;
  delete[] iGfun,jGvar;
  delete[] xmul,xstate;
  delete[] F,Fmul,Fstate;
}
