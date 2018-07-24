/*
The MIT License (MIT)

Copyright (c) 2014 Elizabeth Wong

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdlib.h>
#include <stdio.h>
#include "snopt_cwrap.h"

void toyusrf
( int *Status, int *n,   double x[],
  int *needF,  int *nF,  double F[],
  int *needG,  int *neG, double G[],
  char   cu[], int   *lencu,
  int    iu[], int   *leniu,
  double ru[], int   *lenru ) {
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // neF = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //==================================================================
  if ( *needF > 0 ) {
    F[0] =  x[1]; //  Objective row
    F[1] =  x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void toyusrfg
( int *Status, int *n,   double x[],
  int *needF,  int *nF,  double F[],
  int *needG,  int *neG, double G[],
  char   cu[], int   *lencu,
  int    iu[], int   *leniu,
  double ru[], int   *lenru ) {
  //==================================================================
  // Computes the nonlinear objective and constraint terms for the toy
  // problem featured in the SnoptA users guide.
  // neF = 3, n = 2.
  //
  //   Minimize     x(2)
  //
  //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
  //               (x(1) - 2)**2 +   x(2)**2  <= 5,
  //                x(1) >= 0.
  //==================================================================
  if ( *needF > 0 ) {
    F[0] =  x[1]; //  Objective row
    F[1] =  x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }

  if ( *needG > 0 ) {
    // iGfun[0] = 1
    // jGvar[0] = 0
    G[0] = 2*x[0];

    // iGfun[1] = 1
    // jGvar[1] = 1
    G[1] = 8*x[1];

    // iGfun[2] = 2
    // jGvar[2] = 0
    G[2] = 2*(x[0] - 2);

    // iGfun[3] = 2
    // jGvar[3] = 1
    G[3] = 2*x[1];
  }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main( int argc , char* argv[] )
{
  snProblem toy;

  int    i, info;
  int    Cold   =  0;
  int    n      =  2;
  int    nF     =  3;
  int    neA;
  int    neG;
  int    ObjRow =  0;
  double ObjAdd =  0;

  int    iAfun[6], jAvar[6], iGfun[6], jGvar[6];
  double A[6];

  int    xstate[2];
  double x[2], xlow[2], xupp[2], xmul[2];

  int    Fstate[3];
  double F[3], Flow[3], Fupp[3], Fmul[3];

  int    nS, nInf;
  double sInf;

  // snInit must be called first.
  //   9, 6 are print and summary unit numbers (for Fortran).
  //   6 == standard out
  snInit( &toy, "ToyA", "ToyA.out", 1 );


  // User workspace allocated and set.
  //   May be accesed in the user-defined function toyusrfun.
  toy.leniu = 2;
  toy.iu = (int*)malloc( sizeof(int) * toy.leniu );
  toy.iu[0] = 0;
  toy.iu[1] = 1;


  // Set bounds
  xlow[0] =     0;  xupp[0] = 1e20;
  xlow[1] = -1e20;  xupp[1] = 1e20;

  Flow[0] = -1e20;  Fupp[0] = 1e20;
  Flow[1] = -1e20;  Fupp[1] =    4;
  Flow[2] = -1e20;  Fupp[2] =    5;

  // Initialize states, x and multipliers
  for ( i = 0; i < n; i++ ) {
    xstate[i] = 0;
    x[i]      = 0;
    xmul[i]   = 0;
  }

  for ( i = 0; i < nF; i++ ) {
    Fstate[i] = 0;
    F[i]      = 0;
    Fmul[i]   = 0;
  }

  x[0] = 1.0;
  x[1] = 1.0;


  // Read options.
  // info = setSpecsfile( &toy, "sntoy.spc" );

  // Let snJac estimate the Jacobian structure
  info = snJac( &toy, nF, n, toyusrf,
		x, xlow, xupp,
		&neA, iAfun, jAvar, A,
		&neG, iGfun, jGvar );

  // Solve the problem
  info = solveA( &toy, Cold,
		 nF, n, ObjAdd, ObjRow, toyusrf,
		 neA, iAfun, jAvar, A,
		 neG, iGfun, jGvar,
		 xlow, xupp, Flow, Fupp,
		 x, xstate, xmul, F, Fstate, Fmul,
		 &nS, &nInf, &sInf);

  // Try solving with explicit Jacobian structure now
  // Re-initialize states, x and multipliers
  for ( i = 0; i < n; i++ ) {
    xstate[i] = 0;
    x[i]      = 0;
    xmul[i]   = 0;
  }

  for ( i = 0; i < nF; i++ ) {
    Fstate[i] = 0;
    F[i]      = 0;
    Fmul[i]   = 0;
  }

  x[0] = 1.0;
  x[1] = 1.0;

  // Set up the Jacobian matrix
  iGfun[0] = 1;
  jGvar[0] = 0;

  iGfun[1] = 1;
  jGvar[1] = 1;

  iGfun[2] = 2;
  jGvar[2] = 0;

  iGfun[3] = 2;
  jGvar[3] = 1;
  neG      = 4;

  iAfun[0] = 0;
  jAvar[0] = 1;
  A[0]     = 1.0;
  neA      = 1;

  // Read options, set options.
  // info = setSpecsfile( &toy, "sntoy.spc" );
  setIntParameter( &toy, "Verify level", 3 );
  setIntParameter( &toy, "Derivative option", 3 );

  info = solveA( &toy, Cold,
		 nF, n, ObjAdd, ObjRow, toyusrfg,
		 neA, iAfun, jAvar, A,
		 neG, iGfun, jGvar,
		 xlow, xupp, Flow, Fupp,
		 x, xstate, xmul, F, Fstate, Fmul,
		 &nS, &nInf, &sInf);

  // Deallocate space.
  free( toy.iu );
  deleteSNOPT( &toy );

  return 0;
}
