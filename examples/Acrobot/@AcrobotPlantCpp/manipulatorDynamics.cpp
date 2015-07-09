#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;
/*
 * c++ version of
 *     function [H,C,B] = manipulatorDynamics(obj,q,qd)
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) 
{
  const mxArray* pobj = prhs[0];
  Map<VectorXd> q(mxGetPr(prhs[1]), mxGetNumberOfElements(prhs[1]));
  Map<VectorXd> qd(mxGetPr(prhs[2]), mxGetNumberOfElements(prhs[2]));
  
  // keep it readable:
  double m1  = mxGetScalar(mxGetProperty(pobj,0,"m1"));
  double m2  = mxGetScalar(mxGetProperty(pobj,0,"m2"));
  double l1  = mxGetScalar(mxGetProperty(pobj,0,"l1"));
  double g   = mxGetScalar(mxGetProperty(pobj,0,"g"));
  double lc1 = mxGetScalar(mxGetProperty(pobj,0,"lc1"));
  double lc2 = mxGetScalar(mxGetProperty(pobj,0,"lc2"));
  double b1  = mxGetScalar(mxGetProperty(pobj,0,"b1"));
  double b2  = mxGetScalar(mxGetProperty(pobj,0,"b2"));
  double I1  = mxGetScalar(mxGetProperty(pobj,0,"Ic1")) + m1*lc1*lc1;
  double I2  = mxGetScalar(mxGetProperty(pobj,0,"Ic2")) + m2*lc2*lc2;
  double m2l1lc2 = m2*l1*lc2;  // occurs often!

  double c1 = cos(q(0)), c2 = cos(q(1));
  double s1 = sin(q(0)), s2 = sin(q(1));
  double s12 = sin(q(0)+q(1));
      
  plhs[0] = mxCreateDoubleMatrix(2,2,mxREAL);
  Map<Matrix2d> H(mxGetPr(plhs[0]));
  plhs[1] = mxCreateDoubleMatrix(2,1,mxREAL);
  Map<Vector2d> C(mxGetPr(plhs[1]),2);
  plhs[2] = mxCreateDoubleMatrix(2,1,mxREAL);
  Map<Vector2d> B(mxGetPr(plhs[2]),2);
  
  double h12 = I2 + m2l1lc2*c2;
  H << I1 + I2 + m2*l1*l1 + 2*m2l1lc2*c2, h12, h12, I2;

  //C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
  //G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
            
  Matrix2d Ctmp; Ctmp << -2*m2l1lc2*s2*qd(1), -m2l1lc2*s2*qd(1), m2l1lc2*s2*qd(0), 0;
  Vector2d G; G << g*m1*lc1*s1 + g*m2*(l1*s1+lc2*s12), g*m2*lc2*s12;
            
  // accumate total C and add a damping term:
  C = Ctmp*qd + G;
  C(0)+=b1*qd(0); C(1)+=b2*qd(1);

  B << 0, 1;
}